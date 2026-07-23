# SAM2 + YOLO Image Pipeline

This document describes the current `sam2_image_annotator` runtime: YOLO
detection, SAM2 segmentation, 2D products, and the optional depth-derived
instance point cloud.

## Data Flow

```text
/camera/color/image_raw
        |
        v
busy/rate guard -> YOLO -> confidence/box guard -> SAM2 box prompts
                                                      |
                       +------------------------------+-----------------+
                       |                              |                 |
                       v                              v                 v
                annotated image               mask products       metadata

latest depth + depth CameraInfo + color CameraInfo + depth-to-color TF
                       + 2D instance mask
                                  |
                                  v
                       instance point cloud
```

YOLO proposes boxes. The node applies its own confidence threshold and uses
each accepted box as a SAM2 prompt. SAM2 computes the image embedding once per
frame, then refines every accepted box into a mask. Missing depth inputs stop
only the point-cloud branch; the 2D outputs continue.

## Runtime Requirements

The package builds without loading the inference stack because SAM2, PyTorch,
and Ultralytics are imported lazily. Runtime inference requires:

- ROS 2 Humble with this workspace built and sourced.
- SAM2 and a compatible CUDA-enabled PyTorch build for aarch64.
- Ultralytics and a compatible YOLO weights file.
- A SAM2 checkpoint matching `model_cfg`.
- Color images for 2D output.
- A `16UC1` depth image, camera calibration, and depth-to-color TF for 3D
  output.

Pre-stage both model files on the robot. A named Ultralytics model can trigger
a download when it is not cached.

## Start

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch sam2_image_annotator sam2_image_annotator_launch.py
```

Use absolute model paths for repeatable deployment:

```bash
ros2 launch sam2_image_annotator sam2_image_annotator_launch.py \
  checkpoint:=/opt/sam2/checkpoints/sam2.1_hiera_base_plus.pt \
  yolo_model:=/opt/models/yolo26m.pt
```

The relative checkpoint is checked from the launch working directory and the
installed or editable SAM2 project root. The default model configuration is
`configs/sam2.1/sam2.1_hiera_b+.yaml`.

## Per-Frame Processing

1. The color callback drops a frame if another frame is being processed and
   applies `max_publish_rate` between processing start times. Inference time is
   not followed by an additional rate-limit cooldown.
2. The latest depth image and latest color/depth `CameraInfo` messages are
   snapshotted. There is no synchronized message filter.
3. The image is converted to BGR for YOLO and RGB for SAM2.
4. YOLO runs with its candidate confidence set to `0.0`. The node then applies
   `yolo_confidence`, rejects non-finite scores, clips boxes to the image, and
   drops invalid boxes.
5. SAM2 sets the RGB image once, then predicts one mask per accepted box. If
   multimask output is enabled, the highest-scoring mask is selected.
6. The node composes the overlay, union mask, instance mask, and metadata.
7. When valid depth inputs and TF are available, depth pixels are projected
   into the color mask to create the labeled point cloud.

Instance IDs start at `1` and restart on every processed frame. They are
frame-local IDs, not tracking IDs. If masks overlap, the earlier accepted
segment owns the overlap; later masks fill only pixels that are still zero.

## Topic Interface

Inputs:

| Default topic | Type | Purpose |
| --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | YOLO and SAM2 color input. |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Color calibration. |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | Raw `16UC1` depth. |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Depth calibration. |
| TF | `tf2` | Depth optical frame to color optical frame transform. |

Outputs:

| Default topic | Type | Purpose |
| --- | --- | --- |
| `/sam2/annotated_image` | `sensor_msgs/msg/Image` | BGR boxes, labels, scores, and mask overlays. |
| `/sam2/mask` | `sensor_msgs/msg/Image` | `mono8` union of refined masks. |
| `/sam2/instance_mask` | `sensor_msgs/msg/Image` | `16UC1` frame-local instance IDs. |
| `/sam2/segments` | `std_msgs/msg/String` | JSON image and segment metadata. |
| `/sam2/detections` | typed detection array | Structured segment metadata. |
| `/sam2/instance_pointcloud` | `sensor_msgs/msg/PointCloud2` | Depth-frame XYZ, instance ID, and color. |

Image and metadata outputs retain the color image header. The point cloud uses
the depth image timestamp and depth optical frame. Sensor subscriptions are
best-effort and volatile; output publishers have queue depth `2`.

## Metadata

`/sam2/segments` contains the color header, image dimensions, and an
`objects` array. Each segment has:

| Field | Meaning |
| --- | --- |
| `instance_id` | Value used in the instance mask and point cloud. |
| `class_id` | YOLO class index. |
| `label` | YOLO class label. |
| `confidence` | YOLO confidence after the node guard. |
| `box_xyxy` | Clipped box in `[x1, y1, x2, y2]` order. |
| `sam_score` | Score of the selected SAM2 mask. |
| `mask_area` | Refined mask area in pixels. |

A no-detection frame publishes valid empty image-sized masks and empty
metadata.

## Instance Point Cloud

The 3D branch requires:

- Depth encoding exactly `16UC1`.
- Color/depth timestamp offset no greater than `depth_sync_tolerance`.
- `CameraInfo` dimensions matching the images.
- Positive focal lengths in both camera matrices.
- Nonempty depth and color optical frame IDs.
- A usable depth-to-color transform.

TF is first requested at the depth timestamp and then, if that fails, at the
latest available time:

```text
lookup_transform(color_frame, depth_frame, depth_stamp)
```

Depth pixels are undistorted and back-projected in the depth frame. Points are
transformed for color-mask projection and nearest-Z visibility testing, while
published XYZ remains in the depth frame. Processing is split into 64-row
chunks to limit temporary allocations at high resolution.

Each mask is trimmed inward by `pointcloud_mask_trim_ratio` before 3D
selection. This reduces boundary depth contamination and does not alter the
published 2D mask. The stable depth grid is sampled using `pointcloud_stride`
before intrinsic and projection work.

The unorganized cloud has `height=1`, `point_step=20`, and these fields:

| Field | Offset | Type |
| --- | ---: | --- |
| `x` | 0 | `float32` |
| `y` | 4 | `float32` |
| `z` | 8 | `float32` |
| `instance_id` | 12 | `uint16` |
| `rgb` | 16 | `uint32` |

## Important Defaults

| Parameter | Default | Effect |
| --- | --- | --- |
| `device` | `cuda` | SAM2 device. |
| `use_autocast` | `true` | Enables SAM2 autocast. |
| `autocast_dtype` | `bfloat16` | SAM2 autocast type. |
| `yolo_model` | `yolo26m.pt` | YOLO weights name or path. |
| `yolo_device` | `0` | YOLO CUDA device. |
| `yolo_confidence` | `0.4` | Minimum score before SAM2. |
| `yolo_iou` | `0.7` | YOLO NMS IoU. |
| `yolo_imgsz` | `960` | YOLO inference size for the 1280x720 RGB stream. |
| `yolo_max_detections` | `20` | Maximum boxes refined per frame. |
| `yolo_classes` | empty | Allows every detector class. |
| `yolo_quantize` | `fp16` | YOLO CUDA precision; disabled on CPU. |
| `max_publish_rate` | `7.0` | Maximum processing starts per second; slower inference naturally lowers throughput. |
| `depth_scale` | `0.001` | Converts depth units to metres. |
| `depth_sync_tolerance` | `0.2` | Maximum color/depth offset in seconds. |
| `pointcloud_stride` | `6` | Selects every sixth stable depth-grid pixel before intrinsic and projection work. |
| `pointcloud_mask_trim_ratio` | `0.1` | Trims boundaries for 3D selection. |
| `tf_timeout` | `0.1` | TF timeout in seconds. |
| `queue_size` | `2` | Input queue depth. |

The launch exposes normal model, topic, detector, prompt, depth, TF, and rate
settings. The YAML also records internal defaults such as autocast, overlay
styling, passthrough behavior, and queue depth; the launch does not
automatically load that YAML.

Filter to selected class IDs:

```bash
ros2 launch sam2_image_annotator sam2_image_annotator_launch.py \
  yolo_classes:="0,2"
```

## aarch64 Notes

- `device:=cuda` and `yolo_device:=0` require compatible CUDA-enabled
  PyTorch and Ultralytics installations.
- The launch sets `LD_PRELOAD` to
  `/usr/lib/aarch64-linux-gnu/libopenblas.so.0` for this node. Override
  `openblas_preload` if the compatible library is elsewhere.
- Keep `max_publish_rate` bounded initially. Inference runs in the color
  callback, and busy frames are dropped rather than queued.
- YOLO defaults to `fp16`; SAM2 separately uses `bfloat16` autocast.
- Keep checkpoints and weights on local storage.

## Failure Behavior

- Import or model initialization failures are logged while the node keeps
  spinning.
- With `publish_passthrough_on_error:=true`, the node publishes an unavailable
  overlay with empty masks and metadata.
- Missing, stale, or invalid depth data, calibration mismatch, and TF failures
  skip only point-cloud publication.
- Rejected YOLO scores are reported in throttled batch and cumulative logs.

## Diagnostics

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /sam2/annotated_image
ros2 topic echo /sam2/segments --once
ros2 topic info /sam2/instance_pointcloud --verbose
ros2 run tf2_ros tf2_echo <color_optical_frame> <depth_optical_frame>
```

Manual prompts can isolate detector problems from SAM2 or camera problems:

```bash
ros2 launch sam2_image_annotator sam2_image_annotator_launch.py \
  prompt_mode:=manual point_coords:="320,240" point_labels:="1"

ros2 launch sam2_image_annotator sam2_image_annotator_launch.py \
  prompt_mode:=manual box:="120,80,520,420" default_prompt:=none
```
