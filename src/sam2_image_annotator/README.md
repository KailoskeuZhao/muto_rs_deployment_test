# SAM 2 Image Annotator

This package subscribes to a camera image, detects objects with Ultralytics YOLO,
and uses each detection box as a SAM 2 prompt. It publishes tagged object
overlays, a binary union mask, an instance-ID mask, per-object metadata, and an
instance-marked 3D point cloud.

SAM 2 computes the image embedding once per frame and refines every YOLO box
against that embedding.

The SAM 2 Python package, PyTorch, Ultralytics, model configs, and weights are
runtime dependencies. They are intentionally imported lazily so this ROS
workspace can still build on machines that do not have the inference stack
installed.

The default checkpoint is `checkpoints/sam2.1_hiera_base_plus.pt`. Relative
checkpoint paths are checked from both the launch working directory and the SAM
2 project root, so an editable SAM 2 installation at `/opt/sam2` resolves the
default to `/opt/sam2/checkpoints/sam2.1_hiera_base_plus.pt`. An absolute path can
always be supplied explicitly:

```bash
ros2 launch sam2_image_annotator sam2_image_annotator_launch.py \
  checkpoint:=/path/to/sam2.1_hiera_base_plus.pt
```

```bash
ros2 launch sam2_image_annotator sam2_image_annotator_launch.py
```

The default detector is `yolo26m.pt`. Ultralytics downloads named weights if
they are not cached, or an existing weights path can be supplied:

```bash
ros2 launch sam2_image_annotator sam2_image_annotator_launch.py \
  yolo_model:=/path/to/model.pt yolo_confidence:=0.35
```

`yolo_quantize:=fp16` is the default CUDA inference precision. Set it to
`fp32` when full precision is required.

Use `yolo_classes` to limit processing to selected class IDs. For example,
COCO class IDs `0,2` process people and cars:

```bash
ros2 launch sam2_image_annotator sam2_image_annotator_launch.py \
  yolo_classes:="0,2"
```

On Jetson, the launch file preloads Ubuntu's OpenBLAS for this node only to
avoid a `gotoblas` symbol collision with NVIDIA's fixed-target OpenBLAS. The
library path can be overridden when needed:

```bash
ros2 launch sam2_image_annotator sam2_image_annotator_launch.py \
  openblas_preload:=/path/to/libopenblas.so.0
```

Manual SAM 2 prompts remain available for diagnostics:

```bash
ros2 launch sam2_image_annotator sam2_image_annotator_launch.py \
  prompt_mode:=manual point_coords:="320,240" point_labels:="1"

ros2 launch sam2_image_annotator sam2_image_annotator_launch.py \
  prompt_mode:=manual box:="120,80,520,420" default_prompt:=none
```

Default topics:

| Topic | Type | Role |
| --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/Image` | Input RGB image. |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Color intrinsics used to project depth points into the mask. |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Input 16UC1 depth image. |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | Depth intrinsics used for back-projection. |
| `/sam2/annotated_image` | `sensor_msgs/Image` | Tagged YOLO boxes and refined mask overlays. |
| `/sam2/mask` | `sensor_msgs/Image` | Mono8 union of all refined masks. |
| `/sam2/instance_mask` | `sensor_msgs/Image` | 16UC1 image; each nonzero value is an object instance ID. |
| `/sam2/segments` | `std_msgs/String` | JSON metadata for every refined object. |
| `/sam2/instance_pointcloud` | `sensor_msgs/PointCloud2` | Depth-frame XYZ points with `instance_id` and `rgb` fields. |

Each object in `/sam2/segments` contains `instance_id`, `class_id`,
`label`, YOLO `confidence`, `box_xyxy`, `sam_score`, and `mask_area`.
The `instance_id` matches the pixel value in `/sam2/instance_mask`.

For the instance point cloud, the node back-projects the raw depth image with
the depth intrinsics, obtains the depth-optical to color-optical transform from
the live TF tree, and projects each 3D depth point into the color mask with the
color intrinsics and distortion coefficients. Only points landing on a nonzero
instance ID are published. `depth_scale` converts uint16 values to metres,
`depth_sync_tolerance` rejects stale color/depth pairs, `tf_timeout` controls TF
lookup time, and `pointcloud_stride` can reduce the point count.
