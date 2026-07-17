# SAM 2 Image Annotator

This package subscribes to a camera image, runs SAM 2 image prediction with a
point or box prompt, overlays the selected mask on the input image, and publishes
both the annotated image and a mono8 mask.

The SAM 2 Python package, PyTorch, model config, and checkpoint are runtime
dependencies. They are intentionally imported lazily so this ROS workspace can
still build on machines that do not have SAM 2 installed.

```bash
ros2 launch sam2_image_annotator sam2_image_annotator_launch.py
```

Useful prompt examples:

```bash
ros2 launch sam2_image_annotator sam2_image_annotator_launch.py \
  point_coords:="320,240" point_labels:="1"

ros2 launch sam2_image_annotator sam2_image_annotator_launch.py \
  box:="120,80,520,420" default_prompt:=none
```

Default topics:

| Topic | Type | Role |
| --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/Image` | Input RGB image. |
| `/sam2/annotated_image` | `sensor_msgs/Image` | BGR annotated output image. |
| `/sam2/mask` | `sensor_msgs/Image` | Mono8 selected mask. |
