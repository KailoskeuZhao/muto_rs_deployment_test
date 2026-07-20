from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/camera/color/image_raw",
        description="Input RGB image topic.",
    )
    annotated_topic_arg = DeclareLaunchArgument(
        "annotated_topic",
        default_value="/sam2/annotated_image",
        description="Output annotated image topic.",
    )
    mask_topic_arg = DeclareLaunchArgument(
        "mask_topic",
        default_value="/sam2/mask",
        description="Output mono8 mask topic.",
    )
    instance_mask_topic_arg = DeclareLaunchArgument(
        "instance_mask_topic",
        default_value="/sam2/instance_mask",
        description="Output 16UC1 instance-ID mask topic.",
    )
    segments_topic_arg = DeclareLaunchArgument(
        "segments_topic",
        default_value="/sam2/segments",
        description="Output JSON object-segmentation results topic.",
    )
    checkpoint_arg = DeclareLaunchArgument(
        "checkpoint",
        default_value="checkpoints/sam2.1_hiera_base_plus.pt",
        description=(
            "SAM 2 checkpoint path. Relative paths are checked from the current "
            "working directory and the SAM 2 project root."
        ),
    )
    openblas_preload_arg = DeclareLaunchArgument(
        "openblas_preload",
        default_value="/usr/lib/aarch64-linux-gnu/libopenblas.so.0",
        description=(
            "OpenBLAS library preloaded for this node to avoid mixing the Jetson "
            "and Ubuntu OpenBLAS implementations."
        ),
    )
    model_cfg_arg = DeclareLaunchArgument(
        "model_cfg",
        default_value="configs/sam2.1/sam2.1_hiera_b+.yaml",
        description="SAM 2 model config path.",
    )
    device_arg = DeclareLaunchArgument(
        "device",
        default_value="cuda",
        description="Torch device used for SAM 2 inference.",
    )
    prompt_mode_arg = DeclareLaunchArgument(
        "prompt_mode",
        default_value="yolo",
        description="Prompt source: yolo or manual.",
    )
    yolo_model_arg = DeclareLaunchArgument(
        "yolo_model",
        default_value="yolo11n.pt",
        description="Ultralytics YOLO model name or weights path.",
    )
    yolo_device_arg = DeclareLaunchArgument(
        "yolo_device",
        default_value="0",
        description="Ultralytics inference device, such as 0 or cpu.",
    )
    yolo_confidence_arg = DeclareLaunchArgument(
        "yolo_confidence",
        default_value="0.25",
        description="Minimum YOLO detection confidence.",
    )
    yolo_iou_arg = DeclareLaunchArgument(
        "yolo_iou",
        default_value="0.7",
        description="YOLO non-maximum suppression IoU threshold.",
    )
    yolo_imgsz_arg = DeclareLaunchArgument(
        "yolo_imgsz",
        default_value="640",
        description="YOLO square inference size.",
    )
    yolo_max_detections_arg = DeclareLaunchArgument(
        "yolo_max_detections",
        default_value="20",
        description="Maximum number of boxes refined by SAM 2 per frame.",
    )
    yolo_classes_arg = DeclareLaunchArgument(
        "yolo_classes",
        default_value="",
        description="Optional comma-separated YOLO class IDs.",
    )
    yolo_half_arg = DeclareLaunchArgument(
        "yolo_half",
        default_value="true",
        description="Use FP16 for YOLO when running on CUDA.",
    )
    point_coords_arg = DeclareLaunchArgument(
        "point_coords",
        default_value="",
        description="Prompt points as 'x,y;x,y'. Empty uses default_prompt.",
    )
    point_labels_arg = DeclareLaunchArgument(
        "point_labels",
        default_value="",
        description="Prompt labels as '1,0'. 1 is positive, 0 is negative.",
    )
    box_arg = DeclareLaunchArgument(
        "box",
        default_value="",
        description="Optional box prompt as 'x1,y1,x2,y2'.",
    )
    default_prompt_arg = DeclareLaunchArgument(
        "default_prompt",
        default_value="center_point",
        description="Fallback prompt when no point or box is configured: center_point or none.",
    )
    max_publish_rate_arg = DeclareLaunchArgument(
        "max_publish_rate",
        default_value="1.0",
        description="Maximum processed image rate in Hz. Set 0.0 to process every image.",
    )

    return LaunchDescription([
        image_topic_arg,
        annotated_topic_arg,
        mask_topic_arg,
        instance_mask_topic_arg,
        segments_topic_arg,
        checkpoint_arg,
        openblas_preload_arg,
        model_cfg_arg,
        device_arg,
        prompt_mode_arg,
        yolo_model_arg,
        yolo_device_arg,
        yolo_confidence_arg,
        yolo_iou_arg,
        yolo_imgsz_arg,
        yolo_max_detections_arg,
        yolo_classes_arg,
        yolo_half_arg,
        point_coords_arg,
        point_labels_arg,
        box_arg,
        default_prompt_arg,
        max_publish_rate_arg,
        Node(
            package="sam2_image_annotator",
            executable="sam2_image_annotator_node",
            name="sam2_image_annotator",
            output="screen",
            additional_env={
                "LD_PRELOAD": LaunchConfiguration("openblas_preload"),
            },
            parameters=[{
                "image_topic": LaunchConfiguration("image_topic"),
                "annotated_topic": LaunchConfiguration("annotated_topic"),
                "mask_topic": LaunchConfiguration("mask_topic"),
                "instance_mask_topic": LaunchConfiguration("instance_mask_topic"),
                "segments_topic": LaunchConfiguration("segments_topic"),
                "checkpoint": LaunchConfiguration("checkpoint"),
                "model_cfg": LaunchConfiguration("model_cfg"),
                "device": LaunchConfiguration("device"),
                "prompt_mode": LaunchConfiguration("prompt_mode"),
                "yolo_model": LaunchConfiguration("yolo_model"),
                "yolo_device": LaunchConfiguration("yolo_device"),
                "yolo_confidence": ParameterValue(
                    LaunchConfiguration("yolo_confidence"),
                    value_type=float,
                ),
                "yolo_iou": ParameterValue(
                    LaunchConfiguration("yolo_iou"),
                    value_type=float,
                ),
                "yolo_imgsz": ParameterValue(
                    LaunchConfiguration("yolo_imgsz"),
                    value_type=int,
                ),
                "yolo_max_detections": ParameterValue(
                    LaunchConfiguration("yolo_max_detections"),
                    value_type=int,
                ),
                "yolo_classes": LaunchConfiguration("yolo_classes"),
                "yolo_half": ParameterValue(
                    LaunchConfiguration("yolo_half"),
                    value_type=bool,
                ),
                "point_coords": LaunchConfiguration("point_coords"),
                "point_labels": LaunchConfiguration("point_labels"),
                "box": LaunchConfiguration("box"),
                "default_prompt": LaunchConfiguration("default_prompt"),
                "max_publish_rate": ParameterValue(
                    LaunchConfiguration("max_publish_rate"),
                    value_type=float,
                ),
            }],
        ),
    ])
