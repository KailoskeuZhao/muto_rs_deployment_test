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
    checkpoint_arg = DeclareLaunchArgument(
        "checkpoint",
        default_value="checkpoints/sam2.1_hiera_large.pt",
        description=(
            "SAM 2 checkpoint path. Relative paths are checked from the current "
            "working directory and the SAM 2 project root."
        ),
    )
    model_cfg_arg = DeclareLaunchArgument(
        "model_cfg",
        default_value="configs/sam2.1/sam2.1_hiera_l.yaml",
        description="SAM 2 model config path.",
    )
    device_arg = DeclareLaunchArgument(
        "device",
        default_value="cuda",
        description="Torch device used for SAM 2 inference.",
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
        checkpoint_arg,
        model_cfg_arg,
        device_arg,
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
            parameters=[{
                "image_topic": LaunchConfiguration("image_topic"),
                "annotated_topic": LaunchConfiguration("annotated_topic"),
                "mask_topic": LaunchConfiguration("mask_topic"),
                "checkpoint": LaunchConfiguration("checkpoint"),
                "model_cfg": LaunchConfiguration("model_cfg"),
                "device": LaunchConfiguration("device"),
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
