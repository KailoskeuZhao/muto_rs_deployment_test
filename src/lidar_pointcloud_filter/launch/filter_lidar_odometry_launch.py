from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    input_topic_arg = DeclareLaunchArgument(
        "input_topic",
        default_value="lidar/PointCloud",
        description="Raw LiDAR PointCloud2 topic.",
    )
    filtered_topic_arg = DeclareLaunchArgument(
        "filtered_topic",
        default_value="lidar/PointCloudFiltered",
        description="Filtered LiDAR PointCloud2 topic consumed by odometry.",
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="scan_odom",
        description="Odometry output topic.",
    )
    base_to_lidar_x_arg = DeclareLaunchArgument(
        "base_to_lidar_x",
        default_value="-0.02",
        description="Planar x offset of lidar_frame in base_frame, in meters.",
    )
    base_to_lidar_y_arg = DeclareLaunchArgument(
        "base_to_lidar_y",
        default_value="0.0",
        description="Planar y offset of lidar_frame in base_frame, in meters.",
    )
    base_to_lidar_z_arg = DeclareLaunchArgument(
        "base_to_lidar_z",
        default_value="0.0",
        description="Z offset of lidar_frame in base_frame, in meters.",
    )
    base_to_lidar_roll_arg = DeclareLaunchArgument(
        "base_to_lidar_roll",
        default_value="0.0",
        description="Roll of lidar_frame in base_frame, in radians.",
    )
    base_to_lidar_pitch_arg = DeclareLaunchArgument(
        "base_to_lidar_pitch",
        default_value="-3.1415",
        description="Pitch of lidar_frame in base_frame, in radians.",
    )
    base_to_lidar_yaw_arg = DeclareLaunchArgument(
        "base_to_lidar_yaw",
        default_value="0.20",
        description="Yaw of lidar_frame in base_frame, in radians.",
    )

    return LaunchDescription([
        input_topic_arg,
        filtered_topic_arg,
        odom_topic_arg,
        base_to_lidar_x_arg,
        base_to_lidar_y_arg,
        base_to_lidar_z_arg,
        base_to_lidar_roll_arg,
        base_to_lidar_pitch_arg,
        base_to_lidar_yaw_arg,
        Node(
            package="lidar_pointcloud_filter",
            executable="lidar_pointcloud_filter_node",
            name="lidar_pointcloud_filter_node",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("filtered_topic"),
            }],
        ),
        Node(
            package="lidar_odometry",
            executable="lidar_odometry_node",
            name="lidar_odometry_node",
            output="screen",
            parameters=[{
                "point_cloud_topic_name": LaunchConfiguration("filtered_topic"),
                "odom_topic_name": LaunchConfiguration("odom_topic"),
                "base_to_lidar_x": ParameterValue(LaunchConfiguration("base_to_lidar_x"), value_type=float),
                "base_to_lidar_y": ParameterValue(LaunchConfiguration("base_to_lidar_y"), value_type=float),
                "base_to_lidar_z": ParameterValue(LaunchConfiguration("base_to_lidar_z"), value_type=float),
                "base_to_lidar_roll": ParameterValue(LaunchConfiguration("base_to_lidar_roll"), value_type=float),
                "base_to_lidar_pitch": ParameterValue(LaunchConfiguration("base_to_lidar_pitch"), value_type=float),
                "base_to_lidar_yaw": ParameterValue(LaunchConfiguration("base_to_lidar_yaw"), value_type=float),
            }],
        ),
    ])
