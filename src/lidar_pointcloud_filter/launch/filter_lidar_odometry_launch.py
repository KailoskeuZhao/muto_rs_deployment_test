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
    target_frame_arg = DeclareLaunchArgument(
        "target_frame",
        default_value="base_frame",
        description="Frame that filtered point clouds are transformed into.",
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="scan_odom",
        description="Odometry output topic.",
    )
    odom_child_frame_arg = DeclareLaunchArgument(
        "odom_child_frame",
        default_value="base_frame",
        description="Child frame for the odometry output.",
    )
    queue_size_arg = DeclareLaunchArgument(
        "queue_size",
        default_value="5",
        description="Point cloud and odometry queue size.",
    )
    voxel_leaf_size_arg = DeclareLaunchArgument(
        "voxel_leaf_size",
        default_value="0.02",
        description="LiDAR filter voxel leaf size in meters. 0.0 disables downsampling.",
    )

    return LaunchDescription([
        input_topic_arg,
        filtered_topic_arg,
        target_frame_arg,
        odom_topic_arg,
        odom_child_frame_arg,
        queue_size_arg,
        voxel_leaf_size_arg,
        Node(
            package="lidar_pointcloud_filter",
            executable="lidar_pointcloud_filter_node",
            name="lidar_pointcloud_filter_node",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("filtered_topic"),
                "target_frame": LaunchConfiguration("target_frame"),
                "voxel_leaf_size": ParameterValue(
                    LaunchConfiguration("voxel_leaf_size"),
                    value_type=float,
                ),
                "queue_size": ParameterValue(LaunchConfiguration("queue_size"), value_type=int),
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
                "odom_child_frame_id": LaunchConfiguration("odom_child_frame"),
                "expected_cloud_frame_id": LaunchConfiguration("target_frame"),
                "queue_size": ParameterValue(LaunchConfiguration("queue_size"), value_type=int),
            }],
        ),
    ])
