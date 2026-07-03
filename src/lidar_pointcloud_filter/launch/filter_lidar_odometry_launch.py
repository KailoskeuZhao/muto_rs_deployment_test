from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    return LaunchDescription([
        input_topic_arg,
        filtered_topic_arg,
        odom_topic_arg,
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
            }],
        ),
    ])
