from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="scan_odom",
        description="Odometry topic used by odom_publisher.",
    )

    return LaunchDescription([
        odom_topic_arg,
        Node(
            package="tf2_publisher",
            executable="base_to_camera_publisher",
            name="camera_tf2_broadcaster",
            output="screen",
        ),
        Node(
            package="tf2_publisher",
            executable="base_to_lidar_publisher",
            name="lidar_tf2_broadcaster",
            output="screen",
        ),
        Node(
            package="tf2_publisher",
            executable="base_to_imu_publisher",
            name="imu_tf2_broadcaster",
            output="screen",
        ),
        Node(
            package="tf2_publisher",
            executable="odom_publisher",
            name="odom_tf2_broadcaster",
            output="screen",
            parameters=[{"odom_topic": LaunchConfiguration("odom_topic")}],
        ),
    ])
