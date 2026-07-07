from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lidar_node = Node(
        package="lidar_tg30",
        executable="lidar_node",
        name="lidar_node",
        output="screen",
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("orbbec_camera"),
                "launch",
                "astra_pro_plus.launch.py",
            ])
        )
    )

    driver_node = Node(
        package="yahboomcar_bringup",
        executable="muto_driver",
        name="muto_driver",
        output="screen",
    )

    return LaunchDescription([
        lidar_node,
        camera_launch,
        driver_node,
    ])
