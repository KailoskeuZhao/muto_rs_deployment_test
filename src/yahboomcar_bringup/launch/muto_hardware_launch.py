from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    imu_gyro_lsb_per_dps_arg = DeclareLaunchArgument(
        "imu_gyro_lsb_per_dps",
        default_value="65.5",
        description="Raw gyro counts per degree/second used for processed IMU angular velocity.",
    )

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
        parameters=[{
            "imu_gyro_lsb_per_dps": ParameterValue(
                LaunchConfiguration("imu_gyro_lsb_per_dps"),
                value_type=float,
            ),
        }],
    )

    return LaunchDescription([
        imu_gyro_lsb_per_dps_arg,
        lidar_node,
        camera_launch,
        driver_node,
    ])
