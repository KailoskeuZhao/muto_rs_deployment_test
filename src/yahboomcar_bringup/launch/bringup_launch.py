from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    driver_node = Node(
        package='yahboomcar_bringup',
        executable='muto_driver',
        name='muto_driver',
        output='screen',
    )

    return LaunchDescription([
        driver_node,
    ])
