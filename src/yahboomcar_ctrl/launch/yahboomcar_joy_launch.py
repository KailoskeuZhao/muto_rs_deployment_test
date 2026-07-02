from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
    )
    yahboom_joy = Node(
        package='yahboomcar_ctrl',
        executable='yahboom_joy',
    )
    return LaunchDescription([
        joy_node,
        yahboom_joy,
    ])
