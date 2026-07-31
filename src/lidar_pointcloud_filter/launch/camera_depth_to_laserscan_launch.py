from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            'depth_image_topic',
            default_value='/camera/depth/image_raw',
            description='Input 16UC1 camera depth image topic.',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/depth/camera_info',
            description='Depth camera CameraInfo topic used for back-projection.',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/camera/filtered_laserscan',
            description='Narrow depth-camera LaserScan used as a Nav2 observation source.',
        ),
        DeclareLaunchArgument(
            'processing_frame',
            default_value='base_frame',
            description='Frame used for projection, height filtering, and scan output.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'min_z',
            default_value='-0.07',
            description='Minimum processing-frame z value to keep.',
        ),
        DeclareLaunchArgument(
            'max_z',
            default_value='0.18',
            description='Maximum processing-frame z value to keep.',
        ),
        DeclareLaunchArgument(
            'camera_min_x',
            default_value='0.30',
            description='Minimum processing-frame x value for camera points.',
        ),
        DeclareLaunchArgument(
            'range_min',
            default_value='0.30',
            description='Minimum generated camera scan range in meters.',
        ),
        DeclareLaunchArgument(
            'range_max',
            default_value='3.0',
            description='Maximum generated camera scan range in meters.',
        ),
        DeclareLaunchArgument(
            'horizontal_fov',
            default_value='1.0192722831646884',
            description='Depth camera horizontal field of view in radians (58.4 degrees).',
        ),
        DeclareLaunchArgument(
            'vertical_fov',
            default_value='0.7941248096574199',
            description='Depth camera vertical field of view in radians (45.5 degrees).',
        ),
        DeclareLaunchArgument(
            'angle_min',
            default_value='-0.5096361415823442',
            description='Minimum output scan angle in radians (-29.2 degrees).',
        ),
        DeclareLaunchArgument(
            'angle_max',
            default_value='0.5096361415823442',
            description='Maximum output scan angle in radians (29.2 degrees).',
        ),
        DeclareLaunchArgument(
            'angle_increment',
            default_value='0.017453292519943295',
            description=(
                'Target camera LaserScan angular resolution in radians '
                '(approximately 1 degree).'
            ),
        ),
        DeclareLaunchArgument(
            'queue_size',
            default_value='1',
            description='Depth image, CameraInfo, and output queue size.',
        ),
        DeclareLaunchArgument(
            'max_publish_rate',
            default_value='7.0',
            description=(
                'Maximum camera LaserScan rate in Hz. Set 0.0 to process every '
                'depth image.'
            ),
        ),
        DeclareLaunchArgument(
            'pixel_stride_x',
            default_value='4',
            description='Depth-image block width; only its nearest valid pixel is projected.',
        ),
        DeclareLaunchArgument(
            'pixel_stride_y',
            default_value='4',
            description='Depth-image block height; only its nearest valid pixel is projected.',
        ),
        DeclareLaunchArgument(
            'depth_scale',
            default_value='0.001',
            description='Scale from 16UC1 depth units to meters.',
        ),
        DeclareLaunchArgument(
            'restamp_output',
            default_value='false',
            description=(
                'Use current ROS time for generated scan stamps. Leave false unless '
                'input driver stamps are known bad while the data is fresh.'
            ),
        ),
        DeclareLaunchArgument(
            'input_stamp_warning_age',
            default_value='1.0',
            description=(
                'Warn when an input depth-image stamp differs from this node clock '
                'by more seconds.'
            ),
        ),
        DeclareLaunchArgument(
            'max_input_age',
            default_value='2.0',
            description=(
                'Drop input depth images whose stamp differs from this node clock by '
                'more seconds. Set 0.0 only for intentionally non-live stamps.'
            ),
        ),
        DeclareLaunchArgument(
            'processing_time_warning',
            default_value='0.05',
            description='Warn when one depth-to-scan conversion exceeds this many seconds.',
        ),
        DeclareLaunchArgument(
            'transform_timeout',
            default_value='0.05',
            description='TF lookup timeout in seconds.',
        ),
    ]

    converter = Node(
        package='lidar_pointcloud_filter',
        executable='camera_depth_to_laserscan_node',
        name='camera_depth_to_laserscan_node',
        output='screen',
        parameters=[{
            'depth_image_topic': LaunchConfiguration('depth_image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'processing_frame': LaunchConfiguration('processing_frame'),
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'),
                value_type=bool,
            ),
            'min_z': ParameterValue(LaunchConfiguration('min_z'), value_type=float),
            'max_z': ParameterValue(LaunchConfiguration('max_z'), value_type=float),
            'camera_min_x': ParameterValue(
                LaunchConfiguration('camera_min_x'),
                value_type=float,
            ),
            'range_min': ParameterValue(
                LaunchConfiguration('range_min'),
                value_type=float,
            ),
            'range_max': ParameterValue(
                LaunchConfiguration('range_max'),
                value_type=float,
            ),
            'horizontal_fov': ParameterValue(
                LaunchConfiguration('horizontal_fov'),
                value_type=float,
            ),
            'vertical_fov': ParameterValue(
                LaunchConfiguration('vertical_fov'),
                value_type=float,
            ),
            'angle_min': ParameterValue(
                LaunchConfiguration('angle_min'),
                value_type=float,
            ),
            'angle_max': ParameterValue(
                LaunchConfiguration('angle_max'),
                value_type=float,
            ),
            'angle_increment': ParameterValue(
                LaunchConfiguration('angle_increment'),
                value_type=float,
            ),
            'queue_size': ParameterValue(
                LaunchConfiguration('queue_size'),
                value_type=int,
            ),
            'max_publish_rate': ParameterValue(
                LaunchConfiguration('max_publish_rate'),
                value_type=float,
            ),
            'pixel_stride_x': ParameterValue(
                LaunchConfiguration('pixel_stride_x'),
                value_type=int,
            ),
            'pixel_stride_y': ParameterValue(
                LaunchConfiguration('pixel_stride_y'),
                value_type=int,
            ),
            'depth_scale': ParameterValue(
                LaunchConfiguration('depth_scale'),
                value_type=float,
            ),
            'restamp_output': ParameterValue(
                LaunchConfiguration('restamp_output'),
                value_type=bool,
            ),
            'input_stamp_warning_age': ParameterValue(
                LaunchConfiguration('input_stamp_warning_age'),
                value_type=float,
            ),
            'max_input_age': ParameterValue(
                LaunchConfiguration('max_input_age'),
                value_type=float,
            ),
            'processing_time_warning': ParameterValue(
                LaunchConfiguration('processing_time_warning'),
                value_type=float,
            ),
            'transform_timeout': ParameterValue(
                LaunchConfiguration('transform_timeout'),
                value_type=float,
            ),
        }],
    )

    return LaunchDescription([*arguments, converter])
