from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    depth_image_topic_arg = DeclareLaunchArgument(
        'depth_image_topic',
        default_value='/camera/depth/image_raw',
        description='Input 16UC1 camera depth image topic.',
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/depth/camera_info',
        description='Depth camera CameraInfo topic used for back-projection.',
    )
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/fused/laserscan',
        description='Output fused LaserScan topic.',
    )
    camera_scan_topic_arg = DeclareLaunchArgument(
        'camera_scan_topic',
        default_value='/camera/filtered_laserscan',
        description='Intermediate downsampled camera LaserScan topic.',
    )
    lidar_scan_topic_arg = DeclareLaunchArgument(
        'lidar_scan_topic',
        default_value='/lidar/filtered_laserscan_no_downsample',
        description='Filtered no-downsample LiDAR LaserScan topic merged into the fused scan.',
    )
    processing_frame_arg = DeclareLaunchArgument(
        'processing_frame',
        default_value='base_frame',
        description='Frame used for camera projection, z filtering, and intermediate scan output.',
    )
    fused_scan_frame_arg = DeclareLaunchArgument(
        'fused_scan_frame',
        default_value='base_frame',
        description='Frame used for the final fused LaserScan output.',
    )
    require_lidar_scan_arg = DeclareLaunchArgument(
        'require_lidar_scan',
        default_value='true',
        description=(
            'Require LiDAR for output. LiDAR drives /fused/laserscan and remains '
            'available as a fallback when the camera scan is missing.'
        ),
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true.',
    )
    min_z_arg = DeclareLaunchArgument(
        'min_z',
        default_value='-0.10',
        description='Minimum processing-frame z value to keep.',
    )
    max_z_arg = DeclareLaunchArgument(
        'max_z',
        default_value='0.18',
        description='Maximum processing-frame z value to keep.',
    )
    camera_min_x_arg = DeclareLaunchArgument(
        'camera_min_x',
        default_value='0.30',
        description='Minimum processing-frame x value for depth-camera points.',
    )
    range_max_arg = DeclareLaunchArgument(
        'range_max',
        default_value='3.0',
        description='Maximum depth camera scan range in meters.',
    )
    lidar_range_max_arg = DeclareLaunchArgument(
        'lidar_range_max',
        default_value='15.0',
        description='Maximum LiDAR scan range in meters.',
    )
    angle_min_arg = DeclareLaunchArgument(
        'angle_min',
        default_value='-3.141592653589793',
        description='Minimum scan angle in radians. Default is full-circle -pi.',
    )
    angle_max_arg = DeclareLaunchArgument(
        'angle_max',
        default_value='3.141592653589793',
        description='Maximum scan angle in radians. Default is full-circle pi.',
    )
    camera_angle_increment_arg = DeclareLaunchArgument(
        'camera_angle_increment',
        default_value='0.017453292519943295',
        description='Intermediate camera LaserScan angular resolution in radians.',
    )
    fused_angle_increment_arg = DeclareLaunchArgument(
        'fused_angle_increment',
        default_value='0.004363323129985824',
        description='Final fused LaserScan angular resolution in radians.',
    )
    queue_size_arg = DeclareLaunchArgument(
        'queue_size',
        default_value='1',
        description='Depth image, CameraInfo, and scan queue size.',
    )
    max_publish_rate_arg = DeclareLaunchArgument(
        'max_publish_rate',
        default_value='7.0',
        description=(
            'Maximum generated camera LaserScan rate in Hz. Set 0.0 to process every '
            'input depth image.'
        ),
    )
    pixel_stride_x_arg = DeclareLaunchArgument(
        'pixel_stride_x',
        default_value='4',
        description='Depth-image block width; only the nearest valid pixel is projected.',
    )
    pixel_stride_y_arg = DeclareLaunchArgument(
        'pixel_stride_y',
        default_value='4',
        description='Depth-image block height; only the nearest valid pixel is projected.',
    )
    depth_scale_arg = DeclareLaunchArgument(
        'depth_scale',
        default_value='0.001',
        description='Scale from 16UC1 depth units to meters.',
    )
    max_lidar_age_arg = DeclareLaunchArgument(
        'max_lidar_age',
        default_value='0.5',
        description=(
            'Maximum timestamp difference between camera and LiDAR scans. Older '
            'camera scans are omitted while LiDAR-only output continues.'
        ),
    )
    restamp_output_arg = DeclareLaunchArgument(
        'restamp_output',
        default_value='false',
        description=(
            'Use current ROS time for generated LaserScan stamps. Leave false unless input '
            'driver stamps are known bad while the data is fresh.'
        ),
    )
    input_stamp_warning_age_arg = DeclareLaunchArgument(
        'input_stamp_warning_age',
        default_value='1.0',
        description=(
            'Warn when an input depth-image/scan stamp differs from this node clock '
            'by more seconds.'
        ),
    )
    max_input_age_arg = DeclareLaunchArgument(
        'max_input_age',
        default_value='2.0',
        description=(
            'Drop input depth images/scans whose stamp differs from this node clock by more seconds. '
            'Set 0.0 only when using intentionally non-live stamps.'
        ),
    )
    processing_time_warning_arg = DeclareLaunchArgument(
        'processing_time_warning',
        default_value='0.05',
        description='Warn when one LaserScan conversion takes longer than this many seconds.',
    )
    transform_timeout_arg = DeclareLaunchArgument(
        'transform_timeout',
        default_value='0.05',
        description='TF lookup timeout in seconds.',
    )

    return LaunchDescription([
        depth_image_topic_arg,
        camera_info_topic_arg,
        output_topic_arg,
        camera_scan_topic_arg,
        lidar_scan_topic_arg,
        processing_frame_arg,
        fused_scan_frame_arg,
        require_lidar_scan_arg,
        use_sim_time_arg,
        min_z_arg,
        max_z_arg,
        camera_min_x_arg,
        range_max_arg,
        lidar_range_max_arg,
        angle_min_arg,
        angle_max_arg,
        camera_angle_increment_arg,
        fused_angle_increment_arg,
        queue_size_arg,
        max_publish_rate_arg,
        pixel_stride_x_arg,
        pixel_stride_y_arg,
        depth_scale_arg,
        max_lidar_age_arg,
        restamp_output_arg,
        input_stamp_warning_age_arg,
        max_input_age_arg,
        processing_time_warning_arg,
        transform_timeout_arg,
        Node(
            package='lidar_pointcloud_filter',
            executable='camera_depth_to_laserscan_node',
            name='camera_depth_to_laserscan_node',
            output='screen',
            parameters=[{
                'depth_image_topic': LaunchConfiguration('depth_image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'output_topic': LaunchConfiguration('camera_scan_topic'),
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
                'range_max': ParameterValue(LaunchConfiguration('range_max'), value_type=float),
                'angle_min': ParameterValue(LaunchConfiguration('angle_min'), value_type=float),
                'angle_max': ParameterValue(LaunchConfiguration('angle_max'), value_type=float),
                'angle_increment': ParameterValue(
                    LaunchConfiguration('camera_angle_increment'),
                    value_type=float,
                ),
                'queue_size': ParameterValue(LaunchConfiguration('queue_size'), value_type=int),
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
        ),
        Node(
            package='lidar_pointcloud_filter',
            executable='laserscan_fusion_node',
            name='laserscan_fusion_node',
            output='screen',
            parameters=[{
                'camera_scan_topic': LaunchConfiguration('camera_scan_topic'),
                'lidar_scan_topic': LaunchConfiguration('lidar_scan_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'output_frame': LaunchConfiguration('fused_scan_frame'),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'),
                    value_type=bool,
                ),
                'range_max': ParameterValue(
                    LaunchConfiguration('lidar_range_max'),
                    value_type=float,
                ),
                'angle_min': ParameterValue(LaunchConfiguration('angle_min'), value_type=float),
                'angle_max': ParameterValue(LaunchConfiguration('angle_max'), value_type=float),
                'angle_increment': ParameterValue(
                    LaunchConfiguration('fused_angle_increment'),
                    value_type=float,
                ),
                'queue_size': ParameterValue(LaunchConfiguration('queue_size'), value_type=int),
                'max_lidar_age': ParameterValue(
                    LaunchConfiguration('max_lidar_age'),
                    value_type=float,
                ),
                'require_lidar': ParameterValue(
                    LaunchConfiguration('require_lidar_scan'),
                    value_type=bool,
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
        ),
    ])
