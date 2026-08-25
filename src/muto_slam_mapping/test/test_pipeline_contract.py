import ast
import importlib.util
from pathlib import Path
import sys

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest
import yaml

sys.dont_write_bytecode = True


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT.parent
SENSOR_ROOT = SOURCE_ROOT / 'lidar_pointcloud_filter'
HARDWARE_ROOT = SOURCE_ROOT / 'yahboomcar_bringup'
ODOMETRY_BAG_ROOT = SOURCE_ROOT / 'muto_odometry_bag'
NAV2_BAG_ROOT = SOURCE_ROOT / 'muto_nav2_bag'

RETIRED_RUNTIME_IDENTIFIERS = (
    '/fused/laserscan',
    'laserscan_fusion_node',
    'launch_fused_laserscan',
    'fused_scan_',
    'fusion_lidar_scan_topic',
)


def _load_yaml(path):
    with path.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def _python_strings(path):
    tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    return {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }


def _launch_defaults(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    description = module.generate_launch_description()
    context = LaunchContext()
    defaults = {
        entity.name: perform_substitutions(context, entity.default_value)
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
        and entity.default_value is not None
    }
    nodes = [entity for entity in description.entities if isinstance(entity, Node)]
    return defaults, nodes


def _launch_configuration_names(path):
    tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    names = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not node.args:
            continue
        function_name = getattr(node.func, 'id', None)
        if function_name != 'LaunchConfiguration':
            continue
        first_argument = node.args[0]
        if isinstance(first_argument, ast.Constant):
            names.add(first_argument.value)
    return names


def test_slam_uses_full_resolution_lidar_directly():
    config = _load_yaml(
        PACKAGE_ROOT / 'config' / 'mapper_params_online_async.yaml'
    )
    parameters = config['slam_toolbox']['ros__parameters']

    assert parameters['scan_topic'] == (
        '/lidar/filtered_laserscan_no_downsample'
    )
    assert parameters['base_frame'] == 'base_frame'
    assert parameters['odom_frame'] == 'odom'
    assert parameters['map_frame'] == 'map'


def test_both_nav2_costmaps_use_independent_sensor_sources():
    config = _load_yaml(PACKAGE_ROOT / 'config' / 'nav2_params.yaml')

    for costmap_name in ('local_costmap', 'global_costmap'):
        parameters = config[costmap_name][costmap_name]['ros__parameters']
        layer = parameters['obstacle_layer']
        assert set(layer['observation_sources'].split()) == {'lidar', 'camera'}
        assert parameters['robot_radius'] == 0.26
        assert parameters['footprint_padding'] == 0.01
        assert layer['footprint_clearing_enabled'] is False

        lidar = layer['lidar']
        assert lidar['topic'] == '/lidar/filtered_laserscan'
        assert lidar['data_type'] == 'LaserScan'
        assert lidar['marking'] is True
        assert lidar['clearing'] is True
        assert lidar['inf_is_valid'] is True
        assert lidar['expected_update_rate'] == 0.2

        camera = layer['camera']
        assert camera['topic'] == '/camera/filtered_laserscan'
        assert camera['data_type'] == 'LaserScan'
        assert camera['marking'] is True
        assert camera['clearing'] is True
        assert camera['inf_is_valid'] is False
        assert camera.get('expected_update_rate', 0.0) == 0.0


def test_nav2_does_not_request_unsupported_lateral_turning():
    config = _load_yaml(PACKAGE_ROOT / 'config' / 'nav2_params.yaml')
    smoother = config['velocity_smoother']['ros__parameters']
    controller = config['controller_server']['ros__parameters']['FollowPath']
    behavior = config['behavior_server']['ros__parameters']

    assert smoother['max_velocity'][1] == 0.0
    assert smoother['min_velocity'][1] == 0.0
    assert smoother['max_velocity'][0] == 0.25
    assert smoother['min_velocity'][0] == -0.10
    assert smoother['max_velocity'][2] == 0.50
    assert smoother['min_velocity'][2] == -0.50
    assert smoother['max_accel'][0] == 0.15
    assert smoother['max_accel'][2] == 1.0
    assert smoother['max_decel'][0] == -0.2
    assert smoother['max_decel'][2] == -1.0
    assert controller['desired_linear_vel'] == 0.25
    assert controller['lookahead_dist'] == 0.40
    assert controller['rotate_to_heading_angular_vel'] == 0.30
    # Humble RPP clamps rotate-to-heading around measured yaw velocity before
    # the velocity smoother. Its internal limit must be non-binding across the
    # complete signed path-following yaw range; direct Spin owns the faster
    # 0.50 rad/s envelope below.
    controller_period = 1.0 / config[
        'controller_server']['ros__parameters']['controller_frequency']
    yaw_limit = controller['rotate_to_heading_angular_vel']
    internal_delta = controller['max_angular_accel'] * controller_period
    assert internal_delta >= 2.0 * yaw_limit
    assert min(yaw_limit, 0.0 + internal_delta) == yaw_limit
    assert min(yaw_limit, -yaw_limit + internal_delta) == yaw_limit
    assert behavior['max_rotational_vel'] == 0.50
    assert behavior['rotational_acc_lim'] == 1.0


def test_navfn_clearance_field_covers_the_effective_robot_footprint():
    config = _load_yaml(PACKAGE_ROOT / 'config' / 'nav2_params.yaml')

    for costmap_name in ('local_costmap', 'global_costmap'):
        parameters = config[costmap_name][costmap_name]['ros__parameters']
        inflation = parameters['inflation_layer']
        effective_radius = (
            parameters['robot_radius'] + parameters['footprint_padding'])

        soft_clearance = inflation['inflation_radius'] - effective_radius
        assert inflation['inflation_radius'] >= effective_radius
        assert 0.10 <= soft_clearance <= 0.20
        assert inflation['cost_scaling_factor'] == 6.0
        assert inflation['inflate_unknown'] is False
        assert inflation['inflate_around_unknown'] is False
        assert parameters['obstacle_layer'][
            'footprint_clearing_enabled'] is False

    planner = config['planner_server']['ros__parameters']['GridBased']
    assert planner['allow_unknown'] is True


def test_navigation_recovery_uses_raw_path_fallback_without_blind_motion():
    config = _load_yaml(PACKAGE_ROOT / 'config' / 'nav2_params.yaml')
    plugins = config['bt_navigator']['ros__parameters']['plugin_lib_names']
    assert 'nav2_clear_costmap_service_bt_node' in plugins

    for name in ('muto_nav_to_pose.xml', 'muto_nav_through_poses.xml'):
        text = (PACKAGE_ROOT / 'behavior_trees' / name).read_text(
            encoding='utf-8')
        assert 'name="PlanAndSmoothRecovery"' in text
        assert 'name="FollowPathRecovery"' in text
        assert 'global_costmap/clear_entirely_global_costmap' in text
        assert 'local_costmap/clear_entirely_local_costmap' in text
        assert '<Wait wait_duration="2"/>' in text
        assert '<Wait wait_duration="1"/>' in text
        assert 'wait_duration="0.' not in text
        assert '<RecoveryNode number_of_retries="1" name="NavigateRecovery">' in text
        assert '<ForceSuccess>' in text
        assert 'unsmoothed_path="{path}"' in text
        assert 'smoothed_path="{path}"' in text
        assert '<FollowPath path="{path}" controller_id="FollowPath"/>' in text
        assert '<Spin ' not in text
        assert '<BackUp ' not in text


def test_navfn_paths_use_humble_savitzky_golay_smoothing():
    config = _load_yaml(PACKAGE_ROOT / 'config' / 'nav2_params.yaml')
    parameters = config['smoother_server']['ros__parameters']

    assert parameters['smoother_plugins'] == [
        'savitzky_golay_smoother', 'simple_smoother'
    ]
    savitzky_golay = parameters['savitzky_golay_smoother']
    assert savitzky_golay['plugin'] == (
        'nav2_smoother::SavitzkyGolaySmoother'
    )
    assert savitzky_golay['do_refinement'] is True
    assert savitzky_golay['refinement_num'] == 2

    for name in ('muto_nav_to_pose.xml', 'muto_nav_through_poses.xml'):
        text = (PACKAGE_ROOT / 'behavior_trees' / name).read_text(
            encoding='utf-8')
        assert 'smoother_id="savitzky_golay_smoother"' in text
        assert 'smoother_id="simple_smoother"' not in text


def test_frontier_goal_stays_stable_while_nav2_is_driving():
    config = _load_yaml(
        PACKAGE_ROOT / 'config' / 'frontier_exploration_params.yaml'
    )
    parameters = config['frontier_explorer']['ros__parameters']

    # Frontier timing estimates must use the same request-side linear cap as
    # Nav2; these values do not command the base themselves.
    assert parameters['max_linear_speed_vmax'] == 0.25

    # Do not cancel and redispatch an accepted goal merely because a SLAM map
    # refresh changes its estimated visibility gain. Genuine blocked goals and
    # near-arrival completion remain independent safeguards.
    assert parameters['goal_preemption_enabled'] is False
    assert parameters['goal_skip_on_blocked_goal'] is True
    assert parameters['goal_preemption_complete_if_within_m'] == 0.25
    assert parameters['frontier_suppression_enabled'] is True
    assert parameters['frontier_suppression_attempt_threshold'] == 1
    assert parameters['frontier_suppression_no_progress_timeout_s'] < 10.0
    assert parameters['frontier_suppression_startup_grace_period_s'] == 0.0
    assert parameters['post_goal_settle_enabled'] is False
    assert parameters['frontier_suppression_base_size_m'] == 0.60
    assert parameters['frontier_suppression_timeout_s'] == 90.0


def test_frontier_navigation_uses_the_known_free_goal_adapter():
    config = _load_yaml(
        PACKAGE_ROOT / 'config' / 'frontier_exploration_params.yaml'
    )
    explorer = config['frontier_explorer']['ros__parameters']
    adapter = config['frontier_goal_adapter']['ros__parameters']

    assert explorer['navigate_to_pose_action_name'] == (
        '/frontier/navigate_to_pose')
    assert adapter['input_action'] == '/frontier/navigate_to_pose'
    assert adapter['nav2_action'] == '/navigate_to_pose'
    assert adapter['map_topic'] == '/map'
    assert adapter['effective_robot_radius'] == 0.27
    assert adapter['maximum_projection_distance'] == 0.0
    assert adapter['robot_base_frame'] == 'base_frame'
    assert adapter['robot_pose_timeout'] > 0.0
    assert adapter['robot_seed_search_distance'] >= 0.27
    assert adapter['minimum_staged_advance_distance'] == 0.20
    assert adapter['free_space_max_occupancy'] == 0
    assert explorer['frontier_suppression_persist_across_sessions'] is True
    assert explorer['frontier_candidate_partition_size_m'] > 0.0
    assert explorer['weight_gain_ws'] == 0.0

    standalone_launch = (
        PACKAGE_ROOT / 'launch' / 'frontier_exploration_launch.py'
    ).read_text(encoding='utf-8')
    assert "executable='frontier_goal_adapter'" in standalone_launch

    for profile_name in ('nav2_bag.yaml', 'nav2_bag_full.yaml'):
        profile = _load_yaml(NAV2_BAG_ROOT / 'config' / profile_name)
        topics = profile['nav2_bag_recorder']['ros__parameters']['topics']
        assert '/frontier_goal_adapter/original_goal' in topics
        assert '/frontier_goal_adapter/projected_goal' in topics
        assert '/frontier_goal_adapter/status' in topics
        assert '/frontier/navigate_to_pose/_action/status' in topics


def test_velocity_smoother_is_the_final_cmd_vel_limiter():
    launch_text = (
        PACKAGE_ROOT / 'launch' / 'nav2_planner_controller_launch.py'
    ).read_text(encoding='utf-8')
    assert "('cmd_vel', 'cmd_vel_nav')" in launch_text
    assert "('cmd_vel_smoothed', 'cmd_vel')" in launch_text


def test_camera_launch_is_camera_only_and_matches_declared_fov():
    launch_path = (
        SENSOR_ROOT / 'launch' / 'camera_depth_to_laserscan_launch.py'
    )
    defaults, nodes = _launch_defaults(launch_path)

    assert len(nodes) == 1
    assert defaults['output_topic'] == '/camera/filtered_laserscan'
    assert defaults['processing_frame'] == 'base_frame'
    assert float(defaults['horizontal_fov']) == 1.0192722831646884
    assert float(defaults['vertical_fov']) == 0.7941248096574199
    assert float(defaults['angle_min']) == -0.5096361415823442
    assert float(defaults['angle_max']) == 0.5096361415823442
    assert float(defaults['min_z']) == -0.07
    assert float(defaults['max_z']) == 0.18
    assert float(defaults['max_publish_rate']) == 7.0


def test_camera_preprocessing_is_owned_by_the_top_level_pipeline():
    mapping_launch = (
        PACKAGE_ROOT / 'launch' / 'online_async_mapping_launch.py'
    )
    pipeline_launch = (
        PACKAGE_ROOT / 'launch' / 'muto_nav2_pipeline_launch.py'
    )

    mapping_defaults, _ = _launch_defaults(mapping_launch)
    pipeline_defaults, _ = _launch_defaults(pipeline_launch)

    assert 'camera_depth_to_laserscan_launch.py' not in _python_strings(
        mapping_launch
    )
    assert 'camera_depth_to_laserscan_launch.py' in _python_strings(
        pipeline_launch
    )
    assert 'launch_camera_obstacle_scan' not in mapping_defaults
    assert pipeline_defaults['launch_camera_obstacle_scan'] == 'true'
    assert float(pipeline_defaults['camera_scan_max_publish_rate']) == 7.0


def test_compact_nav2_bag_is_a_default_pipeline_stage():
    pipeline_launch = (
        PACKAGE_ROOT / 'launch' / 'muto_nav2_pipeline_launch.py'
    )
    defaults, _ = _launch_defaults(pipeline_launch)
    source = pipeline_launch.read_text(encoding='utf-8')

    assert defaults['launch_nav2_bag'] == 'true'
    assert defaults['nav2_bag_output_directory'] == '/opt/muto_rs_ws/bags'
    assert int(defaults['max_bag_directories']) == 20
    assert defaults['nav2_bag_params_file'].endswith(
        'muto_nav2_bag/config/nav2_bag.yaml'
    )
    assert 'record_nav2_bag_launch.py' in source
    assert 'additional_success_actions=[nav2_bag_include]' in source
    assert (
        NAV2_BAG_ROOT / 'config' / 'nav2_bag_full.yaml'
    ).is_file()


def test_locomotion_loop_defaults_are_forwarded_by_the_pipeline():
    hardware_launch = HARDWARE_ROOT / 'launch' / 'muto_hardware_launch.py'
    pipeline_launch = (
        PACKAGE_ROOT / 'launch' / 'muto_nav2_pipeline_launch.py'
    )

    hardware_defaults, _ = _launch_defaults(hardware_launch)
    pipeline_defaults, _ = _launch_defaults(pipeline_launch)

    for defaults in (hardware_defaults, pipeline_defaults):
        assert float(defaults['locomotion_update_rate_hz']) == 50.0
        assert defaults['batch_gait_phase_writes'] == 'true'
        assert float(defaults['cmd_vel_timeout']) == 0.5
        assert defaults['locomotion_command_mapping'] == 'geometric'
        assert defaults['locomotion_calibration_file'].endswith(
            'muto_locomotion_provisional_20260806.yaml'
        )
        assert float(defaults['imu_publish_rate_hz']) == 10.0
        assert float(defaults['imu_attitude_publish_rate_hz']) == 10.0
        assert defaults['imu_suppress_identical_snapshots'] == 'true'
        assert float(defaults['imu_response_timeout_sec']) == 0.008
        assert float(defaults['imu_stale_warning_sec']) == 2.0
        assert float(defaults['imu_locomotion_guard_sec']) == 0.003
        assert int(defaults['imu_calibration_sample_count']) == 10
        assert int(defaults['imu_calibration_max_reads']) == 150
        assert float(defaults['imu_calibration_timeout_sec']) == 15.0
        assert float(defaults['imu_calibration_read_interval']) == 0.1
        assert 'motor_validation_settle_time' not in defaults
        assert 'gait_state_publish_rate_hz' not in defaults

    pipeline_configurations = _launch_configuration_names(pipeline_launch)
    assert 'locomotion_command_mapping' in pipeline_configurations
    assert 'locomotion_calibration_file' in pipeline_configurations
    assert 'imu_attitude_publish_rate_hz' in pipeline_configurations

    assert pipeline_defaults['launch_foot_odometry'] == 'false'
    assert float(pipeline_defaults['foot_motor_poll_rate']) == 2.0
    assert pipeline_defaults[
        'allow_experimental_high_rate_motor_polling'
    ] == 'false'
    assert pipeline_defaults['foot_odometry_source'] == 'measured_joints'
    assert int(pipeline_defaults['foot_max_motor_sequence_gap']) == 10


def test_ekf_sensor_ownership_keeps_foot_yaw_out_of_the_filter():
    base = _load_yaml(
        HARDWARE_ROOT / 'config' / 'ekf_lidar_imu.yaml'
    )['ekf_filter_node']['ros__parameters']
    foot = _load_yaml(
        HARDWARE_ROOT / 'config' / 'ekf_lidar_imu_with_foot.yaml'
    )['ekf_filter_node']['ros__parameters']

    assert base['odom0'] == '/scan_odom'
    assert base['odom0_config'] == [
        True, True, False,
        False, False, True,
        False, False, False,
        False, False, False,
        False, False, False,
    ]
    assert base['imu0'] == '/imu/data_processed'
    assert base['imu0_config'] == [
        False, False, False,
        False, False, False,
        False, False, False,
        False, False, True,
        False, False, False,
    ]
    assert foot['odom1'] == '/foot_odom'
    assert foot['odom1_config'] == [
        False, False, False,
        False, False, False,
        True, True, False,
        False, False, False,
        False, False, False,
    ]

    localization_defaults, _ = _launch_defaults(
        HARDWARE_ROOT / 'launch' / 'ekf_imu_lidar_launch.py'
    )
    assert localization_defaults['foot_odometry_source'] == (
        'measured_joints'
    )
    assert localization_defaults['launch_foot_odometry'] == 'false'
    assert float(localization_defaults['foot_motor_poll_rate']) == 2.0
    assert localization_defaults[
        'allow_experimental_high_rate_motor_polling'
    ] == 'false'
    assert int(
        localization_defaults['foot_max_motor_sequence_gap']
    ) == 10
    assert localization_defaults['fuse_controller_attitude_yaw'] == 'true'
    assert float(
        localization_defaults['controller_attitude_yaw_variance']
    ) == pytest.approx(0.004873878716587337)
    assert localization_defaults[
        'controller_attitude_stationary_gate'
    ] == 'true'
    assert float(localization_defaults[
        'controller_attitude_stationary_settle_sec'
    ]) == 2.0


def test_measured_rf2o_covariance_profile_is_the_replayable_default():
    launch_paths = (
        SENSOR_ROOT / 'launch' / 'filter_lidar_odometry_launch.py',
        HARDWARE_ROOT / 'launch' / 'ekf_imu_lidar_launch.py',
        PACKAGE_ROOT / 'launch' / 'muto_nav2_pipeline_launch.py',
        ODOMETRY_BAG_ROOT / 'launch' / 'replay_odometry_bag_launch.py',
        ODOMETRY_BAG_ROOT / 'launch' /
        'replay_odometry_comparison_launch.py',
    )

    for launch_path in launch_paths:
        defaults, _ = _launch_defaults(launch_path)
        assert defaults['rf2o_covariance_profile'] == 'measured'
        assert 'rf2o_covariance_profile' in _launch_configuration_names(
            launch_path
        )


def test_rf2o_deadbands_are_disabled_but_jump_guards_remain_default():
    launch_paths = (
        SENSOR_ROOT / 'launch' / 'filter_lidar_odometry_launch.py',
        HARDWARE_ROOT / 'launch' / 'ekf_imu_lidar_launch.py',
        PACKAGE_ROOT / 'launch' / 'muto_nav2_pipeline_launch.py',
        ODOMETRY_BAG_ROOT / 'launch' / 'replay_odometry_bag_launch.py',
        ODOMETRY_BAG_ROOT / 'launch' /
        'replay_odometry_comparison_launch.py',
    )

    for launch_path in launch_paths:
        defaults, _ = _launch_defaults(launch_path)
        assert float(defaults['rf2o_translation_deadband']) == 0.0
        assert float(defaults['rf2o_yaw_deadband']) == 0.0

    for launch_path in launch_paths[:2]:
        defaults, _ = _launch_defaults(launch_path)
        assert float(
            defaults['rf2o_translation_jump_rejection_threshold']
        ) == 0.03
        assert float(
            defaults['rf2o_yaw_jump_rejection_threshold']
        ) == 0.087266


def test_accelerated_replay_scales_the_stock_rf2o_poll_rate():
    replay_paths = (
        ODOMETRY_BAG_ROOT / 'launch' / 'replay_odometry_bag_launch.py',
        ODOMETRY_BAG_ROOT / 'launch' /
        'replay_odometry_comparison_launch.py',
    )

    for launch_path in replay_paths:
        defaults, _ = _launch_defaults(launch_path)
        assert float(defaults['foot_motor_poll_rate']) == 2.0
        assert defaults['foot_odometry_source'] == 'measured_joints'
        assert int(defaults['foot_max_motor_sequence_gap']) == 10
        source = launch_path.read_text(encoding='utf-8')
        assert "LaunchConfiguration('playback_rate'), ' * 64.0'" in source
        assert 'rf2o_process_on_scan' not in source
        assert 'rf2o_scan_queue_size' not in source
        assert (
            "'allow_experimental_high_rate_motor_polling': 'true'"
            in source
        )

    sensor_source = (
        SENSOR_ROOT / 'launch' / 'filter_lidar_odometry_launch.py'
    ).read_text(encoding='utf-8')
    assert "'process_on_scan'" not in sensor_source
    assert "'laser_scan_queue_size'" not in sensor_source
    assert "'covariance_profile': LaunchConfiguration(" in sensor_source


def test_hardware_motor_polling_above_two_hz_requires_opt_in():
    record_defaults, _ = _launch_defaults(
        ODOMETRY_BAG_ROOT / 'launch' / 'record_odometry_bag_launch.py'
    )

    assert record_defaults['record_motor_angles'] == 'false'
    assert float(record_defaults['motor_poll_rate']) == 2.0
    assert record_defaults[
        'allow_experimental_high_rate_motor_polling'
    ] == 'false'


def test_controller_attitude_has_a_stationary_relative_yaw_fusion_path():
    recorder = (
        ODOMETRY_BAG_ROOT / 'src' / 'odometry_bag_recorder.cpp'
    ).read_text(encoding='utf-8')
    replayer = (
        ODOMETRY_BAG_ROOT / 'src' / 'odometry_bag_replayer.cpp'
    ).read_text(encoding='utf-8')
    ekf = _load_yaml(
        HARDWARE_ROOT / 'config' / 'ekf_lidar_imu.yaml'
    )['ekf_filter_node']['ros__parameters']
    attitude_ekf = _load_yaml(
        HARDWARE_ROOT / 'config' / 'ekf_controller_attitude_yaw.yaml'
    )['ekf_filter_node']['ros__parameters']
    adapter = (
        SOURCE_ROOT / 'yahboomcar_imu' / 'yahboomcar_imu' /
        'controller_attitude_yaw_adapter.py'
    ).read_text(encoding='utf-8')

    for source in (recorder, replayer):
        assert '/imu/controller_attitude' in source
        assert '/muto/imu_telemetry_status' in source
        assert (
            'muto_hexapod_interfaces_custom/msg/ControllerAttitude'
            in source
        )
    assert r'\"schema_version\":4' in recorder
    assert 'imu_telemetry_status_capture_enabled' in recorder
    assert 'controller_yaw_status_capture_enabled' in recorder
    assert 'kImuTelemetryStatusType' in replayer
    assert 'kImuTelemetryStatusTopic,' in replayer
    assert 'validate_optional_topic(' in replayer
    assert ekf['imu0'] == '/imu/data_processed'
    assert '/imu/controller_attitude' not in str(ekf)
    assert attitude_ekf['imu0'] == '/imu/controller_attitude_imu'
    assert attitude_ekf['imu0_config'] == [
        False, False, False,
        False, False, True,
        False, False, False,
        False, False, False,
        False, False, False,
    ]
    assert attitude_ekf['imu0_relative'] is True
    assert attitude_ekf['imu0_differential'] is False
    assert float(attitude_ekf['imu0_pose_rejection_threshold']) == 1.0
    assert 'suppress_identical_snapshots' in adapter
    assert 'StationaryAttitudeGate' in adapter
    assert 'motion_state_is_stationary' in adapter
    assert '/muto/controller_attitude_yaw_status' in adapter
    assert 'angular_velocity_covariance[0] = -1.0' in adapter
    assert 'durability=DurabilityPolicy.VOLATILE' in adapter

    localization_defaults, _ = _launch_defaults(
        HARDWARE_ROOT / 'launch' / 'ekf_imu_lidar_launch.py'
    )
    pipeline_defaults, _ = _launch_defaults(
        PACKAGE_ROOT / 'launch' / 'muto_nav2_pipeline_launch.py'
    )
    replay_defaults, _ = _launch_defaults(
        ODOMETRY_BAG_ROOT / 'launch' / 'replay_odometry_bag_launch.py'
    )
    comparison_defaults, _ = _launch_defaults(
        ODOMETRY_BAG_ROOT / 'launch' /
        'replay_odometry_comparison_launch.py'
    )
    for defaults in (
        localization_defaults,
        pipeline_defaults,
        replay_defaults,
    ):
        assert defaults['fuse_controller_attitude_yaw'] == 'true'
        assert float(
            defaults['controller_attitude_yaw_variance']
        ) == pytest.approx(0.004873878716587337)
        assert defaults['controller_attitude_stationary_gate'] == 'true'
        assert float(
            defaults['controller_attitude_stationary_settle_sec']
        ) == 2.0
        assert float(
            defaults['controller_attitude_motion_state_timeout_sec']
        ) == 0.25
        assert float(
            defaults['controller_attitude_stability_window_sec']
        ) == 1.0
        assert int(
            defaults['controller_attitude_minimum_snapshots']
        ) == 3
        assert float(
            defaults['controller_attitude_max_yaw_span_rad']
        ) == pytest.approx(0.017453292519943295)
        assert float(
            defaults['controller_attitude_republish_interval_sec']
        ) == 0.0
    assert comparison_defaults['compare_controller_attitude'] == 'false'
    assert comparison_defaults[
        'lidar_controller_attitude_output_topic'
    ] == '/odometry/lidar_controller_attitude'
    pipeline = (
        PACKAGE_ROOT / 'launch' / 'muto_nav2_pipeline_launch.py'
    ).read_text(encoding='utf-8')
    readiness_gate = (
        PACKAGE_ROOT / 'scripts' / 'pipeline_readiness_gate'
    ).read_text(encoding='utf-8')
    assert '/imu/controller_attitude' in pipeline
    assert '/muto/motion_command_state' in pipeline
    assert 'localization_controller_attitude_enabled' in pipeline
    assert 'localization_raw_imu_enabled' in pipeline
    assert 'MotionCommandState' in readiness_gate
    assert (
        'muto_hexapod_interfaces_custom/msg/MotionCommandState'
        in readiness_gate
    )
    assert 'require_standard_imu_input' in replayer
    assert 'require_controller_attitude_input' in replayer
    assert 'require_motion_command_state_input' in replayer
    assert 'motion_command_publisher_->get_subscription_count() > 0' in (
        replayer
    )
    assert 'count_subscribers(kControllerAttitudeImuTopic) > 0' in replayer
    assert 'cache_recorded_static_transforms' in replayer
    assert 'publish_cached_static_transforms();' in replayer
    assert replayer.index('publish_cached_static_transforms();') < (
        replayer.index('while (!stop_requested_ && reader_->has_next())')
    )


def test_driver_uses_one_gait_slotted_telemetry_scheduler():
    driver = (
        HARDWARE_ROOT / 'yahboomcar_bringup' / 'muto_driver.py'
    ).read_text(encoding='utf-8')

    assert 'class TelemetryScheduler' in driver
    assert 'self.service_imu_telemetry()' in driver
    assert 'self.imu_timer = self.create_timer' not in driver
    assert 'self.imu_attitude_timer = self.create_timer' not in driver
    assert "'scheduler_policy': 'gait_then_one_telemetry'" in driver


def test_odometry_comparison_varies_inputs_not_covariance_profile():
    launch_path = (
        ODOMETRY_BAG_ROOT / 'launch' /
        'replay_odometry_comparison_launch.py'
    )
    defaults, _ = _launch_defaults(launch_path)

    assert defaults['rf2o_covariance_profile'] == 'measured'
    assert float(defaults['minimum_start_delay_sec']) == 2.0
    assert defaults['foot_odometry_source'] == 'measured_joints'
    assert defaults['launch_foot_odometry'] == 'false'
    assert float(defaults['foot_motor_poll_rate']) == 2.0
    assert int(defaults['foot_max_motor_sequence_gap']) == 10
    assert defaults['lidar_only_output_topic'] == '/odometry/lidar_only'
    assert defaults['lidar_imu_output_topic'] == '/odometry/lidar_imu'
    assert defaults['raw_lidar_imu_output_topic'] == (
        '/odometry/raw_lidar_imu'
    )
    assert defaults['lidar_controller_attitude_output_topic'] == (
        '/odometry/lidar_controller_attitude'
    )

    spec = importlib.util.spec_from_file_location(
        'replay_odometry_comparison_contract', launch_path
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    base = _load_yaml(
        HARDWARE_ROOT / 'config' / 'ekf_lidar_imu.yaml'
    )['ekf_filter_node']['ros__parameters']
    lidar_only = module.without_sensor(base, 'imu0')

    assert lidar_only['odom0'] == '/scan_odom'
    assert not any(
        name == 'imu0' or name.startswith('imu0_')
        for name in lidar_only
    )
    assert base['imu0'] == '/imu/data_processed'
    assert base['odom0'] == '/scan_odom'

    source = launch_path.read_text(encoding='utf-8')
    assert (
        "raw_lidar_imu_parameters['odom0'] = '/scan_odom_raw_profiled'"
        in source
    )
    assert (
        "'rf2o_profiled_raw_odom_topic': '/scan_odom_raw_profiled'"
        in source
    )


def test_retired_combined_scan_path_cannot_reenter_active_packages():
    active_python = list((PACKAGE_ROOT / 'launch').glob('*.py'))
    active_python.extend((SENSOR_ROOT / 'launch').glob('*.py'))

    for path in active_python:
        strings = _python_strings(path)
        for value in strings:
            assert not any(
                retired in value for retired in RETIRED_RUNTIME_IDENTIFIERS
            ), f'{path}: retired runtime identifier in {value!r}'

    active_text = [
        PACKAGE_ROOT / 'CMakeLists.txt',
        PACKAGE_ROOT / 'package.xml',
        SENSOR_ROOT / 'CMakeLists.txt',
        SENSOR_ROOT / 'package.xml',
    ]
    for path in active_text:
        content = path.read_text(encoding='utf-8')
        assert not any(
            retired in content for retired in RETIRED_RUNTIME_IDENTIFIERS
        ), f'{path}: retired runtime identifier'

    assert not (
        SENSOR_ROOT / 'src' / 'laserscan_fusion_node.cpp'
    ).exists()
