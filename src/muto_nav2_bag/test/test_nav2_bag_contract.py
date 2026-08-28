from pathlib import Path
import re

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _parameters():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'nav2_bag.yaml').read_text())
    return config['nav2_bag_recorder']['ros__parameters']


def test_topic_profile_is_explicit_unique_and_navigation_scoped():
    topics = _parameters()['topics']

    assert topics
    assert len(topics) == len(set(topics))
    assert all(topic.startswith('/') for topic in topics)

    required = {
        '/tf',
        '/tf_static',
        '/map',
        '/odometry/filtered',
        '/lidar/filtered_laserscan',
        '/camera/filtered_laserscan',
        '/plan',
        '/plan_smoothed',
        '/received_global_plan',
        '/cmd_vel_nav',
        '/cmd_vel',
        '/muto/motion_command_state',
        '/navigate_to_pose/_action/status',
        '/muto/poi_grid/selected_pose',
        '/muto/poi_grid/result',
        '/muto/mission_board',
        '/muto/mission_event',
        '/muto/mission_rejection',
        '/behavior_tree_log',
        '/diagnostics',
        '/parameter_events',
        '/muto/nav2_bag/metadata',
        '/muto/nav2_bag/event',
        '/muto/nav2_bag/status',
        '/muto/nav2_bag/path',
    }
    assert required <= set(topics)
    assert len(topics) <= 56

    forbidden_fragments = (
        '/image_raw',
        '/compressed',
        '/points',
        'pointcloud',
        '/sam2/',
        'costmap/costmap',
        '_action/feedback',
    )
    assert '/bond' not in topics
    assert not any(
        fragment in topic.lower()
        for topic in topics
        for fragment in forbidden_fragments
    )


def test_yaml_and_compiled_fallback_topic_profiles_match_exactly():
    header_text = (
        PACKAGE_ROOT / 'include' / 'muto_nav2_bag' /
        'nav2_topic_profile.hpp').read_text()
    initializer = header_text.split('return {', 1)[1].split('};', 1)[0]
    compiled_topics = re.findall(r'"(/[^"]+)"', initializer)

    assert set(compiled_topics) == set(_parameters()['topics'])


def test_hidden_action_topics_retain_status_without_high_rate_feedback():
    topics = set(_parameters()['topics'])
    action_names = (
        'navigate_to_pose',
        'navigate_through_poses',
        'compute_path_to_pose',
        'compute_path_through_poses',
        'smooth_path',
        'follow_path',
        'spin',
        'backup',
        'wait',
    )

    assert _parameters()['include_hidden_topics'] is True
    for action_name in action_names:
        assert f'/{action_name}/_action/status' in topics
        assert f'/{action_name}/_action/feedback' not in topics


def test_full_context_profile_remains_available_as_an_explicit_opt_in():
    full = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'nav2_bag_full.yaml').read_text())
    full_topics = set(
        full['nav2_bag_recorder']['ros__parameters']['topics'])
    compact_topics = set(_parameters()['topics'])

    assert compact_topics <= full_topics
    assert '/local_costmap/costmap_raw' in full_topics
    assert '/global_costmap/costmap_raw' in full_topics
    assert '/navigate_to_pose/_action/feedback' in full_topics
    assert '/lidar/filtered_laserscan_no_downsample' in full_topics


def test_launch_exposes_output_and_reproducibility_inputs():
    launch_text = (
        PACKAGE_ROOT / 'launch' / 'record_nav2_bag_launch.py').read_text()

    for argument in (
        'output_directory',
        'bag_name',
        'storage_id',
        'max_bag_directories',
        'nav2_params_file',
        'slam_params_file',
        'nav_to_pose_bt_file',
        'nav_through_poses_bt_file',
        'locomotion_calibration_file',
    ):
        assert f"'{argument}'" in launch_text

    assert "package='muto_nav2_bag'" in launch_text
    assert "executable='nav2_bag_recorder'" in launch_text


def test_default_output_and_control_interfaces_are_stable():
    parameters = _parameters()

    assert parameters['output_directory'] == '/opt/muto_rs_ws/bags'
    assert parameters['storage_id'] == 'mcap'
    assert parameters['storage_preset'] == 'zstd_fast'
    assert parameters['max_bag_directories'] == 20
    assert parameters['record_all_services'] is False
    assert parameters['metadata_topic'] == '/muto/nav2_bag/metadata'
    assert parameters['event_topic'] == '/muto/nav2_bag/event'
    assert parameters['status_topic'] == '/muto/nav2_bag/status'
    assert parameters['path_topic'] == '/muto/nav2_bag/path'
    assert parameters['stop_service'] == '/muto/nav2_bag/stop'
