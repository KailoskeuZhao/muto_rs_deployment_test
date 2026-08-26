"""Static contract checks for the v2-only hardware smoke composition."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "launch" / "v2_hardware_smoke_launch.py"
COMMAND_LAUNCH = ROOT / "launch" / "v2_command_layer_launch.py"
RECORDER = ROOT / "high_level_recorder_node.py"
COMPOSITION = ROOT / "composition.py"
SYSTEM_NODE = ROOT / "v2_system_node.py"
AUTHORITIES = ROOT / "ros_authorities.py"


def test_hardware_smoke_uses_direct_v2_authorities_and_not_retired_composition():
    text = LAUNCH.read_text(encoding="utf-8")

    assert 'package="muto_command_layer"' not in text
    assert 'package="muto_command_layer_v2"' in text
    for package, launch_name in (
        ("muto_slam_mapping", "muto_nav2_pipeline_launch.py"),
        ("muto_slam_mapping", "frontier_exploration_launch.py"),
        ("sam2_image_annotator", "sam2_image_annotator_launch.py"),
        ("sam2_object_registry", "object_registry_launch.py"),
        ("muto_vlm_socket", "vlm_socket_launch.py"),
        ("muto_odometry_bag", "record_odometry_bag_launch.py"),
    ):
        assert f'"{package}"' in text
        assert f'"{launch_name}"' in text

    assert '"launch_hardware",' in text
    assert 'default_value="true"' in text
    assert '"sensor_tf_delay",' in text
    assert '"sensor_tf_delay": LaunchConfiguration("sensor_tf_delay")' in text
    assert '"camera_scan_max_publish_rate",' in text
    assert '"camera_scan_max_publish_rate": LaunchConfiguration(' in text
    assert '"camera_scan_max_publish_rate"' in text
    assert 'scoped=False,' in text
    assert '"launch_nav2_bag", default_value="false"' in text
    assert '"record_odometry_bag", default_value="false"' in text
    assert '"autostart": "false"' in text
    assert '"frontier_goal_result_topic",' in text
    assert '"frontier_safety_watchdog_s",' in text
    assert "/explore/frontier_goal_result" in text
    assert '"bag_storage_id", default_value="mcap"' in text
    assert '"commander_model": LaunchConfiguration("vlm_model")' in text


def test_recorder_defaults_are_unique_persistent_and_overrideable():
    recorder_text = RECORDER.read_text(encoding="utf-8")
    command_text = COMMAND_LAUNCH.read_text(encoding="utf-8")

    assert "/opt/muto_rs_ws/bags/muto_command_v2_" in recorder_text
    assert '"run_id"' in recorder_text
    assert '"storage_id", "mcap"' in recorder_text
    assert (
        'DeclareLaunchArgument(\n            "bag_output_uri",\n'
        '            default_value=""' in command_text
    )
    assert (
        'DeclareLaunchArgument("bag_storage_id", default_value="mcap")'
        in command_text
    )


def test_frontier_result_default_matches_production_explorer_topic():
    for path in (COMPOSITION, SYSTEM_NODE, AUTHORITIES, COMMAND_LAUNCH):
        assert "/explore/frontier_goal_result" in path.read_text(encoding="utf-8")


def test_nodes_do_not_redeclare_ros_owned_use_sim_time_parameter():
    for path in (SYSTEM_NODE, RECORDER):
        text = path.read_text(encoding="utf-8")
        assert 'declare_parameter("use_sim_time"' not in text
        assert "declare_parameter('use_sim_time'" not in text


def test_system_node_treats_supervised_shutdown_as_a_clean_lifecycle_exit():
    text = SYSTEM_NODE.read_text(encoding="utf-8")

    assert "ExternalShutdownException" in text
    assert "except (KeyboardInterrupt, ExternalShutdownException)" in text
    assert "executor.shutdown(timeout_sec=2.0)" in text
    assert "except (Exception, KeyboardInterrupt)" in text
