"""Supervised v2 hardware smoke composition for the Humble Muto.

This launch starts the production sensor/localization/mapping/Nav2 pipeline,
the independent frontier explorer, the direct SAM2/registry/VLM authorities,
and the v2 executive.  It intentionally does not include the retired command
layer launch.  Hardware is enabled by default because this is a field smoke,
not a simulated-world fixture; keep the robot lifted or in a safe test area
until the readiness gates and first mission are confirmed.
"""

from __future__ import annotations

import os
from datetime import datetime, timezone

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_file(package: str, name: str) -> str:
    return os.path.join(get_package_share_directory(package), "launch", name)


def _scoped_include(package: str, name: str, arguments=None, condition=None):
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file(package, name)),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )
    return GroupAction(actions=[include], scoped=True)


def _default_odometry_bag_path() -> str:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
    return f"/opt/muto_rs_ws/bags/muto_odometry_{stamp}"


def generate_launch_description():
    slam_share = get_package_share_directory("muto_slam_mapping")
    v2_share = get_package_share_directory("muto_command_layer_v2")
    frontier_params = os.path.join(
        slam_share, "config", "frontier_exploration_params.yaml"
    )
    vlm_params = os.path.join(v2_share, "config", "v2_vlm_socket.yaml")

    args = [
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="Use ROS simulation time."
        ),
        DeclareLaunchArgument(
            "launch_hardware",
            default_value="true",
            description="Start the Muto base, LiDAR, and camera drivers.",
        ),
        DeclareLaunchArgument("launch_localization", default_value="true"),
        DeclareLaunchArgument("launch_mapping", default_value="true"),
        DeclareLaunchArgument("launch_nav2", default_value="true"),
        # Raw/low-level Nav2 capture is a separate opt-in diagnostic profile;
        # the default v2 recorder below is the sensor-free mission profile.
        DeclareLaunchArgument("launch_nav2_bag", default_value="false"),
        DeclareLaunchArgument(
            "nav2_bag_output_directory", default_value="/opt/muto_rs_ws/bags"
        ),
        DeclareLaunchArgument("launch_sensor_tf", default_value="true"),
        DeclareLaunchArgument(
            "sensor_tf_delay",
            default_value="1.0",
            description="Minimum delay before static sensor TF starts.",
        ),
        DeclareLaunchArgument("launch_camera_obstacle_scan", default_value="true"),
        DeclareLaunchArgument(
            "camera_scan_max_publish_rate",
            default_value="7.0",
            description="Maximum camera-depth-to-scan processing rate in Hz.",
        ),
        DeclareLaunchArgument("nav2_autostart", default_value="true"),
        DeclareLaunchArgument("nav2_use_respawn", default_value="false"),
        DeclareLaunchArgument("nav2_log_level", default_value="info"),
        DeclareLaunchArgument("localization_delay", default_value="3.0"),
        DeclareLaunchArgument("mapping_delay", default_value="8.0"),
        DeclareLaunchArgument("nav2_delay", default_value="12.0"),
        DeclareLaunchArgument("localization_readiness_timeout", default_value="120.0"),
        DeclareLaunchArgument("mapping_readiness_timeout", default_value="90.0"),
        DeclareLaunchArgument("nav2_readiness_timeout", default_value="120.0"),
        DeclareLaunchArgument("frontier_params_file", default_value=frontier_params),
        DeclareLaunchArgument("frontier_log_level", default_value="info"),
        DeclareLaunchArgument("launch_frontier", default_value="true"),
        DeclareLaunchArgument("launch_object_pipeline", default_value="true"),
        DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument(
            "depth_camera_info_topic", default_value="/camera/depth/camera_info"
        ),
        DeclareLaunchArgument(
            "color_camera_info_topic", default_value="/camera/color/camera_info"
        ),
        DeclareLaunchArgument("detections_topic", default_value="/sam2/detections"),
        DeclareLaunchArgument(
            "detection_heartbeat_topic", default_value="/sam2/detection_heartbeat"
        ),
        DeclareLaunchArgument(
            "instance_pointcloud_topic", default_value="/sam2/instance_pointcloud"
        ),
        DeclareLaunchArgument(
            "object_checkpoint", default_value="checkpoints/sam2.1_hiera_base_plus.pt"
        ),
        DeclareLaunchArgument(
            "object_openblas_preload",
            default_value="/usr/lib/aarch64-linux-gnu/libopenblas.so.0",
        ),
        DeclareLaunchArgument(
            "object_model_cfg", default_value="configs/sam2.1/sam2.1_hiera_b+.yaml"
        ),
        DeclareLaunchArgument("object_device", default_value="cuda"),
        DeclareLaunchArgument("object_yolo_model", default_value="yolo26m.pt"),
        DeclareLaunchArgument("object_yolo_device", default_value="0"),
        DeclareLaunchArgument("object_yolo_confidence", default_value="0.4"),
        DeclareLaunchArgument("object_max_publish_rate", default_value="7.0"),
        DeclareLaunchArgument(
            "object_registry_service", default_value="/sam2/get_stored_objects"
        ),
        DeclareLaunchArgument("object_registry_load_existing", default_value="false"),
        DeclareLaunchArgument("object_registry_store_images", default_value="true"),
        DeclareLaunchArgument("object_target_frame", default_value="map"),
        DeclareLaunchArgument("vlm_params_file", default_value=vlm_params),
        DeclareLaunchArgument("vlm_action", default_value="/vlm/generate"),
        DeclareLaunchArgument(
            "vlm_base_url", default_value="http://43.165.176.234:8080/v1"
        ),
        DeclareLaunchArgument("vlm_wire_api", default_value="chat_completions"),
        DeclareLaunchArgument("vlm_model", default_value="gpt-5.6-sol"),
        DeclareLaunchArgument("mission_action", default_value="muto/mission"),
        DeclareLaunchArgument("vlm_timeout_s", default_value="30.0"),
        DeclareLaunchArgument(
            "registry_query_service", default_value="/sam2/get_stored_objects"
        ),
        DeclareLaunchArgument(
            "frontier_control_service", default_value="/control_exploration"
        ),
        DeclareLaunchArgument(
            "frontier_completion_topic",
            default_value="/explore/exploration_complete",
        ),
        DeclareLaunchArgument("frontier_observe_duration_s", default_value="20.0"),
        DeclareLaunchArgument("navigate_action", default_value="/navigate_to_pose"),
        DeclareLaunchArgument("spin_action", default_value="/spin"),
        DeclareLaunchArgument(
            "scenario_completion_policy", default_value="report_confirmed"
        ),
        DeclareLaunchArgument("scenario_id", default_value="hardware_smoke"),
        DeclareLaunchArgument("camera_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("map_stale_after_s", default_value="2.0"),
        DeclareLaunchArgument("tf_stale_after_s", default_value="2.0"),
        DeclareLaunchArgument("footprint_radius_m", default_value="0.26"),
        DeclareLaunchArgument(
            "nav2_lifecycle_state_service", default_value="/bt_navigator/get_state"
        ),
        DeclareLaunchArgument("nav2_lifecycle_timeout_s", default_value="30.0"),
        DeclareLaunchArgument("record_bag", default_value="true"),
        DeclareLaunchArgument(
            "bag_output_uri",
            default_value="",
            description=(
                "Optional high-level MCAP URI/template. Empty uses a unique "
                "persistent URI under /opt/muto_rs_ws/bags."
            ),
        ),
        DeclareLaunchArgument("bag_run_id", default_value=""),
        DeclareLaunchArgument("bag_storage_id", default_value="mcap"),
        # The v2 recorder above is the default mission capture.  The odometry
        # bag is an opt-in diagnostic source bag and intentionally stays off
        # for a normal command-stack smoke.
        DeclareLaunchArgument("record_odometry_bag", default_value="false"),
        DeclareLaunchArgument(
            "odometry_bag_path", default_value=_default_odometry_bag_path()
        ),
        DeclareLaunchArgument("odometry_record_motor_angles", default_value="false"),
    ]

    nav2 = _scoped_include(
        "muto_slam_mapping",
        "muto_nav2_pipeline_launch.py",
        {
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "launch_hardware": LaunchConfiguration("launch_hardware"),
            "launch_localization": LaunchConfiguration("launch_localization"),
            "launch_mapping": LaunchConfiguration("launch_mapping"),
            "launch_nav2": LaunchConfiguration("launch_nav2"),
            "launch_nav2_bag": LaunchConfiguration("launch_nav2_bag"),
            "sensor_tf_delay": LaunchConfiguration("sensor_tf_delay"),
            "nav2_bag_output_directory": LaunchConfiguration(
                "nav2_bag_output_directory"
            ),
            "launch_sensor_tf": LaunchConfiguration("launch_sensor_tf"),
            "launch_camera_obstacle_scan": LaunchConfiguration(
                "launch_camera_obstacle_scan"
            ),
            "camera_scan_max_publish_rate": LaunchConfiguration(
                "camera_scan_max_publish_rate"
            ),
            "nav2_autostart": LaunchConfiguration("nav2_autostart"),
            "nav2_use_respawn": LaunchConfiguration("nav2_use_respawn"),
            "nav2_log_level": LaunchConfiguration("nav2_log_level"),
            "localization_delay": LaunchConfiguration("localization_delay"),
            "mapping_delay": LaunchConfiguration("mapping_delay"),
            "nav2_delay": LaunchConfiguration("nav2_delay"),
            "localization_readiness_timeout": LaunchConfiguration(
                "localization_readiness_timeout"
            ),
            "mapping_readiness_timeout": LaunchConfiguration(
                "mapping_readiness_timeout"
            ),
            "nav2_readiness_timeout": LaunchConfiguration("nav2_readiness_timeout"),
        },
    )
    frontier = _scoped_include(
        "muto_slam_mapping",
        "frontier_exploration_launch.py",
        {
            "params_file": LaunchConfiguration("frontier_params_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            # The v2 executive owns when exploration is started and stopped.
            "autostart": "false",
            "control_service_enabled": "true",
            "log_level": LaunchConfiguration("frontier_log_level"),
        },
        condition=IfCondition(LaunchConfiguration("launch_frontier")),
    )
    annotator = _scoped_include(
        "sam2_image_annotator",
        "sam2_image_annotator_launch.py",
        {
            "image_topic": LaunchConfiguration("image_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "depth_camera_info_topic": LaunchConfiguration("depth_camera_info_topic"),
            "color_camera_info_topic": LaunchConfiguration("color_camera_info_topic"),
            "detections_topic": LaunchConfiguration("detections_topic"),
            "detection_heartbeat_topic": LaunchConfiguration(
                "detection_heartbeat_topic"
            ),
            "instance_pointcloud_topic": LaunchConfiguration(
                "instance_pointcloud_topic"
            ),
            "checkpoint": LaunchConfiguration("object_checkpoint"),
            "openblas_preload": LaunchConfiguration("object_openblas_preload"),
            "model_cfg": LaunchConfiguration("object_model_cfg"),
            "device": LaunchConfiguration("object_device"),
            "yolo_model": LaunchConfiguration("object_yolo_model"),
            "yolo_device": LaunchConfiguration("object_yolo_device"),
            "yolo_confidence": LaunchConfiguration("object_yolo_confidence"),
            "max_publish_rate": LaunchConfiguration("object_max_publish_rate"),
        },
        condition=IfCondition(LaunchConfiguration("launch_object_pipeline")),
    )
    registry = _scoped_include(
        "sam2_object_registry",
        "object_registry_launch.py",
        {
            "pointcloud_topic": LaunchConfiguration("instance_pointcloud_topic"),
            "detections_topic": LaunchConfiguration("detections_topic"),
            "query_service": LaunchConfiguration("object_registry_service"),
            "target_frame": LaunchConfiguration("object_target_frame"),
            "load_existing": LaunchConfiguration("object_registry_load_existing"),
            "store_images": LaunchConfiguration("object_registry_store_images"),
        },
        condition=IfCondition(LaunchConfiguration("launch_object_pipeline")),
    )
    vlm = _scoped_include(
        "muto_vlm_socket",
        "vlm_socket_launch.py",
        {
            "params_file": LaunchConfiguration("vlm_params_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "action_name": LaunchConfiguration("vlm_action"),
            "base_url": LaunchConfiguration("vlm_base_url"),
            "wire_api": LaunchConfiguration("vlm_wire_api"),
            "default_model": LaunchConfiguration("vlm_model"),
        },
        condition=IfCondition(LaunchConfiguration("launch_object_pipeline")),
    )
    odometry_bag = _scoped_include(
        "muto_odometry_bag",
        "record_odometry_bag_launch.py",
        {
            "bag_path": LaunchConfiguration("odometry_bag_path"),
            "record_motor_angles": LaunchConfiguration("odometry_record_motor_angles"),
        },
        condition=IfCondition(LaunchConfiguration("record_odometry_bag")),
    )
    v2 = Node(
        package="muto_command_layer_v2",
        executable="v2_system_node.py",
        name="muto_command_layer_v2_system",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "mission_action": LaunchConfiguration("mission_action"),
                "vlm_action": LaunchConfiguration("vlm_action"),
                "vlm_model": LaunchConfiguration("vlm_model"),
                "vlm_timeout_s": LaunchConfiguration("vlm_timeout_s"),
                "registry_query_service": LaunchConfiguration("registry_query_service"),
                "frontier_control_service": LaunchConfiguration(
                    "frontier_control_service"
                ),
                "frontier_completion_topic": LaunchConfiguration(
                    "frontier_completion_topic"
                ),
                "frontier_observe_duration_s": LaunchConfiguration(
                    "frontier_observe_duration_s"
                ),
                "navigate_action": LaunchConfiguration("navigate_action"),
                "spin_action": LaunchConfiguration("spin_action"),
                "scenario_completion_policy": LaunchConfiguration(
                    "scenario_completion_policy"
                ),
                "scenario_id": LaunchConfiguration("scenario_id"),
                "camera_topic": LaunchConfiguration("camera_topic"),
                "map_stale_after_s": LaunchConfiguration("map_stale_after_s"),
                "tf_stale_after_s": LaunchConfiguration("tf_stale_after_s"),
                "footprint_radius_m": LaunchConfiguration("footprint_radius_m"),
                "nav2_lifecycle_state_service": LaunchConfiguration(
                    "nav2_lifecycle_state_service"
                ),
                "nav2_lifecycle_timeout_s": LaunchConfiguration(
                    "nav2_lifecycle_timeout_s"
                ),
            }
        ],
    )
    recorder = Node(
        package="muto_command_layer_v2",
        executable="high_level_recorder_node.py",
        name="muto_command_layer_v2_recorder",
        output="screen",
        condition=IfCondition(LaunchConfiguration("record_bag")),
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "output_uri": LaunchConfiguration("bag_output_uri"),
                "run_id": LaunchConfiguration("bag_run_id"),
                "storage_id": LaunchConfiguration("bag_storage_id"),
                "commander_model": LaunchConfiguration("vlm_model"),
            }
        ],
    )
    return LaunchDescription(
        args + [nav2, frontier, annotator, registry, vlm, odometry_bag, v2, recorder]
    )
