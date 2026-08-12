"""Declared parameter contract for the model commander."""

import math


PARAMETER_DEFAULTS = (
    ('action_name', '/look_for_object'),
    ('vlm_action', '/vlm/generate'),
    ('find_object_action', '/find_object'),
    ('go_to_object_action', '/go_to_object'),
    ('explore_frontier_action', '/command_primitives/explore_frontier'),
    ('navigate_to_pose_action', '/navigate_to_pose'),
    ('visibility_coverage_service', '/command_layer/visibility_coverage'),
    ('spin_action', '/spin'),
    ('rotate_cmd_vel_topic', '/cmd_vel'),
    ('registry_save_service', '/sam2/save_stored_objects'),
    ('detection_heartbeat_topic', '/sam2/detection_heartbeat'),
    ('registry_topic', '/sam2/stored_objects'),
    ('robot_pose_topic', '/odometry/filtered'),
    ('visual_observation_topic', '/camera/color/image_raw'),
    ('status_topic', '/model_commander/status'),
    ('command_bag_enabled', True),
    ('command_bag_required', False),
    ('command_bag_start_timeout', 2.0),
    ('command_bag_event_topic', '/model_commander/recording_event'),
    ('command_bag_status_topic', '/model_commander/bag_status'),
    ('decision_event_topic', '/model_commander/decision_event'),
    ('inspected_image_topic', '/model_commander/inspected_image'),
    ('vlm_model', 'gpt-5.6-luna'),
    ('endpoint_timeout', 5.0),
    ('vlm_result_timeout', 60.0),
    ('find_result_timeout', 400.0),
    ('child_stop_timeout', 10.0),
    ('default_max_duration', 1800.0),
    ('maximum_mission_duration', 7200.0),
    ('default_max_planning_steps', 64),
    ('maximum_planning_steps', 256),
    ('maximum_command_dispatches', 128),
    ('max_prompt_characters', 8192),
    ('max_reason_characters', 512),
    ('max_visual_observation_characters', 512),
    ('max_state_message_characters', 512),
    ('max_wait_seconds', 60.0),
    ('max_exploration_seconds', 60.0),
    ('max_rotation_radians', 6.283185307179586),
    ('max_observation_seconds', 30.0),
    ('spin_time_allowance', 15.0),
    ('rotate_executable_yaw_velocity', 0.30),
    ('rotate_timeout_reference_yaw_velocity', 0.15),
    ('rotate_goal_tolerance', 0.08),
    ('rotate_control_period', 0.05),
    ('rotate_stop_publish_count', 3),
    ('checkpoint_timeout', 10.0),
    ('observation_min_detection_frames', 3),
    ('minimum_explore_progress_distance_m', 0.1),
    ('minimum_no_match_travel_distance_m', 0.5),
    ('minimum_no_match_observations', 4),
    ('minimum_no_match_checkpoints', 1),
    ('minimum_no_match_rotation_radians', 6.283185307179586),
    ('planner_retry_initial_delay', 2.0),
    ('planner_retry_max_delay', 30.0),
    ('command_retry_initial_delay', 1.0),
    ('command_retry_max_delay', 10.0),
    ('max_consecutive_planner_failures', 3),
    ('max_consecutive_command_failures', 3),
    ('max_repeated_no_progress_decisions', 4),
    ('visual_observation_timeout', 5.0),
    ('visual_observation_max_age', 2.0),
    ('visual_observation_jpeg_quality', 80),
    ('visual_observation_max_width', 960),
    ('visual_observation_max_height', 720),
    ('visual_observation_max_jpeg_bytes', 1048576),
    ('visual_observation_max_source_width', 8192),
    ('visual_observation_max_source_height', 8192),
    ('visual_observation_max_source_bytes', 67108864),
    ('input_worker_poll_period', 0.02),
    ('input_worker_stop_timeout', 2.0),
    ('active_visual_monitoring', True),
    ('active_inspection_period', 20.0),
    ('active_inspection_timeout', 30.0),
    ('active_inspection_max_decision_age', 90.0),
    ('max_consecutive_active_inspection_failures', 3),
    ('max_visual_interrupts', 8),
    ('monitor_period', 0.05),
    ('status_publish_period', 1.0),
)


def declare_parameters(node):
    """Declare every supported commander parameter exactly once."""
    for name, default in PARAMETER_DEFAULTS:
        node.declare_parameter(name, default)


def read_parameters(node):
    """Expose resolved parameter values as node attributes."""
    for name, _ in PARAMETER_DEFAULTS:
        setattr(node, name, node.get_parameter(name).value)


def validate_parameters(config):
    """Validate cross-field and resource-bound invariants at startup."""
    for name in (
            'action_name', 'vlm_action', 'find_object_action',
            'go_to_object_action', 'explore_frontier_action',
            'navigate_to_pose_action', 'visibility_coverage_service',
            'spin_action',
            'rotate_cmd_vel_topic', 'registry_save_service',
            'detection_heartbeat_topic', 'registry_topic', 'robot_pose_topic',
            'visual_observation_topic', 'status_topic',
            'command_bag_event_topic', 'command_bag_status_topic',
            'decision_event_topic', 'inspected_image_topic'):
        if not getattr(config, name):
            raise ValueError(f'{name} must not be empty')
    for name in (
            'endpoint_timeout', 'vlm_result_timeout', 'find_result_timeout',
            'child_stop_timeout', 'default_max_duration',
            'maximum_mission_duration', 'max_wait_seconds',
            'max_exploration_seconds', 'max_rotation_radians',
            'max_observation_seconds', 'spin_time_allowance',
            'rotate_executable_yaw_velocity',
            'rotate_timeout_reference_yaw_velocity',
            'rotate_goal_tolerance', 'rotate_control_period',
            'checkpoint_timeout', 'minimum_explore_progress_distance_m',
            'minimum_no_match_travel_distance_m',
            'minimum_no_match_rotation_radians',
            'planner_retry_initial_delay', 'planner_retry_max_delay',
            'command_retry_initial_delay', 'command_retry_max_delay',
            'visual_observation_timeout', 'visual_observation_max_age',
            'active_inspection_period', 'active_inspection_timeout',
            'active_inspection_max_decision_age', 'monitor_period',
            'status_publish_period', 'command_bag_start_timeout',
            'input_worker_poll_period', 'input_worker_stop_timeout'):
        value = getattr(config, name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or \
                not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
    if config.default_max_duration > config.maximum_mission_duration:
        raise ValueError(
            'default_max_duration exceeds maximum_mission_duration')
    if config.planner_retry_initial_delay > config.planner_retry_max_delay:
        raise ValueError(
            'planner_retry_initial_delay exceeds its maximum')
    if config.command_retry_initial_delay > config.command_retry_max_delay:
        raise ValueError(
            'command_retry_initial_delay exceeds its maximum')
    for name in (
            'default_max_planning_steps', 'maximum_planning_steps',
            'maximum_command_dispatches', 'max_prompt_characters',
            'max_reason_characters', 'max_visual_observation_characters',
            'max_state_message_characters', 'minimum_no_match_observations',
            'minimum_no_match_checkpoints',
            'observation_min_detection_frames',
            'max_consecutive_planner_failures',
            'max_consecutive_command_failures',
            'max_repeated_no_progress_decisions',
            'visual_observation_jpeg_quality',
            'visual_observation_max_width',
            'visual_observation_max_height',
            'visual_observation_max_jpeg_bytes',
            'visual_observation_max_source_width',
            'visual_observation_max_source_height',
            'visual_observation_max_source_bytes', 'rotate_stop_publish_count',
            'max_consecutive_active_inspection_failures',
            'max_visual_interrupts'):
        value = getattr(config, name)
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError(f'{name} must be a positive integer')
    if config.default_max_planning_steps > config.maximum_planning_steps:
        raise ValueError(
            'default_max_planning_steps exceeds its maximum')
    if not 1 <= config.visual_observation_jpeg_quality <= 100:
        raise ValueError(
            'visual_observation_jpeg_quality must be in [1, 100]')
    for name in (
            'active_visual_monitoring', 'command_bag_enabled',
            'command_bag_required'):
        if not isinstance(getattr(config, name), bool):
            raise ValueError(f'{name} must be boolean')
    if config.command_bag_required and not config.command_bag_enabled:
        raise ValueError(
            'command_bag_required cannot be true when recording is disabled')
