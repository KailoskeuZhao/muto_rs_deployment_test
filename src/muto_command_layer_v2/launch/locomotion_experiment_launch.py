#!/usr/bin/env python3
"""Hardware-only locomotion experiment composition.

This is an opt-in diagnostic launch, not a production default.  It reuses the
supervised v2 hardware composition while disabling the object/VLM pipeline and
enabling the low-level odometry and Nav2 source bags.  Motion is issued by the
separate ``v2_nav2_smoke.py`` diagnostic client so each probe has a bounded
wall-clock deadline and an unambiguous Nav2 result.
"""

from __future__ import annotations

import os
from datetime import datetime, timezone

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _default_odometry_bag_path() -> str:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
    return f"/opt/muto_rs_ws/bags/muto_locomotion_experiment_{stamp}"


def generate_launch_description() -> LaunchDescription:
    v2_share = get_package_share_directory("muto_command_layer_v2")
    smoke_launch = os.path.join(v2_share, "launch", "v2_hardware_smoke_launch.py")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("launch_hardware", default_value="true"),
        DeclareLaunchArgument("launch_localization", default_value="true"),
        DeclareLaunchArgument("launch_mapping", default_value="true"),
        DeclareLaunchArgument("launch_nav2", default_value="true"),
        DeclareLaunchArgument("launch_nav2_bag", default_value="true"),
        DeclareLaunchArgument(
            "nav2_bag_output_directory", default_value="/opt/muto_rs_ws/bags"
        ),
        DeclareLaunchArgument("record_odometry_bag", default_value="true"),
        DeclareLaunchArgument("odometry_bag_path", default_value=_default_odometry_bag_path()),
        DeclareLaunchArgument("odometry_record_motor_angles", default_value="false"),
        # Keep the v2 node present for graph parity, but remove perception and
        # its high-level recorder from this motion-only experiment.
        DeclareLaunchArgument("launch_object_pipeline", default_value="false"),
        DeclareLaunchArgument("record_bag", default_value="false"),
        DeclareLaunchArgument("scenario_id", default_value="locomotion_experiment"),
        DeclareLaunchArgument("nav2_log_level", default_value="info"),
    ]

    smoke = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(smoke_launch),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "launch_hardware": LaunchConfiguration("launch_hardware"),
            "launch_localization": LaunchConfiguration("launch_localization"),
            "launch_mapping": LaunchConfiguration("launch_mapping"),
            "launch_nav2": LaunchConfiguration("launch_nav2"),
            "launch_nav2_bag": LaunchConfiguration("launch_nav2_bag"),
            "nav2_bag_output_directory": LaunchConfiguration(
                "nav2_bag_output_directory"
            ),
            "record_odometry_bag": LaunchConfiguration("record_odometry_bag"),
            "odometry_bag_path": LaunchConfiguration("odometry_bag_path"),
            "odometry_record_motor_angles": LaunchConfiguration(
                "odometry_record_motor_angles"
            ),
            "launch_object_pipeline": LaunchConfiguration("launch_object_pipeline"),
            "record_bag": LaunchConfiguration("record_bag"),
            "scenario_id": LaunchConfiguration("scenario_id"),
            "nav2_log_level": LaunchConfiguration("nav2_log_level"),
        }.items(),
    )
    return LaunchDescription([*args, smoke])

