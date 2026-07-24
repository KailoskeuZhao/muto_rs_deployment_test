# TF2

```Plain Text
# Coordinate Frames and TF2 on MutoRS

## Why Frames Matter

Every ROS 2 sensor message with a `header` has a `frame_id`. That field says which coordinate system the data is expressed in.

This matters because the robot does not have one universal sensor coordinate system. Each sensor measures in its own local frame:

```text
/lidar/PointCloud        -> lidar_frame
/camera/depth/points     -> camera_depth_optical_frame
/imu/data_processed      -> imu_link
/fused/laserscan         -> camera_link or chosen processing frame
```

If data from two sensors is fused without transforming frames correctly, the result is geometrically wrong even if the topics are publishing normally\.

## Practical Example: Camera \+ LiDAR Fusion

The depth camera publishes:

```Plain Text
/camera/depth/points
```

in:

```Plain Text
camera_depth_optical_frame
```

But our filtering rules are defined in `camera_link`, for example:

```Plain Text
z range: [-0.4, 0.2]
max range: 3.0 m
```

So the point cloud must first be transformed:

```Plain Text
camera_depth_optical_frame -> camera_link
```

The LiDAR publishes raw data on:

```Plain Text
/lidar/PointCloud
```

in:

```Plain Text
lidar_frame
```

That cloud is filtered first:

```Plain Text
/lidar/PointCloud -> /lidar/PointCloudFiltered
```

The fused LaserScan node then uses:

```Plain Text
/camera/depth/points
/lidar/PointCloudFiltered
```

and publishes:

```Plain Text
/fused/laserscan
```

The final `/fused/laserscan` should not be restricted to the camera FOV\. The camera contributes only where it has valid depth points, while the LiDAR can contribute around the full scan\.

## TF2 Basics

TF2 stores coordinate relationships in a tree\.

Example:

```Plain Text
base_frame -> lidar_frame
base_frame -> camera_link
base_frame -> imu_link
```

If TF2 knows both:

```Plain Text
base_frame -> lidar_frame
base_frame -> camera_linkDeprecated
```

then it can compute:

```Plain Text
lidar_frame <-> camera_link
```

No separate direct transform is needed\.

Important rule:

```Plain Text
one child frame should have one parent frame
```

Do not publish two different transforms for the same child frame\. For example, do not let two nodes both publish `map -> odom`\.

## MutoRS Frame Structure

A practical frame tree for MutoRS is:

```Plain Text
map
└── odom
    └── base_frame
        ├── lidar_frame
        ├── camera_link
        │   └── camera_depth_optical_frame
        └── imu_link
```

Typical responsibilities:

```Plain Text
map -> odom
```

Provided by SLAM or localization\.

```Plain Text
odom -> base_frame
```

Provided by odometry or EKF\.

```Plain Text
base_frame -> sensor_frame
```

Static transforms from physical mounting positions\.

## Static Sensor Transforms

The static TF publishers are in:

```Plain Text
src/tf2_publisher
```

Examples:

```Plain Text
base_frame -> lidar_frame
base_frame -> camera_link
base_frame -> imu_link
```

These are static because the sensors are physically bolted to the robot\.

A static transform describes:

```Plain Text
translation: x, y, z
rotation: roll, pitch, yaw or quaternion
```

Example meaning:

```Plain Text
base_frame -> lidar_frame
```

says where the LiDAR is mounted relative to the robot base\.

## Dynamic Transforms

Dynamic transforms change over time\.

Example:

```Plain Text
odom -> base_frame
```

This transform represents where the robot currently is according to odometry\.

If LiDAR odometry or EKF updates the robot pose, it should update this transform continuously\.

## SLAM Mapping

The `muto_slam_mapping` package launches `slam_toolbox` online async mapping:

```Plain Text
ros2 launch muto_slam_mapping online_async_mapping_launch.py
```

It wraps:

```Plain Text
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=<muto_slam_mapping config yaml>
```

If you temporarily want `map` and `odom` to be identical, the launch file has an optional identity TF:

```Plain Text
ros2 launch muto_slam_mapping online_async_mapping_launch.py \
  publish_map_to_odom_tf:=true
```

That publishes:

```Plain Text
map -> odom
translation: 0 0 0
rotation: 0 0 0
```

Use this only when no other node is publishing `map -> odom`\. If `slam_toolbox` is publishing `map -> odom`, leave this disabled\.

## Debugging TF Problems

Check whether two frames are connected:

```Plain Text
ros2 run tf2_ros tf2_echo camera_link lidar_frame
```

If TF2 says:

```Plain Text
Invalid frame ID "lidar_frame" passed to lookupTransform
```

then one of these is true:

- the TF publisher is not running

- the frame name is different from expected

- the transform has not arrived yet

- the node started before TF was available

Check static transforms:

```Plain Text
ros2 topic echo /tf_static
```

Check dynamic transforms:

```Plain Text
ros2 topic echo /tf
```

Useful sanity check:

```Plain Text
ros2 topic echo /fused/laserscan --once
```

The scan header should contain the expected output frame, and the ranges should not be clipped to only the camera FOV\.

## Practical Rule

Before fusing any data, verify three things:

```Plain Text
1. The topic is publishing.
2. The message header.frame_id is correct.
3. TF2 can transform that frame into the processing frame.
```

For MutoRS sensor fusion, the usual processing frame should be consistent, such as:

```Plain Text
base_frame
```

or:

```Plain Text
camera_link
```

depending on the node\.

The important part is not which frame is chosen, but that all inputs are transformed into the same frame before filtering, projection, fusion, odometry, or mapping\.

## Example Visualization

![Image](https://internal-api-drive-stream-jp.larksuite.com/space/api/box/stream/download/authcode/?code=YzQ1MDEwODA2Y2U1NjU3NjZmZjBlMDMzODcyODJmYTJfNThhOTJmMDMyMDY0NjBlNzg2NmU2MTk3N2JjNzkwMzVfSUQ6NzY2NDkwNjkxMTYzMjgyMTc4Nl8xNzg0ODc4NzU5OjE3ODQ5NjUxNTlfVjM)



