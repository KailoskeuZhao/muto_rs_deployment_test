# Yahboomcar Description Reference

This package is retained as reference material for the original Muto robot
geometry. It is not part of the deployed navigation, TF, or hardware pipeline.

The package contains:

- the original `Muto.urdf`, with 24 links and 23 joints;
- all 22 STL meshes referenced by the URDF;
- the original display launch files and RViz configuration.

## Deployment boundary

The deployed robot uses `base_frame` and the explicit static-transform
publishers in `tf2_publisher`. This URDF instead has `base_link` as its root.
No deployed launch currently starts `robot_state_publisher` from this package
or publishes its articulated leg joint states.

Consequently, do not assume that adding or building this package changes the
live TF tree. Nav2 also does not derive its costmap footprint automatically
from URDF collision geometry; its footprint remains controlled by the
`robot_radius` or `footprint` values in `muto_slam_mapping/config/nav2_params.yaml`.

## Reference footprint measurements

The collision meshes were transformed into `base_link` coordinates at zero
joint angles. Their planar envelope is approximately:

| Geometry | Measurement |
| --- | ---: |
| Central `base_link` maximum radius | 0.144 m |
| Full six-leg maximum radius | 0.295 m |
| Full X bounds | -0.239 m to 0.239 m |
| Full Y bounds | -0.264 m to 0.264 m |

A Nav2 `robot_radius` of 0.16 m therefore represents the central body, not the
legs. A radius of approximately 0.30 m represents the zero-pose leg envelope;
the existing 0.01 m costmap footprint padding would make its effective radius
0.31 m. The final deployed value should still be checked against physical leg
motion, gait poses, mounts, and cabling.

