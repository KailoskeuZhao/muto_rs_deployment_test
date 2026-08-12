# Yahboomcar Description Reference

This package is retained as reference material for the original Muto robot
geometry. It is not part of the deployed navigation, TF, or hardware pipeline.
The package was copied from the Yahboom tutorial materials and is preserved
here as an upstream reference rather than project-authored robot-description
code.

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

Mesh zero-pose geometry reaches 0.295 m, but the field-measured walking
envelope used for deployment is approximately 0.26 m. Nav2 therefore uses a
0.26 m radius plus 0.01 m costmap padding, for an effective 0.27 m collision
radius. Recheck this measured value after gait, mount, or cabling changes.
