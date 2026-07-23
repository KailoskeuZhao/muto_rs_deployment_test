# SAM2 Object Registry

This ROS 2 package provides typed object-observation interfaces and a C++ node
that turns instance-marked point clouds into a persistent, queryable object
registry.

The node synchronizes `/sam2/detections` with
`/sam2/instance_pointcloud`, computes one centroid per instance, and asks TF2
for `target_frame <- cloud_frame` at the cloud timestamp. It does not substitute
the latest transform when historical TF is missing.

Objects are held in hash maps while the node runs. Unique-name queries are
constant-time, label queries use a label index, and same-label spatial matching
checks only neighboring cells in a 3D spatial hash. Observations inside
`duplicate_distance_threshold` update a weighted centroid; distinct objects use
names such as `chair`, `chair_2`, and `chair_3`.

Run the annotator and registry in separate terminals:

```bash
ros2 launch sam2_image_annotator sam2_image_annotator_launch.py
ros2 launch sam2_object_registry object_registry_launch.py
```

The transient-local `/sam2/stored_objects` topic exposes the most recent full
snapshot. New objects publish immediately; repeated centroid updates are
coalesced and published at `snapshot_publish_rate` (2 Hz by default), avoiding
full-registry serialization at the detector rate. The query service always
reads the latest in-memory state and accepts an optional exact `name` and/or
`label`; empty fields return every object:

For RViz, add a `MarkerArray` display using
`/sam2/stored_object_markers` and set the fixed frame to `map` (or the configured
`target_frame`). The transient-local marker snapshot contains one centroid
sphere and one name label per in-memory object, including objects loaded from
YAML at startup. The live masked object surfaces remain available separately as
the `PointCloud2` topic `/sam2/instance_pointcloud`.

```bash
ros2 service call /sam2/get_stored_objects \
  sam2_object_registry/srv/GetStoredObjects \
  "{name: chair, label: ''}"
```

By default, an empty `output_yaml` resolves to `sam2_objects.yaml` in the active
colcon workspace root. For example, a package installed below
`/opt/muto_rs_ws/install` writes `/opt/muto_rs_ws/sam2_objects.yaml`. Set the
`ROS_WORKSPACE` environment variable to select the workspace explicitly, or
pass `output_yaml:=/some/path/objects.yaml`. Existing objects are loaded at
startup and merged with new observations. On clean SIGINT/SIGTERM shutdown, the
complete merged database is written through a temporary file, `fsync`, and
atomic rename. This is a valid YAML rewrite rather than literal text append,
which would corrupt a single YAML document. A manual checkpoint is also
available:

When the workspace file does not exist, the default-path mode loads the legacy
`~/.ros/sam2_objects.yaml` once and writes the merged data to the workspace file
on the next save or clean shutdown.

```bash
ros2 service call /sam2/save_stored_objects std_srvs/srv/Trigger "{}"
```

SIGKILL and sudden power loss cannot run a shutdown hook, so use the save
service when an immediate checkpoint is important. If an existing YAML file is
malformed or uses another `frame_id`, persistence is disabled to protect it;
the in-memory registry remains available.

The C++ layer repeats the `yolo_confidence` check defensively. With the default
`0.4`, lower-confidence instances are not recorded even if an upstream
publisher bypasses the annotator guard.
