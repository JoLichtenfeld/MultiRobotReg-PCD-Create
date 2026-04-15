# MultiRobotReg-PCD-Create

ROS 2 package set for distributed point cloud capture with a master UI and robot-side slave nodes.

## Dependencies

This package requires the GLFW development package for the ImGui/OpenGL UI:

```bash
sudo apt-get install libglfw3-dev
```

If you are using Ubuntu/ROS Jazzy, this installs the missing `glfw3` CMake config that `mrr_pcd_create` expects during configure/build.

## Build

Build from the `~/hector` workspace root:

```bash
cd ~/hector
colcon build --packages-up-to mrr_pcd_create
```

Inside the package source tree, you can also use the local wrapper if available:

```bash
hector build --this
```

## UI Settings Cache

The master UI persists its settings across restarts, including per-robot topic selections, IMU toggle, second lidar toggle, lidar-to-lidar 4x4 transform, sync host/user, timings, and sync root.

Cache file location:

- `~/.cache/mrr_pcd_create/ui_settings.cache`
- or `$XDG_CACHE_HOME/mrr_pcd_create/ui_settings.cache` if `XDG_CACHE_HOME` is set

## Capture Data Layout (PCDs, metadata, IMU)

Each slave writes one capture into:

`/tmp/pcd_captures/<robot_ns_key>/<timestamp>/`

- `<timestamp>` format: `YYYY-MM-DD_HH-MM-SS`
- `<robot_ns_key>` is robot namespace transformed as:
	- remove leading `/`
	- replace remaining `/` with `_`
	- if empty namespace (`/`), use `root`

Example:

- robot namespace `/robot_1` → `/tmp/pcd_captures/robot_1/2026-04-15_13-20-11/`
- robot namespace `/fleet/r2` → `/tmp/pcd_captures/fleet_r2/2026-04-15_13-20-11/`

### Files in one capture directory

Always present:

- `cloud_lidar_1.pcd` — accumulated raw lidar 1 cloud
- `cloud_merged.pcd` — final merged cloud in lidar 1 frame
- `cloud.pcd` — same content as `cloud_merged.pcd` (legacy compatibility)
- `cloud_lidar_1.topic` — source ROS topic for lidar 1 (single line)
- `cloud_merged.topics` — source ROS topic list used for merge (1 or 2 lines)
- `metadata.txt` — key/value metadata for downstream parsing

Present only if second lidar is enabled:

- `cloud_lidar_2.pcd` — accumulated raw lidar 2 cloud
- `cloud_lidar_2.topic` — source ROS topic for lidar 2
- `metadata.txt` includes `lidar_1_to_2_transform` block

Present only if IMU is enabled and samples were received:

- `cloud_merged.gravity` — averaged gravity vector `ax ay az` (single line)
- `cloud.gravity` — same content as `cloud_merged.gravity` (legacy compatibility)

### `metadata.txt` format

`metadata.txt` is plain text lines in `key: value` format. Typical keys:

- `robot_ns`
- `timestamp`
- `pc_topic_1`
- `pc_topic_2`
- `dual_lidar`
- `acc_time_s`
- `clouds_1`, `clouds_2`
- `points_1`, `points_2`
- `num_points`
- `merged_pcd`, `lidar_1_pcd`, `lidar_2_pcd`
- `imu_topic`, `imu_dur_s`, `imu_samples`, `gravity_mps2` (when IMU used)
- `lidar_1_to_2_transform` multiline 4x4 block (when dual lidar used)

### What to use in other packages

If you just need one cloud per capture, consume:

1. `cloud_merged.pcd`
2. `metadata.txt` (for provenance/settings)

If you also need topic provenance, read:

- `cloud_lidar_1.topic`
- `cloud_lidar_2.topic` (if present)
- `cloud_merged.topics`

If you need gravity alignment, read:

- `cloud_merged.gravity` (or `cloud.gravity` for legacy paths)

## Sync Layout (master "Sync dump/Sync All")

When syncing from robots, the master copies each remote capture directory into:

`<sync_local_root>/<robot_host_key>/<capture_leaf>/`

- `<sync_local_root>` is the UI field (default `/tmp/mrr_pcd_sync`)
- `<capture_leaf>` is the timestamp directory name from the robot
- `<robot_host_key>` is derived from robot namespace by:
	- remove leading `/`
	- replace remaining `/` with `-`
	- if empty, use `robot`

This means sync preserves each capture directory as-is, only adding a robot-level grouping folder.
