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
