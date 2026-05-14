# ROS 2 Wrapper for GPD

* [GPD library](https://github.com/atenpas/gpd)
* [Original ROS wrapper](https://github.com/atenpas/gpd_ros)
* [Kinova ROS 2 Repository](https://github.com/Kinovarobotics/ros2_kortex)

---

## Overview

A ROS 2 grasp detection and manipulation pipeline using:

- GPD for 6-DOF grasp pose detection
- Kinova Gen3 robot arm
- Robotiq 2F-85 gripper
- MoveIt 2 Task Constructor (MTC)

This repository contains three ROS 2 packages:

- `gpd_ros`  
  ROS 2 wrapper around GPD for grasp detection from point clouds.

- `kinova_gen3_6dof_robotiq_2f_85_moveit_config_1`  
  MoveIt 2 configuration package for Kinova Gen3 + Robotiq gripper.

- `pick_place`  
  Pick-and-place pipeline using MoveIt Task Constructor (MTC).

---

# 1) Installation

The following instructions were tested on:

- Ubuntu 24.04
- ROS 2 Jazzy

---

## Install ROS 2

Follow the official ROS 2 Jazzy installation guide:

https://docs.ros.org/en/jazzy/Installation.html

---

## Install GPD

Follow the installation instructions from the original GPD repository:

https://github.com/atenpas/gpd#install

Make sure to run:

```bash
make install
```

to install GPD as a shared library.

---

## Clone Repository

Clone this repository inside your ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/manjeetjha070/GPD_with_MTC.git
```

---


## Build Workspace

```bash
cd ~/ros2_ws

colcon build --symlink-install
```

---

## Source Workspace

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

# 2) Run the Robot

Launch the Kinova Gen3 robot and MoveIt 2 configuration:

```bash
ros2 launch kinova_gen3_6dof_robotiq_2f_85_moveit_config_1 robot.launch.py
```

More details about the robot launch configuration can be found in the official Kinova ROS 2 repository:

https://github.com/Kinovarobotics/ros2_kortex

---

# 3) Run GPD ROS 2 Node

Launch the GPD grasp detection node:

```bash
ros2 launch gpd_ros ur5.launch.py
```

The node subscribes to the point cloud topic and waits for grasp detection requests.

---
## Parameters

Brief explanations of parameters are given in [cfg/eigen_params.cfg](gpd_ros/config/ros_eigen_params.cfg).

The two parameters that you typically want to play with to **improve the
number of grasps found** are *workspace* and *num_samples*. The first defines the
volume of space in which to search for grasps as a cuboid of dimensions [minX,
maxX, minY, maxY, minZ, maxZ], centered at the origin of the point cloud frame.
The second is the number of samples that are drawn from the point cloud to
detect grasps. You should set the workspace as small as possible and the number
of samples as large as possible.

Most of the code is parallelized. To **improve runtime**, set *num_threads* to 
the number of (physical) CPU cores that your computer has available.

# 4) Run Pick and Place Pipeline

Launch the MoveIt Task Constructor (MTC) pick-and-place pipeline:

```bash
ros2 launch pick_place pick_place_demo.launch.py
```

This starts the `mtc_node` responsible for:

- receiving grasp poses
- planning motions
- executing pick-and-place tasks

---

# 5) Detect Grasps

Once the full system is running, call the grasp detection service:

```bash
ros2 service call /detect_grasps std_srvs/srv/Trigger
```

The workflow is:

1. GPD detects grasp candidates from the point cloud
2. Grasp poses are published
3. `pick_place` (`mtc_node`) receives the grasp
4. MoveIt 2 plans the motion
5. The Kinova Gen3 executes the pick-and-place task

---

# 6) Topics and Services

| Name | Type | Description |
|------|------|-------------|
| `/camera/depth/color/points` | Topic | Input point cloud |
| `/detect_grasps` | Service | Trigger grasp detection |
| `/plot_grasps` | Topic | Detected grasp poses |
| `/joint_states` | Topic | Robot joint states |
| `/tf` | Topic | Coordinate transforms |

---

# 7) Troubleshooting

## GPD Runtime Crash

If you encounter the following error:

```bash
double free or corruption (out)
```

This issue is related to aggressive compiler optimization flags used by GPD.

Related upstream issue:

- https://github.com/atenpas/gpd/issues/141

### Fix

In the `CMakeLists.txt` file, replace:

```cmake
set(CMAKE_CXX_FLAGS "-O3 -march=native -mtune=intel -msse4.2 -mavx2 -mfma -flto -fopenmp -fPIC -Wno-deprecated -Wenum-compare -Wno-ignored-attributes -std=c++17")
```

with:

```cmake
set(CMAKE_CXX_FLAGS "-O3 -fPIC -fopenmp -Wno-deprecated -Wenum-compare -Wno-ignored-attributes -std=c++17")
```

Removing CPU-specific optimization flags resolves the issue on Ubuntu 24.04 / ROS 2 Jazzy systems.

## GPD Library Not Found

If the shared library cannot be found:

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<path_to_gpd_library>
```

---

## ROS 2 Environment Not Sourced

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

## Rebuild Workspace

```bash
cd ~/ros2_ws

colcon build --symlink-install --cmake-clean-cache
```
