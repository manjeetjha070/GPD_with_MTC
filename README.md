# Reference repositories

* [GPD library](https://github.com/atenpas/gpd)
* [Original ROS wrapper](https://github.com/atenpas/gpd_ros)
* [Kinova ROS 2 Repository](https://github.com/Kinovarobotics/ros2_kortex)

---

## Overview

This repository implements a ROS 2 grasp detection and manipulation pipeline using:

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
# 1. Installation

## a) Install GPD Locally

GPD must be installed locally due to shared system constraints (optional:- Instead of local installation, you can also install globally following the official instruction https://github.com/atenpas/gpd#install )

### Clone and Build GPD

```bash
cd ~
git clone https://github.com/atenpas/gpd.git
cd gpd
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/gpd/local
make -j
make install 
```
> ### ⚠️ Troubleshooting: GPD Runtime Crash
>
> If GPD crashes with:
>
> ```bash
> double free or corruption (out)
> ```
>
> this is a known issue caused by CPU-specific compiler optimization flags.
>
> Related upstream issue:
>
> - https://github.com/atenpas/gpd/issues/141
>
> Edit `CMakeLists.txt` and replace:
>
> ```cmake
> set(CMAKE_CXX_FLAGS "-O3 -march=native -mtune=intel -msse4.2 -mavx2 -mfma -flto -fopenmp -fPIC -Wno-deprecated -Wenum-compare -Wno-ignored-attributes -std=c++17")
> ```
>
> with:
>
> ```cmake
> set(CMAKE_CXX_FLAGS "-O3 -fPIC -fopenmp -Wno-deprecated -Wenum-compare -Wno-ignored-attributes -std=c++17")
> ```
>
> Then rebuild GPD. Removing the CPU-specific optimization flags resolves the issue on Ubuntu 24.04 / ROS 2 Jazzy systems.

## b) Clone and built gpd_with_mtc Repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://git.uni-paderborn.de/rescue-robots/pg/ws-2025-26/gpd_with_mtc.git
cd ~/ros2_ws
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

# 2. Robot Setup

## a) Launch Kinova Gen3 + MoveIt

Launch the Kinova Gen3 arm and MoveIt 2 configuration:

```bash
ros2 launch kinova_gen3_6dof_robotiq_2f_85_moveit_config_1 robot.launch.py
```

More details about the robot launch configuration can be found in the official Kinova ROS 2 repository:

https://github.com/Kinovarobotics/ros2_kortex

---
## b) MoveIt Configuration Changes

The package `kinova_gen3_6dof_robotiq_2f_85_moveit_config_1` is based on the
official Kinova MoveIt 2 configuration package with additional modifications
required for grasp execution and octomap-based collision avoidance.

### Added 3D Sensor Configuration

A `sensors_3d.yaml` file was added to enable octomap updates from the depth camera.

```yaml
sensors:
  - camera_1_pointcloud

camera_1_pointcloud:
    sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater
    point_cloud_topic: /camera/depth/color/points
    max_range: 2.0
    point_subsample: 1
    padding_offset: 0.1
    padding_scale: 1.0
    max_update_rate: 5.0
    filtered_cloud_topic: /camera/depth/color/points_filtered

octomap_resolution: 0.05
```

This configuration enables:

- real-time octomap generation
- collision-aware motion planning
- obstacle updates from the RGB-D camera point cloud

---
## c) Modified Launch file

The MoveIt launch file was modified to:

- load the 3D sensor configuration
- enable octomap updates
- enable MTC task execution capability

```python
moveit_config = (
    MoveItConfigsBuilder(
        "gen3",
        package_name="kinova_gen3_6dof_robotiq_2f_85_moveit_config_1"
    )
    .robot_description(mappings=launch_arguments)
    .trajectory_execution(file_path="config/moveit_controllers.yaml")
    .planning_scene_monitor(
        publish_robot_description=True,
        publish_robot_description_semantic=True
    )
    .planning_pipelines(pipelines=["ompl"])
    .sensors_3d(
        file_path=os.path.join(
            get_package_share_directory(
                "kinova_gen3_6dof_robotiq_2f_85_moveit_config_1"
            ),
            "config/sensors_3d.yaml",
        )
    )
    .to_moveit_configs()
)

move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    output="screen",
    parameters=[
        moveit_config.to_dict(),
        {"default_planning_pipeline": "ompl"},
        {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
    ],
)
```

These modifications are required for:

- octomap-based collision checking
- point cloud integration
- MoveIt Task Constructor execution support
- autonomous pick-and-place planning

---

## d) Added Predefined Robot States

Additional robot poses were added in the SRDF file for the manipulation pipeline.

```xml
<group_state name="pick" group="manipulator">
    <joint name="joint_1" value="1.625808408789453e-06"/>
    <joint name="joint_2" value="-0.45021870732307434"/>
    <joint name="joint_3" value="-1.3422857522964478"/>
    <joint name="joint_4" value="-5.772884105681442e-05"/>
    <joint name="joint_5" value="-1.6796354055404663"/>
    <joint name="joint_6" value="1.570702314376831"/>
</group_state>

<group_state name="top_down" group="manipulator">
    <joint name="joint_1" value="-3.2851510241016513e-06"/>
    <joint name="joint_2" value="0.05423645302653313"/>
    <joint name="joint_3" value="-1.2926411628723145"/>
    <joint name="joint_4" value="-6.772646156605333e-05"/>
    <joint name="joint_5" value="-1.6244707107543945"/>
    <joint name="joint_6" value="1.5707274675369263"/>
</group_state>

<group_state name="place" group="manipulator">
    <joint name="joint_1" value="-1.531174348728916"/>
    <joint name="joint_2" value="1.409422346857654"/>
    <joint name="joint_3" value="-1.7763215228640181"/>
    <joint name="joint_4" value="0.0645870118849261"/>
    <joint name="joint_5" value="1.6382382182400632"/>
    <joint name="joint_6" value="1.5367747101852431"/>
</group_state>
```
These states are used by the MoveIt Task Constructor (MTC) pipeline:

- **`pick`** → Predefined pick position of the arm (default pick location used in the MTC pipeline)
- **`top_down`** → Arm position when looking from top-down at the object (not used, can be use instead of "pick")
- **`place`** → Predefined place position (currently not used in the MTC code; the place location is hard-coded, but users can modify or add their own preferred place position)

---

# 3. Run GPD ROS 2 Node


## Setup Requirements (Important)

Before running GPD, ensure the correct paths are set for your local installation.

### 1. Update GPD model path

In:

```
gpd_ros/config/ros_eigen_params.cfg
```

update the following parameter:

```bash
weights_file = /home/pg/rrs_ss25/manjeet/libs/gpd/models/lenet/15channels/params/
```

👉 This path must point to your local GPD model directory.

---

### 2. Update GPD library path in CMake

In the `CMakeLists.txt` of `gpd_ros`, update the GPD library path:

```cmake
find_library(GPD_LIB
  NAMES gpd
  PATHS /home/pg/rrs_ss25/manjeet/libs/gpd_install/
  PATH_SUFFIXES lib
  NO_DEFAULT_PATH
)

if(NOT GPD_LIB)
  message(FATAL_ERROR "Library GPD not found")
endif()

set(GPD_INCLUDE_DIRS /home/pg/rrs_ss25/manjeet/libs/gpd_install/include)
```

👉 Make sure both `include` and `lib` paths match your local GPD installation prefix.

---


Launch the GPD grasp detection node:

```bash
ros2 launch gpd_ros gpd.launch.py
```

The node subscribes to the point cloud topic and waits for grasp detection requests.

> **Note:** Before running the GPD ROS 2 node, check the voxelization setting in `gpd_ros/src/GraspDetectionNode.cpp`.
>
> ```cpp
> cloudCamera->voxelizeCloud(0.003);
> ```
>
> - **Simulation:** Comment out this line:
>
>   ```cpp
>   // cloudCamera->voxelizeCloud(0.003);
>   ```
>
>   Simulated point clouds generally do not require voxelization, and disabling it helps preserve point cloud density for grasp detection.
>
> - **Real Robot:** Keep this line enabled. Voxelization reduces the size of dense sensor point clouds, improving processing speed and robustness during grasp detection.

---
## Parameters

Brief explanations of parameters are given in [gpd_ros/config/ros_eigen_params.cfg](gpd_ros/config/ros_eigen_params.cfg).

The two parameters that can be typically played with to **improve the
number of grasps found** are *workspace* and *num_samples*. The first defines the
volume of space in which to search for grasps as a cuboid of dimensions [minX,
maxX, minY, maxY, minZ, maxZ], centered at the origin of the point cloud frame.
The second is the number of samples that are drawn from the point cloud to
detect grasps. You should set the workspace as small as possible and the number
of samples as large as possible.

To **improve runtime**, set *num_threads* to 
the number of (physical) CPU cores that your computer has available.

### Configuration of GPD parameters

GPD uses a configuration file to define the robot hand geometry, grasp descriptor parameters, and various algorithmic settings. For this project, the parameters were adapted to match the Kinova Gen3 robot arm with the Robotiq 2F-85 parallel gripper.

**Hand Geometry Parameters:**

| Parameter | Value | Description |
|-----------|-------|-------------|
| `finger_width` | 0.01 m | Width of each gripper finger |
| `hand_outer_diameter` | 0.12 m | Diameter of the hand (maximum aperture + 2 × finger width) |
| `hand_depth` | 0.06 m | Finger length from hand base to finger tip |
| `hand_height` | 0.08 m | Height of the hand |
| `init_bite` | 0.03 m | Minimum amount of object surface to be covered by the hand |

These values correspond to the physical dimensions of the Robotiq 2F-85 gripper and ensure that the grasp candidates generated by GPD are geometrically consistent with the actual end-effector.

**Grasp Descriptor Parameters:**

| Parameter | Value | Description |
|-----------|-------|-------------|
| `volume_width` | 0.085 m | Width of the cube inside the robot hand |
| `volume_depth` | 0.035 m | Depth of the cube inside the robot hand |
| `volume_height` | 0.02 m | Height of the cube inside the robot hand |
| `image_size` | 60 | Size of the input image (60 × 60 pixels) |
| `image_num_channels` | 15 | Number of input channels (3 projections × 5 channels) |

The volume dimensions define the region between the gripper fingers that is analyzed for grasp quality. The 15-channel configuration provides richer geometric information through three orthogonal projections (x, y, z) and five channels per projection (occupied surface, unobserved space, and surface normals). A 3-channel alternative using only surface normals from a single projection is also available for time-sensitive applications; the corresponding weight file can be loaded by changing the `weights_file` path in the configuration.

# 4. Run MTC Node

Launch the MoveIt Task Constructor (MTC) pick-and-place pipeline:

```bash
ros2 launch pick_place pick_place_demo.launch.py
```

This starts the `mtc_node` responsible for:

- receiving grasp poses
- planning motions
- executing pick-and-place tasks

## MTC Parameters

The `mtc_node` exposes several ROS 2 parameters that can be adjusted to tune robot motion and grasp selection behavior:

| Parameter | Default | Description |
|------------|---------|-------------|
| `velocity_scaling` | `0.1` | Scales trajectory execution velocity. |
| `acceleration_scaling` | `0.1` | Scales trajectory execution acceleration. |
| `w_gpd` | `1.0` | Weight assigned to the GPD CNN confidence score. |
| `w_height` | `0.5` | Weight assigned to the grasp height heuristic. |
| `w_top` | `0.8` | Weight assigned to the top-down approach heuristic. |
| `w_dist` | `0.3` | Weight assigned to the distance-to-robot heuristic. |

> **Note:** The default velocity and acceleration scaling factors are intentionally conservative to ensure safe and smooth robot motion during testing. These values can be increased to achieve faster execution once the system has been validated.

The grasp ranking function combines multiple heuristics:

- **`w_gpd = 1.0`** – Prioritizes grasps with high CNN confidence from GPD, as this is the strongest indicator of grasp quality.
- **`w_top = 0.8`** – Favors top-down grasps, which generally achieve higher success rates in tabletop manipulation and reduce the likelihood of collisions.
- **`w_height = 0.5`** – Prefers objects that are higher in a pile and therefore easier to access.
- **`w_dist = 0.3`** – Slightly favors grasps closer to the robot base, mainly serving as a tie-breaker when candidate grasps have similar scores.

These parameters can be modified through the ROS 2 parameter interface or launch configuration to adapt the system to different environments and task requirements.

---

# 5. Detect Grasps

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

# 6. Topics and Services

The following topics and services represent the **current default interface** used by this pipeline.

⚠️ Note: Topic names are configurable in the respective launch files and may differ depending on your setup.

| Name | Type | Description |
|------|------|-------------|
| `/camera/depth/color/points` | Topic | Input point cloud (configurable) |
| `/detect_grasps` | Service | Trigger grasp detection |
| `/plot_grasps` | Topic | Detected grasp poses (configurable) |





