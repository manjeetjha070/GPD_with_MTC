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
  Modified MoveIt 2 configuration package for Kinova Gen3 + Robotiq gripper.

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
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

# 2) Run the Robot

Launch the Kinova Gen3 robot and MoveIt 2 configuration:

```bash
ros2 launch kinova_gen3_6dof_robotiq_2f_85_moveit_config_1 robot.launch.py
```

More details about the original robot configuration can be found in the official Kinova ROS 2 repository:

https://github.com/Kinovarobotics/ros2_kortex

---

## Modified MoveIt Configuration

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

### Added Predefined Robot States

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

These states are used by the MoveIt Task Constructor (MTC) pipeline for:

- pre-grasp motion
- top-down grasp approach
- place motion execution

---

### Modified Launch Configuration

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

# 3) Run GPD ROS 2 Node

Launch the GPD grasp detection node:

```bash
ros2 launch gpd_ros ur5.launch.py
```

The node subscribes to the point cloud topic and waits for grasp detection requests.

---

## Parameters

Brief explanations of parameters are given in:

```bash
gpd_ros/config/ros_eigen_params.cfg
```

The two parameters that you typically want to tune to improve grasp detection are:

- `workspace`
- `num_samples`

`workspace` defines the 3D search volume:

```text
[minX, maxX, minY, maxY, minZ, maxZ]
```

`num_samples` defines the number of sampled points used for grasp generation.

Recommendations:

- keep the workspace as small as possible
- increase `num_samples` for better grasp coverage

To improve runtime, set:

```text
num_threads
```

to the number of physical CPU cores available.

---

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

Workflow:

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

https://github.com/atenpas/gpd/issues/141

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

---

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
