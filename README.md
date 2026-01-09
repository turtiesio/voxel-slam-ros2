# Voxel-SLAM: A Complete, Accurate, and Versatile LiDAR-Inertial SLAM System

## Modifications (ROS2 Fork)

This is a modified version of the original [Voxel-SLAM](https://github.com/hku-mars/Voxel-SLAM) for ROS2.

### Changes from Original

**1. ROS2 Port**
- Refactored from catkin to ament_cmake build system
- Updated all source files to use ROS2 APIs (rclcpp, sensor_msgs::msg, etc.)
- Replaced ROS1 XML launch files with ROS2 Python launch files
- Updated rviz configs for RViz2 compatibility

**2. IMU Gravity Scale Fix (`ekf_imu.hpp`, `voxelslam.cpp`)**

Original code had hardcoded topic name check that broke topic remapping:
```cpp
// Original (broken when topic is remapped)
if(mean_acc.norm() < 2 && imu_topic == "/livox/imu")
    scale_gravity = G_m_s2;
```

Fixed to use explicit parameter:
```cpp
// Fixed: configurable via General.imu_in_g_unit parameter
scale_gravity = imu_in_g_unit ? G_m_s2 : 1.0;
```

**3. New Parameter: `General.imu_in_g_unit`**
- `true`: IMU outputs acceleration in g unit (e.g., Livox) → multiply by 9.8
- `false`: IMU outputs acceleration in m/s² (e.g., Velodyne, Ouster, Hesai)
- Default: `false`

> **Note**: These modifications have only been tested with **Livox Mid-360**. Behavior on other LiDAR/IMU sensors (Avia, Velodyne, Ouster, Hesai, etc.) is untested.

---

## 1. Introduction

**Voxel-SLAM** is a complete, accurate, and versatile LiDAR-inertial SLAM system that fully utilizes short-term, mid-term, long-term, and multi-map data associations. It includes five modules: initialization, odometry, local mapping, loop closure, and global mapping. The initialization can provide accurate states and local map in a static or dynamic initial state. The odometry estimates current states and detect potential system divergence. The local mapping refine the states and local map within the sliding window by a LiDAR-inertial BA. The loop closure can detect in multiple sessions. The global mapping refine the global map with an efficient hierarchical global BA. The system overview is:

<div align="center">
    <a href="https://youtu.be/Cg9W01aIUzE" target="_blank">
    <img src="./figure/systemoverview.png" width = 60% >
</div>

### 1.1 Related Video

The video of **Voxel-SLAM** is available on [YouTube](https://youtu.be/Cg9W01aIUzE).

### 1.2 Related works

Related paper is available on [**arxiv**](https://arxiv.org/abs/2410.08935).

### 1.3 Competitions

Voxel-SLAM has been served as a subsystem to participate in [ICRA HILTI 2023 SLAM Challenge](https://hilti-challenge.com/leader-board-2023.html) (**2nd** place on the LiDAR single-session) and [ICCV 2023 SLAM Challenge](https://superodometry.com/iccv23_challenge_LiI) (**1st** place on the LiDAR inertial track).

## 2. Prerequisited

Ubuntu 22.04+. [ROS2 Humble/Jazzy](https://docs.ros.org/en/humble/Installation.html). [PCL>=1.10](https://pointclouds.org/). [Eigen>=3.3.7](https://eigen.tuxfamily.org/index.php?title=Main_Page)

[GTSAM>=4.0.3](https://github.com/borglab/gtsam/releases?page=2)

[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)

## 3. Build

```
cd ~/ros2_ws/src
git clone https://github.com/hku-mars/Voxel-SLAM
cd ~/ros2_ws && colcon build --packages-select voxel_slam
source ~/ros2_ws/install/setup.bash
```

## 4. Run Voxel-SLAM

### 4.1 Livox Avia

The online relocalization experiment rosbag. Download: [Onedrive](https://1drv.ms/f/c/8b1ef18ae4181c8d/ErEznhkJzTxJiLuJ8AQDGS0BvCy6KsuaWF2D6cnx061GEQ?e=dMRSlf) ([Google Drive](https://drive.google.com/file/d/1LG46i0vreQrMZRap5tJkjKK4IX0t0zfC/view?usp=drive_link))

```
ros2 launch voxel_slam vxlm_avia.launch.py
# Using "--start-paused" guarantees the bag beginning time are the same in different runs
# Press Space to start
ros2 bag play compus_elevator --start-paused
```

In the elevator, the system continues to restart until stepping out of the evevator. The blue point cloud is the map from initialization.

After the rosbag is done, your may find the map is inconsistent as shown in the video. Run

```
ros2 param set /voxel_slam finish true
```

to launch the final global mapping (global bundle adjustment) to refine the global map.

### 4.2 HILTI 2023 (Multi-Session)

The multi-session experiment rosbag.

For quick test to download: [Onedrive](https://1drv.ms/f/c/8b1ef18ae4181c8d/Epp5AQ2Oq1VNhC6MIuCAtN4BJC9jx9VvuVx7VT_cdlvD0A?e=8mXYdc). The whole rosbags of HILTI 2022 and 2023 are on the [website](https://hilti-challenge.com/index.html).

The rosbag had better be played from "site1_handheld_5" to "site_handheld_1", or the "site_handheld_2" and "site_handheld_3" cannot find the loop.

Before launching, please set configure the variables in "hesai.yaml". The '#' means annotation

```
# hesai.yaml
save_path: "${YOUR_FILE_PATH_TO_SAVE_THE_OFFLINE_MAP}"
previous_map: "# site1_handheld_5: 0.50,
               # site1_handheld_4: 0.45,
               # site1_handheld_3: 0.30,
               # site1_handheld_2: 0.50"
bagname: "site1_handheld_${1-5}" # The rosbag name you play
is_save_map: 1 # Enable to save the map
```

```
ros2 launch voxel_slam vxlm_hesai.launch.py
ros2 bag play site1_handheld_5 --start-paused
```

```
ros2 launch voxel_slam vxlm_hesai.launch.py  # Load the site_handheld_5
ros2 bag play site1_handheld_4 --start-paused
```

```
ros2 launch voxel_slam vxlm_hesai.launch.py  # Load the site_handheld_{5, 4}
ros2 bag play site1_handheld_3 --start-paused
```

For the "site1_handheld_2", do not forget load the offline maps. The "hesai.yaml" should be like this

```
# hesai.yaml
save_path: "${YOUR_FILE_PATH_TO_SAVE_THE_OFFLINE_MAP}"
previous_map: "site1_handheld_5: 0.50,
               site1_handheld_4: 0.45,
               site1_handheld_3: 0.30,
               # site1_handheld_2: 0.50"
bagname: "site1_handheld_2" # The rosbag name you play
is_save_map: 1 # Enable to save the map
```

```
ros2 launch voxel_slam vxlm_hesai.launch.py  # Load the site_handheld_{5, 4, 3}
ros2 bag play site1_handheld_2 --start-paused
```

```
ros2 launch voxel_slam vxlm_hesai.launch.py  # Load the site_handheld_{5, 4, 3, 2}
ros2 bag play site1_handheld_1 --start-paused
```

The map may not be consistent as shown in the video. Run

```
ros2 param set /voxel_slam finish true
```

for the final global BA.

### 4.3 MARS Dataset

For quick test to download: [Onedrive](https://1drv.ms/f/c/8b1ef18ae4181c8d/EpjsGW6coYlMvBWo8TlgJXoBttAuoocLi24V6kw-r_3A8w?e=vVB6RY). The whole rosbags of MARS dataset are on the [website](https://mars.hku.hk/dataset.html).

```
ros2 launch voxel_slam vxlm_avia_fly.launch.py
ros2 bag play HKisland03 --start-paused
```

The beginning of the point cloud is empty and failing to initialize until the drone at a certain height.

```
ros2 launch voxel_slam vxlm_avia_fly.launch.py
ros2 bag play AMvalley03 --start-paused
ros2 param set /voxel_slam finish true
```

This sequence is difficult to find loop. Please run the GBA to ensure the global map consistence.

### 4.4 Livox Mid360

The rosbag begin in a violent speed: [Onedrive](https://1drv.ms/f/c/8b1ef18ae4181c8d/ErtuXCFhFrBErZxzS5vLASkBJEfgDB9R2CSCgKe8BwhneQ?e=zbr7NL)

```
ros2 launch voxel_slam vxlm_mid360.launch.py
ros2 bag play jungle_challenge --start-paused
```

### 4.5 Others

Other types of LiDAR will be released later.

## 5. VoxelSLAMPointCloud2

**VoxelSLAMPointCloud2**: A customized plugin for RViz2. It has the same usage to original "PointCloud2" in RViz2, but it can **clear the point cloud map automatically** when receiving an empty point cloud, with any **Decay Time** of the plugin.

(1) Put the "VoxelSLAMPointCloud2" within the same workspace and build with colcon:
```
colcon build --packages-select voxelslam_pointcloud2
```

(2) Launch your program with RViz2.

(3) Click the "Add" button to add the "VoxelSLAMPointCloud2".

