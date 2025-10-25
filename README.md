# mrpt_ros_bridge

[![CI Build colcon (ROS1 Noetic)](https://github.com/MRPT/mrpt_ros_bridge/actions/workflows/ros1-build.yml/badge.svg)](https://github.com/MRPT/mrpt_ros_bridge/actions/workflows/ros1-build.yml)

ROS 1 package providing a **C++ bridge between MRPT (Mobile Robot Programming Toolkit)** and the ROS ecosystem.
It enables seamless conversion between MRPT and ROS message types for maps, sensor data, poses, images, and more.

---

## Overview

The **`mrpt_libros_bridge`** library provides bidirectional converters between MRPT data structures and ROS message types.  
It simplifies integrating MRPT-based algorithms (e.g. SLAM, perception, mapping) with ROS nodes.

### Included packages

| Package | Description |
| --- | --- |
| **`mrpt_libros_bridge`** | Core C++ library bridging MRPT data types with ROS messages (poses, maps, sensor data, etc.). |
| **`rosbag2rawlog`** | Command-line tool to convert ROS `.bag` files into MRPT `.rawlog` datasets. |

---

## Features

- Convert between **MRPT and ROS** types:
  - `geometry_msgs::Pose` ⇄ `mrpt::poses::CPose3D`
  - `sensor_msgs::LaserScan` ⇄ `mrpt::obs::CObservation2DRangeScan`
  - `sensor_msgs::Image` ⇄ `mrpt::img::CImage`
  - `sensor_msgs::Imu`, `NavSatFix`, `PointCloud2`, etc.
- Written in **C++17**, using **ament_cmake**.
- Includes **unit tests** for conversions and message integrity.

---

## Installing

Install from ROS build servers with:

```bash
sudo apt install ros-${ROS_DISTRO}-rosbag2rawlog ros-${ROS_DISTRO}-mrpt-ros-bridge
```

## Building from sources

If needed to build from source code, do this:

```bash
# Source your ROS 1 environment
source /opt/ros/noetic/setup.bash

# Clone dependencies
mkdir -p ~/ros1_ws/src
cd ~/ros1_ws/src
git clone https://github.com/MRPT/mrpt_ros_bridge.git

# Install dependencies
rosdep install --from-paths . --ignore-src -r -y

# Build
cd ~/ros1_ws
colcon build --symlink-install
````

### 2. Converting a ROS 1 bag to MRPT rawlog

Refer to the [dataset conversions page](https://docs.mola-slam.org/latest/dataset-conversions.html) on the MOLA project website.

---

## Testing

Run unit tests with:

```bash
colcon test --packages-select mrpt_libros_bridge
colcon test-result --verbose
```

---

## Documentation

Auto-generated API docs (Doxygen):

```bash
cd mrpt_ros_bridge
doxygen doc/rosdoc.yaml
```

Further information about MRPT:
👉 [https://www.mrpt.org](https://www.mrpt.org)

---

## License

This package is distributed under the **BSD License**.

---

## Maintainer

**José-Luis Blanco-Claraco**
📧 [joseluisblancoc@gmail.com](mailto:joseluisblancoc@gmail.com)
🔗 [https://github.com/MRPT](https://github.com/MRPT)

