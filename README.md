# AirBot Modules (on going)

This repository contains ROS 2-based modules that I developed as part of the **AirBot** mobile robot project. These components were designed to handle core system functionalities such as sensor data abstraction, error state management, and LiDAR scan processing.

---

## 📦 Included Modules

### 1. `sensor_interface`
Manages multiple onboard sensors and provides a unified interface for use in the robot's control system.

**Handled sensors:**
- Camera: OG02B10
- Time-of-Flight (ToF)
  - Multi-Zone ToF: VL53L8CX
  - 1D ToF:
- Cliff sensors (IR): WEJ324JC-T2A

**Key features:**
- Publishes various raw sensor data in `sensor_msgs::PointCloud2` format
- Supports runtime coordinate frame transformation (e.g., `base_link` ⇄ `map`)
- Dynamic sensor activation and deactivation (on/off) via parameter server
- Flexible update and publishing rates for each individual sensor
- Sensor data filtering & normalization

---

### 2. `error_manager`
Monitors and manages system-wide errors based on incoming sensor and state data.

**Key features:**
- Manages the Occurrence and Release of all system-level errors and delivers them to the user interface
- Includes `error_monitor` modules that evaluate complex error conditions by combining multiple sensor and state inputs
- Supports flexible error specification using a YAML-based `error_list_`, allowing easy addition and removal
- Provides a standard error monitoring interface for adding custom `error_monitor` modules when new error conditions are needed
- Handles errors with ambiguous clear conditions by issuing a notify-only event that requires manual or external clearing logic

---

### 3. `lidar_processing`
Processes raw LiDAR scan data and extracts useful information for navigation and obstacle avoidance.

**Key features:**
- Integrates vendor-provided SDK to decode raw LiDAR data packets within a specified angular or distance range
- Merges front and back LiDAR data into a unified `PointCloud2` using open-source pointcloud-to-laserscan conversion tools
- Transforms the combined pointcloud data into a `LaserScan` message in the `base_link` coordinate frame, published to the `/scan` topic


---

## 🔧 Tech Stack

- **ROS 2 (Humble)** — modular node-based architecture, rclcpp, parameter server, TF2
- **C++** — real-time system programming, memory-safe modular design
- **Sensor Integration** — multi-sensor fusion (IMU, ToF, LiDAR, IR, bumper)
- **Coordinate Transformations** — convert sensor data to sensor_frame & sensor_frame to base_link & base_link to map_frame, runtime frame conversions (`base_link`, `map`, etc.)
- **Error Handling System** — extensible monitoring interface, YAML-based config
- **LiDAR Data Processing** — SDK integration, pointcloud merging, scan conversion


---


## 🧑‍💻 My Role

In the development of an autonomous mobile air purifier robot, I was solely responsible for designing and implementing the following three core components:

- **Sensor Interface Module**: abstraction and management of all onboard sensors
- **Error Manager Module**: centralized system error tracking and handling
- **LiDAR Processing Module**: decoding, merging, and publishing LiDAR data

These modules were fully developed and maintained by me from scratch.
I also ensured real-time performance on an embedded ARM64 system and integrated these components with other parts of the robot software, such as motion control and navigation.
