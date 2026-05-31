# Navigation Development Roadmap

## Purpose

This document defines the phased implementation plan for the SAADS navigation stack.

The goal is to move from architecture and interface definitions to a fully operational autonomous navigation system while minimizing integration risk.

---

# Current Status

## Completed

* ROS2 workspace architecture created
* Navigation package scaffolded
* Bringup package scaffolded
* Navigation interface specification created
* Nav2 architecture document created
* TF tree specification created
* GitHub repository established
* Development workflow established

## In Progress

* Navigation subsystem design
* Nav2 integration planning

---

# Phase 1 - Interface Definition

## Goal

Define all interfaces required for navigation before implementation begins.

### Deliverables

* navigation_interface_spec.md
* nav2_architecture.md
* tf_tree_spec.md

### Success Criteria

* Required topics identified
* Required TF frames identified
* Dependencies identified
* Team agreement on interfaces

Status:

COMPLETE

---

# Phase 1.5 - Component Integration Lab (CIL) Strategy

## Goal

Validate each sensor and subsystem independently before full integration.

### Philosophy

SAADS will follow a Component Integration Lab (CIL) approach.

Each sensor must be validated individually before being connected to localization, navigation, or mission execution.

Development Flow:

```text
Sensor Hardware
        ↓
Driver Validation
        ↓
Processing Validation
        ↓
CIL Integration Testing
        ↓
Subsystem Integration
        ↓
Full System Integration
```

---

## Sensor Validation Stages

### Stage 1 - Hardware Validation

Verify:

* Power
* Communications
* Physical mounting
* Driver startup

Example:

```text
LiDAR powers on
LiDAR detected by Linux
Driver launches successfully
```

---

### Stage 2 - Driver Validation

Verify:

* ROS2 node launches
* Topics publish correctly
* Message types are correct
* Frame IDs are correct

Example:

```text
/lidar/points_raw
sensor_msgs/msg/PointCloud2
```

---

### Stage 3 - Processing Validation

Verify:

```text
Raw Data
    ↓
Processing Node
    ↓
Processed Output
```

Example:

```text
points_raw
     ↓
pointcloud_filter
     ↓
points_filtered
```

---

### Stage 4 - Navigation Validation

Verify:

* TF transforms valid
* Costmaps update correctly
* Nav2 consumes data correctly

Example:

```text
LiDAR
     ↓
pointcloud_filter
     ↓
Nav2 Costmap
```

---

### Stage 5 - Full System Validation

Verify:

* Sensor interoperability
* Localization stability
* Navigation stability
* Mission execution stability

---

## Planned Sensor Validation Order

1. LiDAR
2. IMU
3. GPS
4. Wheel Encoders
5. Camera
6. Radar (if used)
7. Sonar (if used)

### Success Criteria

* Sensor passes standalone validation
* Sensor passes CIL testing
* Sensor approved for system integration

Status:

ACTIVE DEVELOPMENT APPROACH

---

# Phase 2 - Navigation Framework Setup

## Goal

Establish the minimum Nav2 structure.

### Tasks

* Review nav2_params.yaml
* Review nav_launch.py
* Identify required Nav2 packages
* Determine initial planner plugin
* Determine initial controller plugin
* Define costmap strategy

### Expected Decisions

Planner:

```text
Smac Planner
```

Controller:

```text
Regulated Pure Pursuit
```

Costmaps:

```text
Global Costmap
Local Costmap
```

### Success Criteria

* Nav2 configuration strategy documented
* Launch structure defined

Status:

NEXT

---

# Phase 3 - TF and Localization Foundation

## Goal

Provide the minimum TF structure required by Nav2.

### Tasks

Create:

```text
map
odom
base_link
```

Create sensor frames:

```text
lidar_frame
imu_frame
gps_frame
camera_frame
```

Determine ownership of:

```text
map -> odom
odom -> base_link
```

### Success Criteria

* TF tree visible in RViz
* Required transforms available

Status:

PLANNED

---

# Phase 4 - Simulated Navigation Testing

## Goal

Validate Nav2 before integrating real sensors.

### Test Nodes

Create:

```text
fake_odom_node
fake_lidar_node
tf_test_node
```

### Test Pipeline

```text
Fake Odom
     ↓
Nav2

Fake LiDAR
     ↓
Costmaps

TF
     ↓
Navigation
```

### Success Criteria

* Nav2 launches
* Costmaps initialize
* Planner generates paths
* Controller outputs /cmd_vel

Status:

PLANNED

---

# Phase 5 - LiDAR Integration

## Goal

Replace simulated LiDAR with the real Unitree LiDAR.

### Pipeline

```text
lidar_driver
      ↓
pointcloud_filter
      ↓
Nav2 Costmap
```

### Tasks

* Verify PointCloud2 publishing
* Verify frame IDs
* Validate transforms
* Validate obstacle layer updates

### Success Criteria

* Real obstacles appear in costmaps
* Planner reacts to obstacles

Status:

PLANNED

---

# Phase 6 - Localization Integration

## Goal

Integrate GPS, IMU, and wheel encoders into state estimation.

### Inputs

```text
GPS
IMU
Wheel Encoders
```

### Output

```text
/odom
```

Potential tools:

```text
robot_localization
EKF
```

### Success Criteria

* Stable odometry
* Stable localization
* Nav2 receives valid pose estimates

Status:

PLANNED

---

# Phase 7 - GPS Navigation

## Goal

Enable waypoint navigation using GPS coordinates.

### Tasks

* Define GPS datum
* Convert GPS coordinates to local frame
* Define waypoint interface
* Connect GPS goals to Nav2

### Success Criteria

* GPS waypoints converted into navigation goals
* Robot navigates between waypoints

Status:

PLANNED

---

# Phase 8 - Mission Execution

## Goal

Integrate mission management with Nav2.

### Mission Commands

```text
Start Mission
Pause Mission
Resume Mission
Stop Mission
Return Home
```

### Interfaces

```text
/mission/start
/mission/stop
/mission/pause
/mission/resume
/mission/return_home
```

### Success Criteria

* Mission manager can command Nav2
* Navigation state is reported back

Status:

PLANNED

---

# Phase 9 - Full System Integration

## Goal

Integrate all navigation components into the complete robot.

### Integrated Components

```text
Nav2
Localization
LiDAR
GPS
Camera
Vehicle Controller
Mission Manager
Diagnostics
Telemetry
```

### Success Criteria

* End-to-end autonomous navigation
* Obstacle avoidance
* Waypoint navigation
* Mission execution

Status:

FUTURE

---

# Long-Term Enhancements

Potential future additions:

* Radar integration
* Sonar integration
* RTK GPS
* Visual SLAM
* Multi-sensor obstacle fusion
* Dynamic obstacle classification
* Multi-robot coordination
* Advanced mission planning

---

# Current Priority

The next task is:

```text
Phase 2 - Navigation Framework Setup
```

Specifically:

1. Review nav2_params.yaml
2. Review nav_launch.py
3. Review navigation_launch.py
4. Select initial planner plugin
5. Select initial controller plugin
6. Define initial costmap configuration
