# SAADS ROS 2 Workspace

## Overview

SAADS is organized by subsystem rather than by vendor, sensor model, or individual scripts. This architecture is designed to support modular sensor integration, distributed computation, maintainability, scalability, and future expansion.

The workspace follows a layered robotics architecture:

```text
Sensors
    ↓
sensor_drivers
    ↓
sensor_processing
    ↓
localization
    ↓
navigation
    ↓
control
    ↓
actuators
```

The goal is to keep each subsystem focused on a single responsibility while allowing components to be replaced without affecting the rest of the system.

---

## Current Status

This repository currently contains the architectural foundation and ROS2 package structure for the SAADS robotics platform.

The workspace is presently focused on:

- System architecture design
- ROS2 package organization
- Sensor integration planning
- Navigation and localization planning
- Testing framework development

Most packages currently contain scaffolding and configuration required for future implementation.

Upcoming development phases include:

1. Sensor driver implementation
2. Sensor processing implementation
3. Localization integration
4. Navigation (Nav2) integration
5. Vehicle control integration
6. GUI and operator tooling

---
## Design Goals

* Modular subsystem architecture
* Hardware abstraction through drivers
* ROS 2 native communication
* Support for Jetson and Raspberry Pi deployment
* Sensor vendor independence
* Maintainable and scalable codebase
* Support for future GUI and operator tools
* Separation of hardware, processing, navigation, and control
* Support future perception and autonomy expansion

---

## Architecture Philosophy

SAADS follows a layered robotics architecture where each subsystem has a clearly defined responsibility.

The objectives are to:

- Minimize coupling between subsystems
- Maximize modularity and maintainability
- Support future sensor replacement
- Support distributed computation across multiple devices
- Simplify testing and validation
- Encourage ROS2-native communication patterns

Subsystems should communicate primarily through ROS2 topics, services, actions, and TF rather than direct dependencies whenever possible.

---
## Workspace Structure
```text
src/
├── control/
│   ├── teleop_interface/
│   └── vehicle_controller/
├── localization/
│   └── state_estimation/
├── navigation/
│   └── nav_system/
├── platform/
│   ├── saads_bringup/
│   ├── saads_common/
│   └── saads_description/
├── sensor_drivers/
│   ├── camera_driver/
│   ├── encoder_driver/
│   ├── gps_driver/
│   ├── imu_driver/
│   └── lidar_driver/
├── sensor_processing/
│   ├── gps_transform/
│   ├── image_preprocessing/
│   └── pointcloud_filter/
├── system/
│   ├── diagnostics/
│   └── resource_monitor/
└── testing/
    └── sensor_validation/
```
---

# Platform

## saads_bringup

Responsible for launching major system components.

Examples:

* sensors_launch.py
* processing_launch.py
* localization_launch.py
* navigation_launch.py
* system_launch.py
* platform_launch.py

This package serves as the primary entry point for starting the robot.

---

## saads_common

Shared utilities, constants, helper functions, and common interfaces used across packages.

Examples:

* Shared configuration utilities
* Constants
* Common helper functions
* Future shared message interfaces

---

## saads_description

Robot description package.

Future responsibilities:

* URDF/XACRO
* TF frame definitions
* Sensor mounting transforms
* Robot geometry
* Robot physical dimensions

Example future TF tree:

```text
map
 └── odom
      └── base_link
           ├── lidar_frame
           ├── camera_frame
           ├── imu_frame
           └── gps_frame
```

---

# Sensor Drivers

Raw hardware interfaces.

Packages:

* lidar_driver
* camera_driver
* imu_driver
* gps_driver
* encoder_driver

Responsibilities:

* Connect to hardware
* Communicate with SDKs
* Publish raw ROS messages
* Provide hardware abstraction

Examples:

```text
/lidar/points_raw
/camera/image_raw
/imu/data_raw
/gps/fix
/encoder/odom_raw
```

Drivers should remain lightweight and avoid heavy processing whenever possible.

---

# Sensor Processing

Transforms raw sensor data into more useful information.

Packages:

* pointcloud_filter
* image_preprocessing
* gps_transform

Examples:

```text
/lidar/points_raw
    ↓
pointcloud_filter
    ↓
/lidar/points_filtered
```

```text
/gps/fix
    ↓
gps_transform
    ↓
/gps/local_pose
```

Responsibilities:

* Filtering
* Noise reduction
* Coordinate transformations
* Data normalization
* Sensor-specific preprocessing

---

# Future Perception Layer

Perception is expected to be added later as system complexity grows.

Potential packages:

* obstacle_detection
* terrain_analysis
* object_detection
* target_tracking

Example flow:

```text
/lidar/points_filtered
    ↓
obstacle_detection
    ↓
/obstacles
```

Outputs from perception may feed:

* Nav2 costmaps
* Mission planning
* Autonomous behaviors

---

# Localization

Packages:

* state_estimation

Responsibilities:

* Sensor fusion
* Robot pose estimation
* Odometry generation
* State estimation

Potential inputs:

* IMU
* GPS
* Wheel encoders
* Lidar odometry

Outputs:

```text
/odom
robot pose
TF transforms
```

Localization provides the robot state information required by navigation.

---

# Navigation

Packages:

* nav_system

Responsibilities:

* Nav2 integration
* Planner configuration
* Controller configuration
* Costmap configuration
* Path planning validation

Navigation consumes:

* Localization information
* Obstacle information
* Costmap information

Navigation generates:

```text
/cmd_vel
```

Future work includes:

* Planner tuning
* Controller tuning
* Autonomous mission execution
* Navigation testing and validation

---

# Control

Packages:

* vehicle_controller
* teleop_interface

Responsibilities:

## vehicle_controller

Converts navigation commands into robot-specific actuator commands.

Example:

```text
/cmd_vel
    ↓
motor commands
```

Responsibilities:

* Motor control
* Steering control
* Velocity control
* Actuator interfacing

---

## teleop_interface

Provides manual control and operator override capability.

Examples:

* Joystick control
* Keyboard control
* Remote operator commands
* Emergency stop interfaces

---

# System

Packages:

* diagnostics
* resource_monitor

Responsibilities:

* Sensor health monitoring
* CPU monitoring
* Memory monitoring
* Error reporting
* System status monitoring
* Logging

Example monitored metrics:

* Topic publication rate
* Sensor status
* CPU usage
* RAM usage
* Disk usage
* Network health

---

# Testing

Packages:

* sensor_validation

Responsibilities:

* Verify sensor operation
* Verify topic publication
* Verify message rates
* Verify frame configuration
* Integration testing

Examples:

* Lidar validation
* IMU validation
* GPS validation
* Navigation validation
* TF validation

Testing should be repeatable and independent of implementation details.

---

# Expected Data Flow

Example sensor pipeline:

```text
Lidar Hardware
    ↓
lidar_driver
    ↓
pointcloud_filter
    ↓
obstacle_detection
    ↓
Nav2 Costmap
    ↓
nav_system
    ↓
vehicle_controller
    ↓
robot actuators
```

Example localization pipeline:

```text
GPS
IMU
Encoders
    ↓
state_estimation
    ↓
/odom
    ↓
nav_system
```

---

# Future Hardware Deployment

## Jetson

Expected responsibilities:

* Sensor drivers
* Point cloud processing
* Localization
* Perception
* Nav2
* Vehicle control

High-performance compute workloads should remain local to the Jetson.

---

## Raspberry Pi

Expected responsibilities:

* Diagnostics
* Monitoring
* Telemetry
* GPS
* Support services

The Raspberry Pi may act as a support processor for non-critical workloads.

---

# Python vs C++

Python is expected to be used for:

* Bringup
* Diagnostics
* Monitoring
* Testing
* GUI integration
* Mission orchestration

C++ is expected to be used for:

* Lidar drivers
* Point cloud processing
* Performance-critical nodes
* Real-time control
* Custom Nav2 plugins

Language decisions should prioritize maintainability and performance requirements.

---

# Design Principles

1. Organize by subsystem, not vendor.
2. Standardize interfaces, not implementations.
3. Keep drivers lightweight.
4. Separate processing from hardware access.
5. Make components independently testable.
6. Design for distributed computation.
7. Prefer modularity and maintainability.
8. Support future sensor replacement with minimal system changes.
9. Keep hardware-specific code isolated.
10. Build around ROS 2 communication standards.

