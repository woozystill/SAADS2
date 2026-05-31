# Navigation Interface Specification

## Purpose

The nav_system package is responsible for autonomous navigation,
path planning, obstacle avoidance, and mission execution using Nav2.

---

## Required Inputs

### Localization

Topic:
    /odom

Type:
    nav_msgs/msg/Odometry

Provided By:
    state_estimation

---

### TF

Frames:

    map
      └── odom
            └── base_link

Provided By:
    state_estimation

---

### LiDAR

Topic:
    /lidar/points_filtered

Type:
    sensor_msgs/msg/PointCloud2

Provided By:
    pointcloud_filter

Purpose:
    Obstacle detection and costmap updates

---

### GPS

Topic:
    /gps/fix

Type:
    sensor_msgs/msg/NavSatFix

Provided By:
    gps_driver

Purpose:
    Global waypoint navigation

---

### Mission Goals

Topic:
    /mission/goal

Purpose:
    Receive navigation goals from operator or mission manager

---

## Outputs

### Velocity Commands

Topic:
    /cmd_vel

Type:
    geometry_msgs/msg/Twist

Consumed By:
    vehicle_controller

---

### Navigation Status

Topic:
    /navigation/status

Purpose:
    Report navigation state and mission progress

---

### Planned Path

Topic:
    /plan

Purpose:
    Visualization and debugging

---

## TF Frames

map
 └── odom
      └── base_link
           ├── lidar_frame
           ├── imu_frame
           ├── gps_frame
           └── camera_frame

---

## Dependencies

state_estimation
pointcloud_filter
gps_transform
vehicle_controller

---

## Future Sensors

Potential future integrations:

- Radar
- Sonar
- Additional cameras
- RTK GPS
