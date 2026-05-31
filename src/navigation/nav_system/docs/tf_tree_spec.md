# TF Tree Specification

## Purpose

This document defines the initial TF frame structure required by the SAADS navigation stack.

TF is used in ROS 2 to describe spatial relationships between coordinate frames. Navigation, localization, sensor fusion, perception, and visualization all depend on a consistent TF tree.

---

## Initial TF Tree

```text
map
 └── odom
      └── base_link
           ├── lidar_frame
           ├── imu_frame
           ├── gps_frame
           └── camera_frame
```

---

## Frame Definitions

### map

Global navigation reference frame.

Used by:

* Nav2 global planning
* GPS waypoint navigation
* Map-based localization
* Mission planning

Expected publisher:

* Localization system
* SLAM/localization node
* GPS localization pipeline

---

### odom

Locally continuous odometry frame.

Used by:

* Nav2 local planning
* State estimation
* Robot motion tracking

Expected publisher:

* state_estimation

Notes:

The `odom` frame should be smooth and continuous, but it may drift over time.

---

### base_link

Main robot body frame.

Used by:

* Nav2
* Vehicle control
* Sensor transforms
* Robot description
* Visualization

Expected publisher:

* state_estimation publishes `odom -> base_link`
* robot description/static transforms define sensor frames relative to `base_link`

---

### lidar_frame

LiDAR mounting frame.

Parent:

```text
base_link
```

Expected data:

```text
/lidar/points_raw
/lidar/points_filtered
```

Message type:

```text
sensor_msgs/msg/PointCloud2
```

Notes:

LiDAR data should use `lidar_frame` as its frame ID.

Exact physical offsets are TODO.

---

### imu_frame

IMU mounting frame.

Parent:

```text
base_link
```

Expected data:

```text
/imu/data_raw
/imu/data
```

Message type:

```text
sensor_msgs/msg/Imu
```

Notes:

IMU orientation must be verified against the robot's physical forward direction.

---

### gps_frame

GPS receiver or antenna frame.

Parent:

```text
base_link
```

Expected data:

```text
/gps/fix
```

Message type:

```text
sensor_msgs/msg/NavSatFix
```

Notes:

GPS data must eventually be converted into a local navigation frame before direct Nav2 use.

GPS conversion strategy is currently unresolved.

---

### camera_frame

Main camera mounting frame.

Parent:

```text
base_link
```

Expected data:

```text
/camera/image_raw
/camera/camera_info
```

Message types:

```text
sensor_msgs/msg/Image
sensor_msgs/msg/CameraInfo
```

Notes:

Camera data may later support perception, obstacle classification, object detection, or operator visualization.

---

## Transform Responsibilities

### Dynamic Transforms

Dynamic transforms may change over time.

Expected dynamic transform chain:

```text
map -> odom
odom -> base_link
```

Likely publisher:

```text
state_estimation
```

The exact localization strategy is still under design.

Possible inputs:

* GPS
* IMU
* Wheel encoders
* LiDAR odometry
* Visual odometry

---

### Static Transforms

Static transforms describe fixed sensor mounting locations.

Expected static transforms:

```text
base_link -> lidar_frame
base_link -> imu_frame
base_link -> gps_frame
base_link -> camera_frame
```

Likely publisher:

```text
saads_description
```

Static transforms should come from the robot description or static transform publishers, not from individual sensor processing nodes when possible.

---

## Nav2 TF Requirements

Nav2 requires a valid transform chain between:

```text
map
odom
base_link
```

Minimum required chain:

```text
map -> odom -> base_link
```

Sensor data used by costmaps must also be transformable into the robot or map frame.

Example:

```text
lidar_frame -> base_link -> odom -> map
```

If this transform chain is missing, Nav2 costmaps and planning may fail.

---

## Costmap-Related Frames

Initial costmap assumption:

```text
local_costmap frame: odom
global_costmap frame: map
robot_base_frame: base_link
```

LiDAR obstacle data should initially publish in:

```text
lidar_frame
```

Nav2 costmaps must be able to transform LiDAR data into:

```text
base_link
odom
map
```

---

## GPS Considerations

Raw GPS topic:

```text
/gps/fix
```

Raw GPS message type:

```text
sensor_msgs/msg/NavSatFix
```

Nav2 cannot directly plan using raw latitude/longitude coordinates without conversion.

A GPS transformation/localization layer must define:

* GPS datum or origin
* Conversion into local coordinates
* Relationship between GPS frame and map frame
* How GPS waypoints become Nav2 goals

Potential future output:

```text
/gps/local_pose
```

Potential message type:

```text
geometry_msgs/msg/PoseStamped
```

---

## Future Optional Frames

If additional sensors are added, the following frames may be introduced:

```text
radar_frame
sonar_frame
depth_camera_frame
thermal_camera_frame
rtk_gps_frame
```

These should attach to `base_link` unless the sensor has a moving mount.

---

## Testing and Validation Commands

View available TF frames:

```bash
ros2 run tf2_tools view_frames
```

Inspect transform between two frames:

```bash
ros2 run tf2_ros tf2_echo base_link lidar_frame
```

Check active TF topics:

```bash
ros2 topic echo /tf
ros2 topic echo /tf_static
```

Visualize TF in RViz:

```bash
rviz2
```

Recommended RViz displays:

* TF
* RobotModel
* PointCloud2
* Odometry

---

## Validation Checklist

Before Nav2 testing, verify:

* map frame exists
* odom frame exists
* base_link frame exists
* map -> odom -> base_link chain is available
* sensor frames are attached to base_link
* LiDAR messages use lidar_frame
* IMU messages use imu_frame
* GPS messages are associated with gps_frame
* Camera messages use camera_frame
* TF tree can be visualized in RViz
* Nav2 can transform costmap sensor data into the correct frame

---

## Open Questions

* What package will publish `map -> odom`?
* What package will publish `odom -> base_link`?
* Will GPS provide global corrections to localization?
* What datum/origin will GPS conversion use?
* What are the exact physical sensor offsets?
* Will LiDAR data remain PointCloud2 or be converted to LaserScan for early Nav2 testing?
* Will camera data feed Nav2 directly or only perception/GUI?
* Will radar or sonar be added?
* How will sensor transforms be generated: URDF, static transform publishers, or custom launch files?

---

## Current Status

Initial status:

* Initial frame hierarchy defined
* Sensor frames identified
* Nav2 TF requirements documented
* GPS conversion remains unresolved
* Exact physical transform offsets remain TODO
* Dynamic transform ownership remains under design
