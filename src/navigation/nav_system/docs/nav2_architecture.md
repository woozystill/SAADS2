# Nav2 Architecture Plan

## Purpose

The nav_system package is responsible for integrating Nav2 into the SAADS ROS 2 architecture. 
Nav2 will provide global path planning, local control, costmap-based obstacle avoidance, and 
navigation behavior execution.

---

## Nav2 Role in SAADS

Nav2 sits between localization/perception and vehicle control.

```text
Localization + Sensors
        ↓
Costmaps
        ↓
Nav2 Planner
        ↓
Nav2 Controller
        ↓
/cmd_vel
        ↓
vehicle_controller
        ↓
actuators
```

---

## Current Design Decisions

- Nav2 will run on the Jetson.
- The navigation stack will support a hybrid approach:
  - map-based planning
  - GPS waypoint-based navigation
- Initial planner direction:
  - A*/Smac Planner
- LiDAR will publish PointCloud2 data.
- LiDAR data will feed costmaps through a filtered point cloud topic.
- The vehicle controller will consume `/cmd_vel`.
- Raspberry Pi will support diagnostics, health monitoring, telemetry, and mission supervision.
- The final actuator interface is currently unresolved.

---

## Known Unknowns

- GPS coordinate conversion strategy
- GPS frame and local origin/datum
- Final actuator interface
- Exact drivetrain control model
- Whether LiDAR data must be converted from PointCloud2 to LaserScan for initial Nav2 testing
- Final camera role in navigation
- Whether radar/sonar will be included

---

## Initial Navigation Strategy

Development should begin with map-based Nav2 before GPS waypoint navigation.

Reason:

Map-based navigation requires fewer unresolved assumptions and allows early testing of:

- TF
- odometry
- costmaps
- planner server
- controller server
- `/cmd_vel` output

After map-based navigation is stable, GPS waypoint support can be added.

Development order:

1. Map-based Nav2 launch
2. Static/fake TF and odometry test
3. LiDAR costmap integration
4. GPS waypoint interface
5. Hybrid map + GPS behavior
6. Mission execution integration

---

## Minimum Required Interfaces

### Inputs to Nav2

```text
/tf
/tf_static
/odom
/lidar/points_filtered
/map or GPS-derived navigation reference
/mission/goal
```
### Outputs from Nav2

```text
/cmd_vel
/plan
/navigation/status
```
### Initial TF Assumption

```text
map
 └── odom
      └── base_link
           ├── lidar_frame
           ├── imu_frame
           ├── gps_frame
           └── camera_frame
```

### Deployment Assumption

Jetson

Runs:
-Nav2
-LiDAR driver/processing
-Camera processing
-Localization
-Vehicle Controller

Raspberry Pi:

Runs:
-Diagnostics
-Resource monitoring
-Telemetry
-Mission Supervision
-Health/Status reporting
