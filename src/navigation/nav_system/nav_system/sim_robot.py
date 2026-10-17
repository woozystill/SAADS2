#!/usr/bin/env python3
"""Bench simulator for SAADS - fakes the SENSORS, not the robot.

--------------------------------------------------------------------------
WHAT THIS IS
--------------------------------------------------------------------------
Subscribes to /cmd_vel, integrates where the robot would have gone, and
publishes the sensor streams the rest of the stack expects:

    /gps/fix        sensor_msgs/NavSatFix      5 Hz
    /imu/data       sensor_msgs/Imu           50 Hz
    /wheel_odom     nav_msgs/Odometry         10 Hz
    /unilidar/cloud sensor_msgs/PointCloud2   10 Hz   (optional, see below)
    /sim/ground_truth nav_msgs/Odometry       10 Hz   (the yardstick)

Nothing downstream changes. The real ekf_dual, the real navsat_transform,
the real /fromLL and real Nav2 all run unmodified on top. The only fake
thing in the chain is the hardware.

That matters: a simulator that replaced the EKFs would be testing a
pipeline you are not going to ship. This one tests the pipeline you are.

--------------------------------------------------------------------------
OBSTACLES
--------------------------------------------------------------------------
Set the `obstacles` parameter to a flat list of [x, y, radius, ...] in the
MAP frame, and the node ray-casts a point cloud against them from the
robot's simulated pose. Uncomment the voxel layers in
nav2_params_gps.yaml and Nav2 will see them.

    ros2 run nav_system sim_robot --ros-args \\
      -p obstacles:="[10.0, 0.0, 1.0,  15.0, 3.0, 0.8]"

That places a 1 m barrel 10 m east and a 0.8 m one at (15, 3).

The cloud is published in base_link by default, so no extra static
transform is needed. The real lidar will have its own frame and a real
offset - that is a hardware-side problem this cannot test.

WHAT THIS DOES NOT SIMULATE, and you must not conclude otherwise:
  - ground returns. The real failure mode with a 3D lidar is the costmap
    filling with the floor. min_obstacle_height can only be tuned against
    a real cloud.
  - occlusion behind obstacles beyond the first hit, beam divergence,
    reflectivity, rain, water surface.
  - CPU load of a real 5300-point cloud at full rate.

--------------------------------------------------------------------------
GPS DROPOUT
--------------------------------------------------------------------------
`gps_dropout_after` stops GPS publication once the robot has travelled
that many metres, so you can measure dead-reckoning error against ground
truth. This is how you turn the modelled 25 m outage tolerance into a
measured curve.

    ros2 run nav_system sim_robot --ros-args -p gps_dropout_after:=20.0

Then watch /odometry/global drift away from /sim/ground_truth.

--------------------------------------------------------------------------
IMPORTANT
--------------------------------------------------------------------------
Do NOT run this with the real drivers. Two publishers on /gps/fix or
/imu/data will interleave and the EKF will fuse nonsense:

    pkill -f bno085_node; pkill -f gps_driver; pkill -f wheel_odometry

Noise levels are the values MEASURED on real hardware, so the EKF behaves
here the way it behaves outdoors:

    GPS      5 Hz,  3.26 m 1-sigma   (localization_enh_02)
    IMU     50 Hz,  6.1 deg heading  (chip's own estimate)
    wheels  10 Hz,  0.1 m/s
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, PointCloud2
from sensor_msgs_py import point_cloud2

EARTH_RADIUS = 6378137.0
GRAVITY = 9.80665


class SimRobot(Node):

    def __init__(self):
        super().__init__('sim_robot')

        # ---- world ----
        self.declare_parameter('datum_lat', 32.2319)
        self.declare_parameter('datum_lon', -110.9501)
        self.declare_parameter('start_yaw', 0.0)

        # Flat list [x, y, radius, ...] in the map frame. Empty = open field.
        # rclpy infers a parameter's type from its default, and an empty
        # list comes out as BYTE_ARRAY. A zero-radius placeholder keeps it
        # a DOUBLE_ARRAY; radius <= 0 is filtered out below, so this still
        # means "no obstacles".
        self.declare_parameter('obstacles', [0.0, 0.0, 0.0])

        # ---- sensor rates and noise, measured on real hardware ----
        self.declare_parameter('gps_rate', 5.0)
        self.declare_parameter('imu_rate', 50.0)
        self.declare_parameter('odom_rate', 10.0)
        self.declare_parameter('gps_sigma', 3.26)
        # Correlation time of the GPS error, seconds. Real receiver error
        # is dominated by ionospheric delay and satellite geometry, which
        # drift over minutes - two fixes a second apart share almost all
        # of their error. White noise would make consecutive fixes
        # independent, which is both unrealistic and breaks any analysis
        # that measures distance or course between fixes.
        self.declare_parameter('gps_noise_tau', 60.0)
        # Fast receiver noise on top of the slow drift.
        self.declare_parameter('gps_white_sigma', 0.3)
        self.declare_parameter('heading_sigma_deg', 6.1)
        self.declare_parameter('wheel_sigma', 0.1)
        self.declare_parameter('add_noise', True)

        # ---- fake lidar ----
        self.declare_parameter('publish_lidar', True)
        self.declare_parameter('lidar_rate', 10.0)
        self.declare_parameter('lidar_range', 15.0)
        self.declare_parameter('lidar_beams', 360)
        self.declare_parameter('lidar_frame', 'base_link')
        self.declare_parameter('lidar_layers_z', [0.2, 0.4, 0.6])

        # ---- failure injection ----
        # Metres travelled before GPS stops. 0 disables.
        self.declare_parameter('gps_dropout_after', 0.0)

        # ---- physical limits ----
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 1.5)

        p = self.get_parameter
        self.lat0 = float(p('datum_lat').value)
        self.lon0 = float(p('datum_lon').value)
        self.gps_sigma = float(p('gps_sigma').value)
        self.gps_tau = max(1e-3, float(p('gps_noise_tau').value))
        self.gps_white = float(p('gps_white_sigma').value)
        self._gps_dt = 1.0 / float(p('gps_rate').value)
        # Gauss-Markov state, metres of error in east and north.
        self._gps_err_e = 0.0
        self._gps_err_n = 0.0
        self.yaw_sigma = math.radians(float(p('heading_sigma_deg').value))
        self.wheel_sigma = float(p('wheel_sigma').value)
        self.noise = bool(p('add_noise').value)
        self.max_v = float(p('max_linear_vel').value)
        self.max_w = float(p('max_angular_vel').value)
        self.lidar_range = float(p('lidar_range').value)
        self.lidar_beams = int(p('lidar_beams').value)
        self.lidar_frame = str(p('lidar_frame').value)
        self.lidar_z = [float(z) for z in p('lidar_layers_z').value]
        self.dropout_at = float(p('gps_dropout_after').value)

        raw = [float(v) for v in p('obstacles').value]
        if len(raw) % 3 != 0:
            self.get_logger().error(
                'obstacles must be a flat list of [x, y, radius, ...]. '
                f'Got {len(raw)} values, which is not a multiple of 3. '
                'Ignoring.')
            raw = []
        self.obstacles = [tuple(raw[i:i + 3])
                          for i in range(0, len(raw), 3)]
        self.obstacles = [o for o in self.obstacles if o[2] > 0.0]

        # ---- state ----
        self.x = 0.0
        self.y = 0.0
        self.yaw = float(p('start_yaw').value)
        self.vx = 0.0
        self.wz = 0.0
        self.distance = 0.0
        self.gps_live = True
        self.last_cmd_time = self.get_clock().now()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)

        self.gps_pub = self.create_publisher(NavSatFix, 'gps/fix', sensor_qos)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', sensor_qos)
        self.odom_pub = self.create_publisher(Odometry, 'wheel_odom', 10)
        self.truth_pub = self.create_publisher(Odometry, 'sim/ground_truth', 10)

        # Integration runs faster than any publisher, so the pose stays
        # accurate regardless of the output rates.
        self.integrate_dt = 0.01
        self.create_timer(self.integrate_dt, self.integrate)

        self.create_timer(1.0 / float(p('gps_rate').value), self.pub_gps)
        self.create_timer(1.0 / float(p('imu_rate').value), self.pub_imu)
        self.create_timer(1.0 / float(p('odom_rate').value), self.pub_odom)
        self.create_timer(2.0, self.report)

        if bool(p('publish_lidar').value):
            self.cloud_pub = self.create_publisher(
                PointCloud2, 'unilidar/cloud', sensor_qos)
            self.create_timer(
                1.0 / float(p('lidar_rate').value), self.pub_cloud)
        else:
            self.cloud_pub = None

        self.get_logger().info(
            f'Simulated hardware up. Datum {self.lat0:.5f}, {self.lon0:.5f}. '
            f'Noise {"on" if self.noise else "off"}. '
            f'{len(self.obstacles)} obstacle(s). '
            f'Lidar {"on" if self.cloud_pub else "off"}.')
        if self.dropout_at > 0:
            self.get_logger().warn(
                f'GPS will be CUT after {self.dropout_at:.1f} m travelled.')
        self.get_logger().warn(
            'The real drivers must not be running - two publishers on these '
            'topics will corrupt the EKF.')

    # ------------------------------------------------------------------
    def on_cmd(self, msg):
        self.vx = max(-self.max_v, min(self.max_v, float(msg.linear.x)))
        self.wz = max(-self.max_w, min(self.max_w, float(msg.angular.z)))
        self.last_cmd_time = self.get_clock().now()

    def integrate(self):
        # A real robot whose controller died should not coast forever.
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > 0.5:
            self.vx = 0.0
            self.wz = 0.0

        dt = self.integrate_dt
        # Midpoint on yaw, so a turning arc does not systematically cut
        # the corner.
        mid_yaw = self.yaw + self.wz * dt * 0.5
        step = self.vx * dt
        self.x += step * math.cos(mid_yaw)
        self.y += step * math.sin(mid_yaw)
        self.yaw = (self.yaw + self.wz * dt + math.pi) % (2 * math.pi) - math.pi
        self.distance += abs(step)

        if self.gps_live and 0 < self.dropout_at <= self.distance:
            self.gps_live = False
            self.get_logger().error(
                f'GPS CUT at {self.distance:.1f} m travelled. From here the '
                f'filter is dead reckoning - compare /odometry/global '
                f'against /sim/ground_truth.')

    def jitter(self, sigma):
        return random.gauss(0.0, sigma) if self.noise else 0.0

    # ------------------------------------------------------------------
    def pub_gps(self):
        if not self.gps_live:
            return

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        # Gauss-Markov drift: e <- e*rho + sqrt(1-rho^2)*sigma*N(0,1).
        # rho near 1 means the error barely changes between fixes, which
        # is how real GPS behaves.
        if self.noise:
            rho = math.exp(-self._gps_dt / self.gps_tau)
            step = math.sqrt(max(0.0, 1.0 - rho * rho)) * self.gps_sigma
            self._gps_err_e = self._gps_err_e * rho + random.gauss(0.0, step)
            self._gps_err_n = self._gps_err_n * rho + random.gauss(0.0, step)

        east = self.x + self._gps_err_e + self.jitter(self.gps_white)
        north = self.y + self._gps_err_n + self.jitter(self.gps_white)

        msg.latitude = self.lat0 + math.degrees(north / EARTH_RADIUS)
        msg.longitude = self.lon0 + math.degrees(
            east / (EARTH_RADIUS * math.cos(math.radians(self.lat0))))
        msg.altitude = 728.0

        var = self.gps_sigma ** 2
        msg.position_covariance = [var, 0.0, 0.0,
                                   0.0, var, 0.0,
                                   0.0, 0.0, var * 4.0]
        # 1 = APPROXIMATED, matching what the real receiver reports.
        msg.position_covariance_type = 1
        self.gps_pub.publish(msg)

    def pub_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        yaw = self.yaw + self.jitter(self.yaw_sigma)
        msg.orientation.z = math.sin(yaw / 2.0)
        msg.orientation.w = math.cos(yaw / 2.0)

        msg.angular_velocity.z = self.wz + self.jitter(0.0017)
        msg.linear_acceleration.z = GRAVITY

        msg.orientation_covariance = [
            0.0019, 0.0, 0.0,
            0.0, 0.0019, 0.0,
            0.0, 0.0, self.yaw_sigma ** 2,
        ]
        gyro_var = 3.022e-06
        msg.angular_velocity_covariance = [
            gyro_var, 0.0, 0.0,
            0.0, gyro_var, 0.0,
            0.0, 0.0, gyro_var,
        ]
        accel_var = 4.844e-04
        msg.linear_acceleration_covariance = [
            accel_var, 0.0, 0.0,
            0.0, accel_var, 0.0,
            0.0, 0.0, accel_var,
        ]
        self.imu_pub.publish(msg)

    def pub_odom(self):
        stamp = self.get_clock().now().to_msg()

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.twist.twist.linear.x = self.vx + self.jitter(self.wheel_sigma)
        msg.twist.twist.angular.z = self.wz + self.jitter(0.02)

        # Pose at 1e3: never fuse wheel position, same as the real node.
        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36
        for i in range(0, 36, 7):
            pose_cov[i] = 1e3
            twist_cov[i] = 1e3
        twist_cov[0] = self.wheel_sigma ** 2
        twist_cov[35] = 0.02
        msg.pose.covariance = pose_cov
        msg.twist.covariance = twist_cov
        self.odom_pub.publish(msg)

        # Ground truth, map frame, no noise. This is the yardstick - no
        # outdoor test can give you this.
        truth = Odometry()
        truth.header.stamp = stamp
        truth.header.frame_id = 'map'
        truth.child_frame_id = 'base_link'
        truth.pose.pose.position.x = self.x
        truth.pose.pose.position.y = self.y
        truth.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        truth.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        truth.twist.twist.linear.x = self.vx
        truth.twist.twist.angular.z = self.wz
        self.truth_pub.publish(truth)

    # ------------------------------------------------------------------
    def pub_cloud(self):
        """Ray-cast the obstacle list from the robot's true pose.

        One ray per beam. For each, the nearest circle intersection within
        range becomes a point, replicated at each z layer so the voxel
        layer has something above min_obstacle_height to mark.

        Points are in the robot frame, which is what the costmap needs
        after transforming. Publishing in base_link avoids needing a
        static lidar transform that does not exist in simulation.
        """
        if self.cloud_pub is None:
            return

        points = []
        if self.obstacles:
            for i in range(self.lidar_beams):
                bearing = 2.0 * math.pi * i / self.lidar_beams
                # Ray direction in the map frame.
                world = self.yaw + bearing
                dx, dy = math.cos(world), math.sin(world)

                nearest = None
                for (ox, oy, r) in self.obstacles:
                    # Circle-ray intersection: solve |P + t*D - C|^2 = r^2
                    fx, fy = self.x - ox, self.y - oy
                    b = 2.0 * (fx * dx + fy * dy)
                    c = fx * fx + fy * fy - r * r
                    disc = b * b - 4.0 * c
                    if disc < 0.0:
                        continue
                    root = math.sqrt(disc)
                    for t in ((-b - root) * 0.5, (-b + root) * 0.5):
                        if 0.0 < t <= self.lidar_range:
                            if nearest is None or t < nearest:
                                nearest = t
                            break

                if nearest is None:
                    continue

                # Back into the robot frame: bearing is already relative
                # to the robot's heading.
                px = nearest * math.cos(bearing)
                py = nearest * math.sin(bearing)
                for z in self.lidar_z:
                    points.append([px, py, z])

        msg = point_cloud2.create_cloud_xyz32(
            self._header(self.lidar_frame), points)
        self.cloud_pub.publish(msg)

    def _header(self, frame):
        from std_msgs.msg import Header
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame
        return h

    # ------------------------------------------------------------------
    def report(self):
        if abs(self.vx) > 1e-3 or abs(self.wz) > 1e-3:
            gps = 'GPS on' if self.gps_live else 'GPS CUT'
            self.get_logger().info(
                f'truth  x {self.x:7.2f}  y {self.y:7.2f}  '
                f'yaw {math.degrees(self.yaw):6.1f}  '
                f'travelled {self.distance:6.1f} m  [{gps}]')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SimRobot()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
