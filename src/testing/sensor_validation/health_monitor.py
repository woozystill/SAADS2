#!/usr/bin/env python3
"""Live sensor health dashboard.

Redraws in place, like show_yaw.py, showing what every sensor currently
says about its own confidence. Answers "how do you know if a sensor is
degraded?" by showing it happening rather than describing it.

WHAT IS REAL AND WHAT IS A DISPLAY CONVENIENCE

  Covariances are real. They come from the messages and are what
  robot_localization actually weights by.

  The trust percentages are NOT used by anything. They are a reading
  aid, mapping a covariance onto 0-100% against thresholds chosen for
  this robot. A panel asking "where does 62% come from?" deserves the
  honest answer: it is a display scale, and the metre figure beside it
  is the measurement.

WHAT EACH ROW MEANS

  GPS position    receiver-reported, from satellite geometry via HDOP.
                  Varies during a run as satellites move.

  IMU heading     the chip's own per-reading accuracy estimate, read
                  from the rotation vector report. Adapts.

  IMU gyro/accel  measured from a stationary recording. Fixed - the
                  chip offers only a qualitative level for these, so
                  there is nothing live to show.

  EKF position    the filter's own uncertainty. Grows while predicting
                  between GPS fixes, collapses when one arrives. This
                  is the catch-all: it rises whatever the cause.

  Cross-check     IMU heading against GPS course over ground. Course
                  comes from position change, so magnetism cannot touch
                  it. The difference is a fixed mounting rotation; if it
                  starts moving, the magnetometer is being disturbed -
                  which no self-reported value would reveal.

Usage:
    python3 health_monitor.py
    python3 health_monitor.py --plain     # no ANSI, for logging
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2

# ANSI
CLEAR = '\033[2J\033[H'
HIDE = '\033[?25l'
SHOW = '\033[?25h'
BOLD = '\033[1m'
DIM = '\033[2m'
RESET = '\033[0m'
GREEN = '\033[32m'
YELLOW = '\033[33m'
RED = '\033[31m'
GREY = '\033[90m'

FIX_STATUS = {-1: 'no fix', 0: 'fix', 1: 'fix + SBAS', 2: 'fix + GBAS'}

# The chip's own four-level accuracy scale, carried in the status byte
# of every report. Qualitative - there is no figure in physical units
# for the accelerometer or gyroscope, only this.
ACCURACY = {0: 'unreliable', 1: 'low', 2: 'medium', 3: 'high'}

# Trust scale endpoints. Below the first value reads 100%, above the
# second reads 0%. Chosen for this robot, not derived from anything.
GPS_BEST, GPS_WORST = 1.0, 20.0        # metres, 1 sigma
HEAD_BEST, HEAD_WORST = 1.0, 45.0      # degrees, 1 sigma
EKF_BEST, EKF_WORST = 1.0, 20.0        # metres, 1 sigma

# Wheel and IMU yaw rate measure the same quantity by different
# means, so a persistent gap between them is slip or a wrong track
# width rather than noise.
SLIP_WARN = 0.15                       # rad/s disagreement
SLIP_ERROR = 0.40


def trust(value, best, worst):
    """Map a 1-sigma figure onto 0-100%.

    Logarithmic, because sensor quality spans orders of magnitude and a
    linear scale would sit pinned at one end. Purely for display.
    """
    if value <= best:
        return 100.0
    if value >= worst:
        return 0.0
    lo, hi, v = math.log(best), math.log(worst), math.log(value)
    return 100.0 * (1.0 - (v - lo) / (hi - lo))


def bar(pct, width=12, plain=False):
    filled = int(round(pct / 100.0 * width))
    body = '#' * filled + '.' * (width - filled)
    if plain:
        return f'[{body}] {pct:3.0f}%'
    colour = GREEN if pct >= 60 else YELLOW if pct >= 25 else RED
    return f'[{colour}{body}{RESET}] {colour}{pct:3.0f}%{RESET}'


class Rate:
    def __init__(self, window=40):
        self.t = []
        self.window = window

    def tick(self):
        self.t.append(time.monotonic())
        if len(self.t) > self.window:
            self.t.pop(0)

    def hz(self):
        if len(self.t) < 2:
            return 0.0
        span = self.t[-1] - self.t[0]
        return (len(self.t) - 1) / span if span > 0 else 0.0

    def age(self):
        return float('inf') if not self.t else time.monotonic() - self.t[-1]


class HealthMonitor(Node):

    def __init__(self, plain=False):
        super().__init__('health_monitor')
        self.plain = plain

        self.declare_parameter('refresh_hz', 4.0)
        self.declare_parameter('crosscheck_min_move', 0.5)

        self.min_move = float(
            self.get_parameter('crosscheck_min_move').value)

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        self.r_gps = Rate()
        self.r_imu = Rate()
        self.r_ekf = Rate()
        self.r_lidar = Rate()
        self.r_wheel = Rate()

        self.gps = None
        self.imu = None
        self.ekf = None
        self.started = time.monotonic()

        self.lidar = None
        self.wheel = None
        self.imu_diag = None

        self.last_pos = None
        self.offset = None        # running mean, the mounting rotation
        self.deviation = None     # departure from it

        self.create_subscription(NavSatFix, '/gps/fix', self.cb_gps, 10)
        self.create_subscription(Imu, '/imu/data', self.cb_imu, best_effort)
        self.create_subscription(Odometry, '/odometry/global',
                                 self.cb_ekf, 10)
        self.create_subscription(PointCloud2, '/unilidar/cloud',
                                 self.cb_lidar, best_effort)
        self.create_subscription(DiagnosticArray, '/imu/diagnostics',
                                 self.cb_diag, 10)
        self.create_subscription(Odometry, '/wheel_odom',
                                 self.cb_wheel, 10)

        period = 1.0 / float(self.get_parameter('refresh_hz').value)
        self.create_timer(period, self.draw)

        if not plain:
            sys.stdout.write(HIDE)

    # ---------------- callbacks ----------------
    def cb_gps(self, msg):
        self.r_gps.tick()
        self.gps = msg

    def cb_imu(self, msg):
        self.r_imu.tick()
        self.imu = msg

    def cb_lidar(self, msg):
        self.r_lidar.tick()
        self.lidar = msg

    def cb_wheel(self, msg):
        self.r_wheel.tick()
        self.wheel = msg

    def cb_diag(self, msg):
        self.imu_diag = msg

    def cb_ekf(self, msg):
        self.r_ekf.tick()
        self.ekf = msg
        self.cross_check(msg)

    def cross_check(self, odom):
        if self.imu is None:
            return
        p = odom.pose.pose.position
        now = (p.x, p.y)
        if self.last_pos is None:
            self.last_pos = now
            return
        dx, dy = now[0] - self.last_pos[0], now[1] - self.last_pos[1]
        if math.hypot(dx, dy) < self.min_move:
            return
        self.last_pos = now

        course = math.degrees(math.atan2(dy, dx)) % 360.0
        o = self.imu.orientation
        yaw = math.degrees(math.atan2(
            2 * (o.w * o.z + o.x * o.y),
            1 - 2 * (o.y * o.y + o.z * o.z))) % 360.0
        diff = (yaw - course + 540.0) % 360.0 - 180.0

        if self.offset is None:
            self.offset = diff
            return
        self.deviation = (diff - self.offset + 540.0) % 360.0 - 180.0
        self.offset += 0.02 * self.deviation

    # ---------------- drawing ----------------
    def hdr(self, text):
        return text if self.plain else f'{BOLD}{text}{RESET}'

    def bad(self, text):
        return text if self.plain else f'{RED}{text}{RESET}'

    def dim(self, text):
        return text if self.plain else f'{DIM}{text}{RESET}'

    def level(self, sensor):
        """The chip's own accuracy for one sensor, formatted inline.

        Returns a blank string when that sensor's report is not enabled
        - the status byte arrives inside the report, so a sensor that is
        not reporting has no level to show. The runtime launch leaves
        the magnetometer report off, so its level reads '-' there.
        """
        if self.imu_diag is None:
            return ''
        for st in self.imu_diag.status:
            if not st.name.endswith(sensor):
                continue
            if st.level == DiagnosticStatus.STALE:
                txt = 'level -'
                return txt if self.plain else f'{GREY}{txt}{RESET}'
            lv = st.message.split('(')[-1].split('/')[0]
            txt = f'level {lv}'
            if self.plain:
                return txt
            c = GREEN if st.level == DiagnosticStatus.OK else YELLOW
            return f'{c}{txt}{RESET}'
        return ''

    def section(self, name, detail=''):
        """One section header, padded before any colour codes are added.

        ANSI sequences count toward string length but not display width,
        so the padding has to happen on the plain text first.
        """
        padded = f'{name:<14}'
        return '  ' + self.hdr(padded) + detail

    def row(self, label, value, extra=''):
        return f'    {label:<14}{value:<26}{extra}'

    def draw(self):
        L = []
        elapsed = time.monotonic() - self.started
        mm, ss = divmod(int(elapsed), 60)
        hh, mm = divmod(mm, 60)
        L.append('')
        L.append('  ' + self.hdr(f'{"SENSOR HEALTH":<14}')
                 + self.dim(f'clock {time.strftime("%H:%M:%S")}')
                 + self.dim(f'     elapsed {hh:02d}:{mm:02d}:{ss:02d}'))
        L.append('  ' + self.dim('-' * 62))
        L.append('')

        # ---- GPS ----
        if self.gps is None or self.r_gps.age() > 5:
            L.append(self.section('GPS', self.bad('no data')))
        else:
            var = self.gps.position_covariance[0]
            sigma = math.sqrt(var) if var > 0 else float('nan')
            fix = self.gps.status.status
            L.append(self.section(
                'GPS', f'/gps/fix         {self.r_gps.hz():5.1f} Hz'))
            L.append(self.row('fix status',
                              f'{fix}  ({FIX_STATUS.get(fix, "?")})'))
            if fix >= 0 and not math.isnan(sigma):
                L.append(self.row('position',
                                  f'{sigma:.2f} m 1-sigma',
                                  bar(trust(sigma, GPS_BEST, GPS_WORST),
                                      plain=self.plain)))
            else:
                L.append(self.row('position', 'unavailable'))
        L.append('')

        # ---- IMU ----
        if self.imu is None or self.r_imu.age() > 2:
            L.append(self.section('IMU', self.bad('no data')))
        else:
            o = self.imu.orientation
            norm = math.sqrt(o.x**2 + o.y**2 + o.z**2 + o.w**2)
            yv = self.imu.orientation_covariance[8]
            ysig = math.degrees(math.sqrt(yv)) if yv > 0 else 0.0
            gv = self.imu.angular_velocity_covariance[0]
            av = self.imu.linear_acceleration_covariance[0]

            L.append(self.section(
                'IMU', f'/imu/data        {self.r_imu.hz():5.1f} Hz'))
            L.append(self.row(
                'heading', f'{ysig:.2f} deg 1-sigma',
                bar(trust(ysig, HEAD_BEST, HEAD_WORST), plain=self.plain)
                + '  | ' + self.level('orientation')))
            L.append(self.row(
                'gyroscope', f'{math.sqrt(gv):.5f} rad/s 1-sig',
                self.dim('measured, fixed') + '  | '
                + self.level('gyroscope')))
            L.append(self.row(
                'accelerometer', f'{math.sqrt(av):.5f} m/s2 1-sig',
                self.dim('measured, fixed') + '  | '
                + self.level('accelerometer')))
            L.append(self.row(
                'magnetometer',
                self.dim('inside the quaternion'),
                self.dim('no covariance') + '  | '
                + self.level('magnetometer')))
            flag = '' if abs(norm - 1.0) < 0.01 else '  <- INVALID'
            L.append(self.row('quaternion', f'norm {norm:.5f}{flag}'))

        L.append('')

        # ---- wheel odometry ----
        if self.wheel is None or self.r_wheel.age() > 2:
            L.append(self.section(
                'WHEEL ODOM', self.bad('no data')))
        else:
            tw = self.wheel.twist.twist
            vx_var = self.wheel.twist.covariance[0]
            vyaw_var = self.wheel.twist.covariance[35]
            L.append(self.section(
                'WHEEL ODOM', f'/wheel_odom      {self.r_wheel.hz():5.1f} Hz'))
            L.append(self.row('forward', f'{tw.linear.x:+.3f} m/s',
                              self.dim(f'1-sig {math.sqrt(vx_var):.3f}')))
            L.append(self.row('yaw rate', f'{tw.angular.z:+.3f} rad/s',
                              self.dim(f'1-sig {math.sqrt(vyaw_var):.3f}')))
        L.append('')

        # ---- EKF ----
        if self.ekf is None or self.r_ekf.age() > 2:
            L.append(self.section('EKF global', self.bad('no data')))
        else:
            var = self.ekf.pose.covariance[0]
            sigma = math.sqrt(var) if var > 0 else 0.0
            L.append(self.section(
                'EKF global', f'/odometry/global {self.r_ekf.hz():5.1f} Hz'))
            L.append(self.row('position', f'{sigma:.2f} m 1-sigma',
                              bar(trust(sigma, EKF_BEST, EKF_WORST),
                                  plain=self.plain)
                              + '  ' + self.dim('grows between fixes')))
        L.append('')

        # ---- Lidar ----
        if self.lidar is None or self.r_lidar.age() > 3:
            L.append(self.section('LIDAR', self.bad('no data')))
        else:
            pts = self.lidar.width * self.lidar.height
            L.append(self.section(
                'LIDAR', f'/unilidar/cloud  {self.r_lidar.hz():5.1f} Hz'))
            L.append(self.row('points', f'{pts} per cloud'))
        L.append('')

        # ---- cross-check ----
        L.append(self.section(
            'HEADING X-Check',
            self.dim('   IMU heading vs GPS course over ground')))
        if self.deviation is None:
            L.append(self.row('status', self.dim('   waiting for motion')))
        else:
            dev = abs(self.deviation)
            verdict = ('OK' if dev < 15 else
                       'DISTURBED' if dev < 30 else 'UNTRUSTWORTHY')
            if not self.plain:
                c = GREEN if dev < 15 else YELLOW if dev < 30 else RED
                verdict = f'{c}{verdict}{RESET}'
            L.append(self.row('mounting offset', f'{self.offset:+.1f} deg'))
            L.append(self.row('deviation', f'{dev:.1f} deg', verdict))
        # ---- wheel vs IMU yaw rate ----
        # Two independent measurements of the same quantity. A gap that
        # persists while turning means the track width is wrong; a gap
        # that appears only under acceleration or on loose ground is
        # wheel slip, which nothing else in the stack can see.
        L.append('')
        L.append(self.section(
            'SLIP X-Check', self.dim('   wheel yaw rate vs IMU yaw rate')))
        have_both = (self.wheel is not None and self.imu is not None
                     and self.r_wheel.age() < 2 and self.r_imu.age() < 2)
        if not have_both:
            L.append(self.row('status',
                              self.dim('   waiting for wheel odometry')))
        else:
            w_yaw = self.wheel.twist.twist.angular.z
            i_yaw = self.imu.angular_velocity.z
            gap = abs(w_yaw - i_yaw)
            verdict = ('OK' if gap < SLIP_WARN else
                       'SLIP?' if gap < SLIP_ERROR else 'DISAGREE')
            if not self.plain:
                c = (GREEN if gap < SLIP_WARN else
                     YELLOW if gap < SLIP_ERROR else RED)
                verdict = f'{c}{verdict}{RESET}'
            L.append(self.row('wheels', f'{w_yaw:+.3f} rad/s'))
            L.append(self.row('IMU', f'{i_yaw:+.3f} rad/s',
                              f'gap {gap:.3f}  ' + verdict))
        L.append('')

        L.append('  ' + self.dim('-' * 62))
        L.append('  ' + self.dim('trust bars are a display scale; the '
                                 'covariance beside each is the measurement'))
        L.append('')

        out = '\n'.join(L)
        if self.plain:
            print(out, flush=True)
        else:
            sys.stdout.write(CLEAR + out)
            sys.stdout.flush()


def main():
    plain = '--plain' in sys.argv
    rclpy.init()
    node = None
    try:
        node = HealthMonitor(plain=plain)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not plain:
            sys.stdout.write(SHOW + '\n')
            sys.stdout.flush()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
