#!/usr/bin/env python3
"""M2: FAST-LIO2 odometry -> PX4 EKF2 external vision.

Subscribes  /lidar_slam/odom              (nav_msgs/Odometry, from FAST-LIO2)
Publishes   /fmu/in/vehicle_visual_odometry (px4_msgs/VehicleOdometry)

Frame convention
----------------
FAST-LIO2's world frame (`lidar_init`, upstream `camera_init`) is gravity-aligned z-UP,
with its x-axis pointing wherever the IMU faced at initialisation -- i.e. FLU
with an arbitrary, constant heading offset from True North.

That is exactly PX4's POSE_FRAME_FRD:
    "Forward-Right-Down frame. Constant arbitrary heading offset from True
     North. Z is down."

so we publish FRD, *not* NED. Sending NED would claim the frame is North-aligned
and EKF2 would fight the magnetometer.

FLU -> FRD is a 180 deg rotation about x:
    position / velocity   (x, y, z) -> (x, -y, -z)
    quaternion (w,x,y,z)            -> (w, x, -y, -z)
Conjugating a rotation by Rx(pi) negates the y and z components of its axis,
which is why only those two flip.

Velocity
--------
Off by default. nav_msgs/Odometry says `twist` is in `child_frame_id` (lidar_imu),
but FAST-LIO forks disagree -- several publish it in the world frame. Feeding
EKF2 velocity in the wrong frame is worse than feeding none, so this stays
opt-in until verified on hardware. With it off, use EKF2_EV_CTRL=11
(horiz pos + vert pos + yaw); add the velocity bit (4) -> 15 only after
confirming the frame.

Divergence guard (G0.2, 2026-09-12)
-----------------------------------
On 2026-09-12 this bridge forwarded a FAST-LIO pose 3.74 km from the truth
(x -3340, y -1672, z -86.7 m) for ~9 minutes, and EKF2 accepted every sample:
xy_valid, z_valid, eph 0.06 m. The aircraft was armed on it twice. FAST-LIO had
received IMU but no LiDAR scans for ~225 s, integrated the IMU alone the whole
time, then resumed scan matching around the wrong pose and tracked it smoothly.
A diverged-then-settled estimate is precise, low-covariance and self-consistent.
Nothing downstream can tell it from the truth, because with GPS off EKF2 has
nothing absolute to compare it against. docs/fact-fastlio-divergence-2026-09-12.md
has the evidence.

So trust is LATCHED, not re-earned. The bridge forwards samples only while it is
TRACKING, and it latches DIVERGED on the first of:
  * a position step larger than max(max_step_m, max_speed_mps * dt);
  * a gap between consecutive sample STAMPS longer than max_gap_s. FAST-LIO
    publishes nothing while it has no scans, so a stamp gap is exactly an
    interval it integrated on IMU alone. Arrival gaps are deliberately NOT a
    trigger: under load the 09-09 bags show 2.1 s arrival gaps whose stamps are
    0.1-0.4 s apart -- every scan was matched, the output was just bunched;
  * a sample stamp that does not advance;
  * a first sample farther than max_start_radius_m from lidar_init's origin.
While DIVERGED it publishes nothing, so EKF2 times vision out rather than
fusing it. It never un-latches on the same FAST-LIO: the bad estimate SETTLES, so
"consistent again for N samples" would have accepted it on 09-12. It resumes
only for a NEW publisher (a restarted FAST-LIO, which starts a new lidar_init)
whose first sample is near the origin, and then it increments reset_counter so
EKF2 resets onto the new frame instead of fusing a 3 km innovation.

Restarting the bridge itself clears the latch, and that is why the start-radius
check exists. Restarting only the bridge in flight, far from where FAST-LIO
started, needs max_start_radius_m:=-1.0, deliberately.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

WAITING, TRACKING, DIVERGED = 'WAITING', 'TRACKING', 'DIVERGED'


def _finite(*vals):
    return all(math.isfinite(v) for v in vals)


class DivergenceGuard:
    """The trust state machine, kept free of ROS so it can be reasoned about alone.

    `admit()` is called once per finite input sample and returns True if that sample
    may be forwarded. `publishers` is the set of endpoint GIDs currently publishing
    the input topic; a change in it is how a FAST-LIO restart is recognised.
    """

    def __init__(self, max_step_m, max_speed_mps, max_gap_s, max_start_radius_m):
        self.max_step_m = max_step_m
        self.max_speed_mps = max_speed_mps
        self.max_gap_s = max_gap_s
        self.max_start_radius_m = max_start_radius_m
        self.state = WAITING
        self.reason = 'no odometry received yet'
        self.reset_counter = 0
        self.n_rejected = 0
        self._last = None             # (stamp_s, (x, y, z), arrival_s) of the last admitted sample
        self._trusted_pubs = None     # publisher GIDs when a sample was last admitted

    def _near_origin(self, p):
        return self.max_start_radius_m < 0 or math.hypot(*p) <= self.max_start_radius_m

    def _latch(self, reason):
        self.state = DIVERGED
        self.reason = reason
        self.n_rejected += 1

    def admit(self, stamp_s, p, now_s, publishers):
        publishers = frozenset(publishers)

        if self.state == TRACKING:
            dt = stamp_s - self._last[0]
            step = math.dist(p, self._last[1])
            if dt <= 0.0:
                self._latch('stamp did not advance (%.3f s): two publishers, or a clock jump'
                            % dt)
            elif dt > self.max_gap_s:
                self._latch('no scan-matched pose for %.1f s (stamp gap > max_gap_s %.1f): '
                            'FAST-LIO integrated IMU alone across it' % (dt, self.max_gap_s))
            elif step > max(self.max_step_m, self.max_speed_mps * dt):
                self._latch('position jumped %.2f m in %.2f s (limit %.2f m)'
                            % (step, dt, max(self.max_step_m, self.max_speed_mps * dt)))

        if self.state == WAITING:
            if not self._near_origin(p):
                self._latch('first sample is %.1f m from lidar_init origin (> max_start_radius_m '
                            '%.1f): FAST-LIO was not started here, or has already diverged'
                            % (math.hypot(*p), self.max_start_radius_m))
                self._trusted_pubs = publishers
                return False
            self.state, self.reason = TRACKING, ''
        elif self.state == DIVERGED:
            # An empty trusted set means we never knew who was publishing; that is not
            # evidence of a restart, so it can never resume on it.
            restarted = (bool(self._trusted_pubs) and publishers != self._trusted_pubs
                         and len(publishers) == 1)
            if not (restarted and self._near_origin(p)):
                self.n_rejected += 1
                return False
            self.reset_counter = (self.reset_counter + 1) % 256
            self.state, self.reason = TRACKING, ''

        self._last = (stamp_s, tuple(p), now_s)
        self._trusted_pubs = publishers
        return True


class Px4OdomBridge(Node):

    def __init__(self):
        super().__init__('px4_odom_bridge')

        self.declare_parameter('input_topic', '/lidar_slam/odom')
        self.declare_parameter('output_topic', '/fmu/in/vehicle_visual_odometry')
        self.declare_parameter('publish_velocity', False)
        # These are FLOORS, not just zero-fallbacks. Measured on hardware, FAST-LIO
        # reports position variance ~6.8e-6 m^2 (2.6 mm std). That is its *internal
        # relative* uncertainty, not its absolute drift -- passing it through tells
        # EKF2 to trust vision to millimetres, so EKF2 stops correcting against the
        # IMU and fights hard whenever FAST-LIO drifts or jumps. Clamp to something
        # honest: 0.01 m^2 => 0.1 m std.
        self.declare_parameter('min_position_variance', 0.01)
        self.declare_parameter('min_orientation_variance', 0.01)
        self.declare_parameter('min_velocity_variance', 0.04)
        # Divergence guard -- see the module docstring. 5 m/s is 3.3x the fastest planned
        # follow envelope (docs/m5-m6-plan.md S3, 1.5 m/s); 0.5 m is that speed over one
        # 10 Hz frame. Consecutive FAST-LIO stamps are 0.1 s apart and were never more than
        # 0.40 s apart in any recorded bag, loaded 09-09 runs included; 1.0 s of IMU-only
        # integration is a stall, not jitter (docs/fact-fastlio-divergence-2026-09-12.md).
        self.declare_parameter('max_step_m', 0.5)
        self.declare_parameter('max_speed_mps', 5.0)
        self.declare_parameter('max_gap_s', 1.0)
        self.declare_parameter('max_start_radius_m', 1.0)
        self.declare_parameter('status_topic', '~/status')

        self.in_topic = self.get_parameter('input_topic').value
        self.out_topic = self.get_parameter('output_topic').value
        self.pub_vel = self.get_parameter('publish_velocity').value
        self.pos_var = self.get_parameter('min_position_variance').value
        self.att_var = self.get_parameter('min_orientation_variance').value
        self.vel_var = self.get_parameter('min_velocity_variance').value
        self.guard = DivergenceGuard(
            max_step_m=float(self.get_parameter('max_step_m').value),
            max_speed_mps=float(self.get_parameter('max_speed_mps').value),
            max_gap_s=float(self.get_parameter('max_gap_s').value),
            max_start_radius_m=float(self.get_parameter('max_start_radius_m').value))

        # PX4 uXRCE-DDS expects BEST_EFFORT + TRANSIENT_LOCAL on /fmu/ topics.
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub = self.create_publisher(VehicleOdometry, self.out_topic, px4_qos)
        self.sub = self.create_subscription(
            Odometry, self.in_topic, self.on_odom, 10)
        # Latched, so a preflight or dashboard that subscribes late still gets the state.
        self.status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1))

        self.n_in = 0
        self.n_out = 0
        self.n_rejected = 0
        self.last_in_time = None
        self._publishers = frozenset()
        self._ticks = 0
        self._status = None
        self.create_timer(5.0, self.report)
        self.create_timer(0.1, self.tick)

        self.get_logger().info(
            f'px4_odom_bridge: {self.in_topic} -> {self.out_topic} '
            f'(FLU->FRD, publish_velocity={self.pub_vel}, divergence guard: '
            f'step {self.guard.max_step_m} m / {self.guard.max_speed_mps} m/s, '
            f'gap {self.guard.max_gap_s} s, start radius {self.guard.max_start_radius_m} m)')
        self._publish_status()

    def _status_text(self):
        g = self.guard
        text = f'{g.state} reset_counter={g.reset_counter}'
        return f'{text} reason={g.reason}' if g.reason else text

    def _publish_status(self, keepalive=False):
        """Publish the guard state on every change, and once a second regardless."""
        text = self._status_text()
        changed = text != self._status
        if changed:
            if self.guard.state == DIVERGED:
                self.get_logger().error(
                    f'DIVERGED -- NOT forwarding odometry to PX4: {self.guard.reason}. '
                    'Resumes only on a restarted FAST-LIO (new lidar_init) that starts near '
                    'the origin. Restart FAST-LIO where the aircraft is, on the ground.')
            elif self._status is not None:
                self.get_logger().warn(f'guard state -> {text}')
            self._status = text
        if changed or keepalive:
            self.status_pub.publish(String(data=text))

    def _lookup_publishers(self):
        return frozenset(
            bytes(i.endpoint_gid) for i in self.get_publishers_info_by_topic(self.in_topic))

    def tick(self):
        # Graph lookups here, not per message: 10 Hz of rcl graph calls buys nothing, and a
        # restart is seen one tick late at worst (the sample after it resumes instead).
        self._publishers = self._lookup_publishers()
        self._ticks += 1
        self._publish_status(keepalive=self._ticks % 10 == 0)
        if self.guard.state == DIVERGED:
            self.get_logger().error(
                f'DIVERGED, not forwarding: {self.guard.reason}', throttle_duration_sec=2.0)

    def on_odom(self, msg: Odometry):
        self.n_in += 1
        self.last_in_time = self.get_clock().now()

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # FAST-LIO can emit NaN/inf if it diverges (a known MID360 stall mode).
        # Never forward that into EKF2 -- it would corrupt the state estimate.
        if not _finite(p.x, p.y, p.z, q.w, q.x, q.y, q.z):
            self.n_rejected += 1
            if self.n_rejected % 20 == 1:
                self.get_logger().error(
                    'non-finite odometry from FAST-LIO, dropping '
                    f'(count={self.n_rejected}) -- has the LiDAR stalled?')
            return

        if not self._publishers:   # a sample before the first tick: look now, not never
            self._publishers = self._lookup_publishers()
        stamp = msg.header.stamp
        admitted = self.guard.admit(stamp.sec + stamp.nanosec * 1e-9, (p.x, p.y, p.z),
                                    self.last_in_time.nanoseconds / 1e9, self._publishers)
        self._publish_status()
        if not admitted:
            self.n_rejected += 1
            return

        out = VehicleOdometry()

        now_us = int(self.get_clock().now().nanoseconds / 1000)
        sample_us = int(stamp.sec * 1_000_000 + stamp.nanosec / 1000)
        out.timestamp = now_us
        out.timestamp_sample = sample_us if sample_us > 0 else now_us

        out.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        out.position = [float(p.x), float(-p.y), float(-p.z)]
        out.q = [float(q.w), float(q.x), float(-q.y), float(-q.z)]

        if self.pub_vel:
            v = msg.twist.twist.linear
            w = msg.twist.twist.angular
            if _finite(v.x, v.y, v.z, w.x, w.y, w.z):
                out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_FRD
                out.velocity = [float(v.x), float(-v.y), float(-v.z)]
                out.angular_velocity = [float(w.x), float(-w.y), float(-w.z)]
            else:
                self._invalidate_velocity(out)
        else:
            self._invalidate_velocity(out)

        # Variance is sign-invariant, so the FLU->FRD flip leaves it unchanged.
        # Floor it (see min_*_variance above) -- FAST-LIO's raw covariance is
        # wildly over-confident and would make EKF2 over-trust vision.
        pc = msg.pose.covariance
        out.position_variance = [
            self._floor(pc[0], self.pos_var),
            self._floor(pc[7], self.pos_var),
            self._floor(pc[14], self.pos_var),
        ]
        out.orientation_variance = [
            self._floor(pc[21], self.att_var),
            self._floor(pc[28], self.att_var),
            self._floor(pc[35], self.att_var),
        ]
        if self.pub_vel:
            tc = msg.twist.covariance
            out.velocity_variance = [
                self._floor(tc[0], self.vel_var),
                self._floor(tc[7], self.vel_var),
                self._floor(tc[14], self.vel_var),
            ]
        else:
            out.velocity_variance = [self.vel_var] * 3

        # Incremented only when the guard resumes on a restarted FAST-LIO: its new lidar_init
        # is a different frame, and EKF2 must reset onto it (ev_control.cpp:52), not fuse the
        # jump. Hardcoding 0 here is how a restart used to reach EKF2 as a 3 km innovation.
        out.reset_counter = self.guard.reset_counter
        out.quality = 0  # 0 == "unused" per the message definition

        self.pub.publish(out)
        self.n_out += 1

    @staticmethod
    def _floor(cov, minimum):
        """Clamp a reported variance up to `minimum`; non-finite/zero -> minimum."""
        c = float(cov)
        if not math.isfinite(c) or c <= 0.0:
            return float(minimum)
        return float(max(c, minimum))

    @staticmethod
    def _invalidate_velocity(out):
        """PX4 reads NaN as 'not provided' rather than 'zero'."""
        nan = float('nan')
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        out.velocity = [nan, nan, nan]
        out.angular_velocity = [nan, nan, nan]

    def report(self):
        if self.n_in == 0:
            self.get_logger().warn(
                f'no messages on {self.in_topic} yet -- is FAST-LIO running?')
            return
        age = 'n/a'
        if self.last_in_time is not None:
            age = f'{(self.get_clock().now() - self.last_in_time).nanoseconds / 1e9:.1f}s'
        self.get_logger().info(
            f'in={self.n_in} out={self.n_out} rejected={self.n_rejected} '
            f'last_input={age} ago guard={self._status_text()}')


def main(args=None):
    rclpy.init(args=args)
    node = Px4OdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
