"""sensor_watchdog -- detect a sensor that has gone SILENT while everything still looks healthy.

WHY THIS EXISTS
---------------
On 2026-09-07 the Orbbec driver **stalled**: colour and depth both stopped publishing while the
container process stayed alive (1175 s uptime), the USB device stayed enumerated at 5 Gbps, every
topic stayed registered, and **nothing was logged**. It recovered fully on restart. Nothing in
the system noticed.

That is the dangerous shape of failure for a follower. A crash is loud and something restarts it;
a stall is silent, and a follow controller keeps commanding from its last known target while the
world moves on. No existing gate catches it either -- `check_camera_serial` and
`check_lidar_source` both verify identity at START-UP and never look again.

WHAT IT WATCHES, AND WHY NOT THE IMAGES
---------------------------------------
It watches the TINY companion topics, never the payloads:

    /camera/color/camera_info   0.41 KB @ 30 Hz   proxy for the colour stream
    /camera/depth/camera_info   0.41 KB @ 30 Hz   proxy for the depth stream
    /lidar_slam/odom            small    @ 10 Hz   proves LiDAR *and* FAST-LIO are alive

Measured 2026-09-07: `camera_info` tracks the image cadence exactly (29.96 vs 29.97 Hz) at
1/2000th the bytes. Subscribing to `/camera/depth/image_raw` (24 MB/s) would make the watchdog a
load source in its own right -- and on this box a Python subscriber on a multi-MB/s topic drops
frames, so it would invent the very outages it exists to detect.

`/lidar_slam/odom` is chosen deliberately over `/livox/lidar`: it is small, and being downstream
of FAST-LIO its health proves the whole LiDAR chain rather than just the driver.

STALLED vs NEVER-ARRIVED
------------------------
These are reported differently and must not be conflated. A topic that never produced anything is
usually start-up ordering -- nodes come up in any order and that is not a fault. A topic that WAS
publishing and stopped is a stall, which is the condition worth acting on. Only the second raises
the alarm during the grace period.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool

# (topic, message type, nominal Hz). All small by design -- see the module docstring.
DEFAULT_WATCH = (
    ('/camera/color/camera_info', CameraInfo, 30.0),
    ('/camera/depth/camera_info', CameraInfo, 30.0),
    ('/lidar_slam/odom', Odometry, 10.0),
)

# A stream is STALE after this many nominal periods with nothing. Loose enough to ride out a
# scheduling hiccup, tight enough that a follower does not act on a stale world for long: at
# 30 Hz this is 0.33 s, at 10 Hz 1.0 s.
STALE_PERIODS = 10.0
# Below this fraction of nominal the stream is DEGRADED -- alive but not keeping up.
RATE_FLOOR = 0.7
# Do not call a topic that has NEVER published a fault before this; nodes start in any order.
STARTUP_GRACE_S = 20.0


class Watched:
    def __init__(self, topic, nominal_hz):
        self.topic = topic
        self.nominal_hz = nominal_hz
        self.count = 0
        self.last_stamp = None      # monotonic seconds of the most recent message
        self.recent = []            # arrival times, for a windowed rate
        self.ever_seen = False

    def note(self, now):
        self.count += 1
        self.ever_seen = True
        self.last_stamp = now
        self.recent.append(now)
        cutoff = now - 3.0
        while self.recent and self.recent[0] < cutoff:
            self.recent.pop(0)

    # A rate computed over a very short span is meaningless, and right after a stall it is
    # actively misleading: the driver dumps queued frames on reconnect, so the first samples
    # arrive in a burst. That produced "recovered (188.1 Hz)" on a 30 Hz stream on 2026-09-07.
    # Requiring a real span makes the reported number trustworthy, which matters because a
    # watchdog nobody believes is a watchdog nobody acts on.
    MIN_SPAN_S = 1.0

    def rate(self, now):
        if len(self.recent) < 2:
            return 0.0
        span = self.recent[-1] - self.recent[0]
        if span < self.MIN_SPAN_S:
            return 0.0        # "not yet measurable", not "stopped" -- callers check age for that
        return (len(self.recent) - 1) / span

    def age(self, now):
        return None if self.last_stamp is None else now - self.last_stamp


class SensorWatchdog(Node):
    def __init__(self):
        super().__init__('sensor_watchdog')

        self.declare_parameter('stale_periods', STALE_PERIODS)
        self.declare_parameter('rate_floor', RATE_FLOOR)
        self.declare_parameter('startup_grace_s', STARTUP_GRACE_S)
        self._stale_periods = float(self.get_parameter('stale_periods').value)
        self._rate_floor = float(self.get_parameter('rate_floor').value)
        self._grace = float(self.get_parameter('startup_grace_s').value)

        self._t0 = self._now()
        self._watched = {}
        self._subs = []
        for topic, msg_type, hz in DEFAULT_WATCH:
            w = Watched(topic, hz)
            self._watched[topic] = w
            # default arg binds the current w -- a closure over the loop variable would make
            # every callback update the LAST topic only.
            self._subs.append(self.create_subscription(
                msg_type, topic,
                lambda _msg, ww=w: ww.note(self._now()),
                qos_profile_sensor_data))

        self._diag_pub = self.create_publisher(DiagnosticArray, '/dtp/sensor_health', 10)
        # A single cheap boolean for command_mux: it must be able to veto on sensor loss without
        # parsing diagnostics on its control path.
        self._ok_pub = self.create_publisher(Bool, '/dtp/sensors_ok', 10)

        # PERCEPTION HEALTH -- rate is not enough here, and that is the whole point.
        #
        # perception_node publishes an EMPTY TargetArray at a healthy 15 Hz whenever it cannot
        # lift a frame into the world (no TF chain, depth/colour skew rejected, no camera_info).
        # Every rate-based check in this file would call that healthy, while downstream it
        # silently EMPTIES command_mux's human-proximity veto: the veto iterates an empty list,
        # finds no violation, and a blind pipeline reads as a calm one. A watchdog that only
        # measures cadence cannot see the difference between "nobody is here" and "I cannot see".
        # So perception states it directly on /dtp/perception_ok and we fold it in.
        #
        # Absence is NOT a fault: perception_node may legitimately not be running (bench sensor
        # bringup, calibration). Only an explicit False vetoes.
        self._perception_ok = True
        self._perception_stamp = None
        self.create_subscription(
            Bool, '/dtp/perception_ok',
            lambda m: (setattr(self, '_perception_ok', bool(m.data)),
                       setattr(self, '_perception_stamp', self._now())),
            10)

        self._prev_level = {}
        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            'sensor_watchdog up, watching %s (companion topics only -- never the payloads)'
            % ', '.join(t for t, _, _ in DEFAULT_WATCH))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        now = self._now()
        booting = (now - self._t0) < self._grace
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        all_ok = True

        for topic, w in self._watched.items():
            period = 1.0 / w.nominal_hz
            age = w.age(now)
            rate = w.rate(now)
            st = DiagnosticStatus(name='dtp/sensor/%s' % topic.strip('/').replace('/', '_'),
                                  hardware_id=topic)

            if not w.ever_seen:
                # Never arrived: ordering, not a stall -- unless we are past the grace period.
                if booting:
                    st.level, st.message = DiagnosticStatus.WARN, 'waiting for first message'
                else:
                    st.level, st.message = DiagnosticStatus.ERROR, 'NEVER PUBLISHED'
                    all_ok = False
            elif age is not None and age > self._stale_periods * period:
                # Was publishing and stopped. This is the 2026-09-07 driver stall.
                st.level = DiagnosticStatus.ERROR
                st.message = 'STALLED -- %.2f s since last message (nominal %.0f Hz)' % (age, w.nominal_hz)
                all_ok = False
            elif rate > 0 and rate < w.nominal_hz * self._rate_floor:
                st.level = DiagnosticStatus.WARN
                st.message = 'DEGRADED -- %.1f Hz of %.0f' % (rate, w.nominal_hz)
            elif rate == 0.0:
                # Arriving (age is fresh) but the window is too short to state a rate yet.
                st.level, st.message = DiagnosticStatus.OK, 'streaming (rate settling)'
            else:
                st.level, st.message = DiagnosticStatus.OK, '%.1f Hz' % rate

            st.values = [KeyValue(key='rate_hz', value='%.2f' % rate),
                         KeyValue(key='age_s', value='%.3f' % (age if age is not None else -1.0)),
                         KeyValue(key='count', value=str(w.count))]
            arr.status.append(st)

            # Log only on transitions: a watchdog that repeats itself every tick gets muted, and
            # a muted watchdog is worse than none.
            if self._prev_level.get(topic) != st.level:
                if st.level == DiagnosticStatus.ERROR:
                    self.get_logger().error('%s: %s' % (topic, st.message))
                elif st.level == DiagnosticStatus.WARN:
                    self.get_logger().warn('%s: %s' % (topic, st.message))
                else:
                    self.get_logger().info('%s: recovered (%s)' % (topic, st.message))
                self._prev_level[topic] = st.level

        # Fold in perception's own verdict. Once it has spoken, a False is latched into
        # sensors_ok until it recovers -- a blind follower must stop the drone, not be outvoted
        # by three healthy sensor cadences.
        if self._perception_stamp is not None and not self._perception_ok:
            st = DiagnosticStatus(name='dtp/perception/blind',
                                  hardware_id='/dtp/perception_ok')
            st.level = DiagnosticStatus.ERROR
            st.message = ('perception is ALIVE BUT BLIND -- publishing empty TargetArrays, so the '
                          'human-proximity veto has nothing to act on')
            arr.status.append(st)
            all_ok = False
            if self._prev_level.get('__perception__') != st.level:
                self.get_logger().error('/dtp/perception_ok: %s' % st.message)
                self._prev_level['__perception__'] = st.level
        elif self._perception_stamp is not None:
            if self._prev_level.get('__perception__') not in (None, DiagnosticStatus.OK):
                self.get_logger().info('/dtp/perception_ok: recovered')
            self._prev_level['__perception__'] = DiagnosticStatus.OK

        self._diag_pub.publish(arr)
        self._ok_pub.publish(Bool(data=all_ok))


def main(args=None):
    rclpy.init(args=args)
    node = SensorWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException is how SIGTERM arrives -- the normal stop path for systemd
        # and `ros2 launch`. Without it, every managed shutdown exits with a stack trace.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
