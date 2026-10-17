#!/usr/bin/env python3
"""ROS 2 side of the web GCS: PX4 telemetry + topic rate monitors.

Everything the GUI shows about the aircraft comes off /fmu/out/* over uXRCE-DDS,
so the GUI never touches MAVLink and never contends for UDP 14569 with
scripts/px4/*.py.  ARK's dds-agent.service is always up, so these topics are
live even with the SUPER stack stopped.

CPU is the constraint, not bandwidth.  rclpy costs roughly 1.6 ms of Python
per message on this Jetson, so what you subscribe to matters far more than
what you do with it -- a first cut of this file that also watched
/fmu/out/vehicle_odometry (100 Hz) and /livox/imu (200 Hz) burned 28% of a
core doing nothing.  Three rules keep it near 3%:

  1. Never subscribe to a topic whose health another subscription already
     proves.  /fmu/out/vehicle_odometry only showed "the XRCE link is alive",
     which vehicle_status freshness already shows.  /livox/imu only showed
     "the driver is alive", which /lidar_slam/odom at 10 Hz already shows --
     FAST-LIO cannot produce odometry without IMU and LiDAR.
  2. Every subscription is raw=True, so DDS hands us bytes and no message is
     ever deserialised in a callback.
  3. Values are deserialised lazily in snapshot(), at the poll rate (2 Hz),
     not the publish rate (50 Hz for vehicle_local_position).
  4. The high-rate subscriptions exist only while a browser is actually
     polling.  vehicle_local_position alone is 7.6% of a core at 50 Hz even
     raw, and there is no reason to pay it while nobody is looking -- which is
     most of a bench session, and all of the time if the GCS is left running.
     Idle cost is then ~2%.

The RC card adds a THIRD tier on the same rule.  /fmu/out/vehicle_attitude is
100 Hz and costs 7.5% of a core even raw (measured on this Jetson: 0.75 ms per
message), which is why it exists only while that card is open and visible --
not merely while a browser is polling.  Altitude and heading cost nothing
extra: they come off vehicle_local_position, which the estimator panel already
subscribes to.

Also deliberately not subscribed: /cloud_registered and /rog_map/*.  Their
liveness comes from count_publishers(), and their rate is the same 10 Hz as
/lidar_slam/odom -- the same FAST-LIO callback publishes both.
"""
import math
import threading
import time
from collections import deque

import camera
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

# SUPER's follow-goal path answers every goal on /planning/goal_status, which
# turns three behaviours it otherwise applies SILENTLY (z override, the 0.1 m
# drop, the snap) into reported ones.  The message comes from the DTP patch and
# a clean setup.sh install of this repo does not have it -- so this is optional
# by construction, and rc.choose_path() falls back to the click path.
try:
    from dtp_msgs.msg import GoalStatus
except ImportError:                              # no DTP tree on this machine
    GoalStatus = None

GOAL_STATUS_NAME = {0: 'ACCEPTED', 1: 'SNAPPED', 2: 'DROPPED'}
from px4_msgs.msg import (BatteryStatus, EstimatorStatusFlags, OffboardControlMode,
                          TrajectorySetpoint, VehicleAttitude,
                          VehicleControlMode, VehicleLandDetected,
                          VehicleLocalPosition, VehicleOdometry, VehicleStatus)

import rc

# The house mode table -- same two lines as offboard_engage.py:54 and
# m3_gate_check.py:77.  Derived from the message so it cannot drift from px4_msgs.
NAV = {v: k[len('NAVIGATION_STATE_'):]
       for k, v in vars(VehicleStatus).items() if k.startswith('NAVIGATION_STATE_')}
ARMING = {v: k[len('ARMING_STATE_'):]
          for k, v in vars(VehicleStatus).items() if k.startswith('ARMING_STATE_')}
BATT_WARN = {v: k[len('WARNING_'):]
             for k, v in vars(BatteryStatus).items() if k.startswith('WARNING_')}

# PX4 publishes BEST_EFFORT/TRANSIENT_LOCAL.  A default (RELIABLE) subscription
# silently never matches and never errors -- see docs/fact-super-cmd-stream.md.
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=5)

# What SUPER offers on its own topics (fsm_ros2.hpp:328).  Volatile, not
# transient-local, so unlike PX4_QOS there is no last value waiting on connect:
# the first status arrives with the first goal after the card is opened.
SUPER_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=1)

RATE_WINDOW = 3.0          # seconds of arrivals kept per topic
TOPIC_TTL = 3.0            # graph query cache for the bag recorder's picker
STALE_AFTER = 3.0          # a /fmu/* value older than this is reported stale
IDLE_AFTER = 15.0          # no snapshot() for this long -> drop the hot subs

# Topics counted but never decoded: one chip each for the three links that can
# independently break -- M1 odometry, the M2 bridge's output, the M3 bridge's.
# The one rate topic whose CONTENT is also wanted: /lidar_slam/odom is the
# aircraft's pose in lidar_init, which is the frame every RC goal is expressed
# in.  Counting it and keeping its last bytes is the same callback.
# The M3 bridge has two chips since 2026-09-28: its heartbeat (always 50 Hz while
# it runs) and its setpoints, which it sends ONLY while PX4 is in Offboard -- so
# "silent" on trajectory_setpoint is normal on the ground and in manual modes, and
# a fault only in Offboard.  Outside Offboard the cost is what the one chip was.
ODOM_STORE = {'/lidar_slam/odom': 'odom'}

RATE_TOPICS = {
    '/lidar_slam/odom':                 (Odometry, None, 10.0),
    '/fmu/in/vehicle_visual_odometry':  (VehicleOdometry, PX4_QOS, 10.0),
    '/fmu/in/offboard_control_mode':    (OffboardControlMode, PX4_QOS, 50.0),
    '/fmu/in/trajectory_setpoint':      (TrajectorySetpoint, PX4_QOS, 50.0),
}

# The follower's two state strings, read only while a browser polls (the hot
# tier): /dtp/lock_state is ~1 Hz, /mux/state up to ~35 Hz -- kept raw and
# decoded once per poll like everything else.  BEST_EFFORT/KEEP_LAST(1) matches
# a reliable or a best-effort publisher alike, so the QoS the DTP nodes chose
# cannot silently leave this unmatched (docs/fact-super-cmd-stream.md).
DTP_STATE_TOPICS = {'lock': '/dtp/lock_state', 'mux': '/mux/state'}
# Designating a target is a parameter on target_manager, exactly what
# field.sh arm / reengage do: follow_enabled false, a beat, then true.
FOLLOW_PARAM_SERVICE = '/target_manager/set_parameters'
FOLLOW_PARAM = 'follow_enabled'
FOLLOW_REENGAGE_GAP_S = 0.7

# Reported by publisher presence only -- these are either huge (point clouds)
# or already implied by a rate above.
PRESENCE_TOPICS = ['/livox/lidar', '/livox/imu', '/cloud_registered',
                   '/rog_map/occ', '/rog_map/inf_occ', '/planning/pos_cmd',
                   '/fmu/out/vehicle_odometry']


def _f(x):
    """JSON-safe float: NaN/inf become None rather than invalid JSON."""
    try:
        x = float(x)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


class Telemetry(Node):

    def __init__(self):
        super().__init__('gcs_telemetry')
        self._lock = threading.Lock()
        self._raw = {}         # name -> (monotonic stamp, serialised bytes)
        self._decoded = {}     # name -> (monotonic stamp, message)  [cache]
        self._types = {}
        self._rates = {t: deque(maxlen=1024) for t in RATE_TOPICS}
        self._topics = {'t': 0.0, 'rows': []}

        # Always on -- 7 msg/s between them, so the panel is live the instant
        # someone opens the page.
        for name, typ, topic in (
                ('status',   VehicleStatus,        '/fmu/out/vehicle_status_v1'),
                ('battery',  BatteryStatus,        '/fmu/out/battery_status_v1'),
                ('estflags', EstimatorStatusFlags, '/fmu/out/estimator_status_flags'),
                ('ctrlmode', VehicleControlMode,   '/fmu/out/vehicle_control_mode'),
                ('land',     VehicleLandDetected,  '/fmu/out/vehicle_land_detected')):
            self._types[name] = typ
            self.create_subscription(typ, topic, self._store(name), PX4_QOS,
                                     raw=True)

        self._types['lpos'] = VehicleLocalPosition
        self._types['odom'] = Odometry
        self._types['att'] = VehicleAttitude
        if GoalStatus is not None:
            self._types['gstatus'] = GoalStatus
        for k in DTP_STATE_TOPICS:
            self._types[k] = String
        self._hot = []
        self._last_poll = 0.0
        # The RC tier: vehicle_attitude is 100 Hz and 7.5% of a core even raw,
        # so unlike the others it is not enough that a browser is polling --
        # the RC card has to actually be open and visible.
        self._rc_subs = []
        self._rc_poll = 0.0
        # Graph queries, cached per topic: at the RC card's 5 Hz an uncached
        # count_subscribers() per poll is exactly the cost aircraft_only() was
        # split out to avoid.
        self._graph = {}
        self._status = None        # last GoalStatus, decoded (it is ~1 Hz)
        # Publishing a goal is exactly what RViz's 2D Goal Pose tool does, and
        # SUPER subscribes BEST_EFFORT/KEEP_LAST(1) (fsm_ros2.hpp:330). A
        # RELIABLE publisher is compatible with that (offered >= requested),
        # which is why the RViz click path works -- so mirror RViz and use the
        # default profile rather than inventing one.
        self._goal_pub = self.create_publisher(PoseStamped, rc.GOAL_TOPIC_DEFAULT, 10)
        # The DTP patch's second goal input.  Created unconditionally and this
        # early on purpose: rc.choose_path() decides contention from the
        # publisher count on this topic, and "ours is exactly one of them" is
        # only true if it always exists.
        self._follow_pub = self.create_publisher(
            PoseStamped, rc.FOLLOW_GOAL_TOPIC, 10)
        # The Engage FOLLOW button's client.  Created here, on the executor's
        # side of things, so the HTTP thread only ever calls it; a client with
        # no server behind it costs nothing and service_is_ready() says so.
        self._param_cli = self.create_client(SetParameters, FOLLOW_PARAM_SERVICE)
        # Create/destroy the hot subscriptions from a timer, i.e. on the
        # executor thread -- doing it from an HTTP thread races the executor.
        self.create_timer(0.5, self._retune)

    # -- demand-driven subscriptions ---------------------------------------
    def _retune(self):
        wanted = (self._now() - self._last_poll) < IDLE_AFTER
        if wanted and not self._hot:
            self._hot.append(self.create_subscription(
                VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
                self._store('lpos'), PX4_QOS, raw=True))
            for topic, (typ, qos, _hz) in RATE_TOPICS.items():
                self._hot.append(self.create_subscription(
                    typ, topic, self._tick(topic, ODOM_STORE.get(topic)),
                    qos if qos is not None else 10, raw=True))
            for name, topic in DTP_STATE_TOPICS.items():
                self._hot.append(self.create_subscription(
                    String, topic, self._store(name), SUPER_QOS, raw=True))
        rc_wanted = (self._now() - self._rc_poll) < IDLE_AFTER
        if rc_wanted and not self._rc_subs:
            self._rc_subs.append(self.create_subscription(
                VehicleAttitude, '/fmu/out/vehicle_attitude',
                self._store('att'), PX4_QOS, raw=True))
            if GoalStatus is not None:
                # BEST_EFFORT is not optional: fsm publishes this
                # best_effort/keep_last(1)/volatile (fsm_ros2.hpp:328), and a
                # default RELIABLE subscription never matches and never errors
                # -- the same trap docs/fact-super-cmd-stream.md documents for
                # /planning/pos_cmd, which cost a session there.
                self._rc_subs.append(self.create_subscription(
                    GoalStatus, rc.GOAL_STATUS_TOPIC, self._store('gstatus'),
                    SUPER_QOS, raw=True))
        elif not rc_wanted and self._rc_subs:
            for sub in self._rc_subs:
                self.destroy_subscription(sub)
            self._rc_subs.clear()
            with self._lock:
                for k in ('att', 'gstatus'):
                    self._raw.pop(k, None)
                    self._decoded.pop(k, None)

        if not wanted and self._hot:
            for sub in self._hot:
                self.destroy_subscription(sub)
            self._hot.clear()
            with self._lock:
                # Drop what they fed, so nothing stale is ever rendered as
                # live -- the RC card would otherwise put a goal at a pose the
                # aircraft left minutes ago.
                for k in ('lpos', 'odom', *DTP_STATE_TOPICS):
                    self._raw.pop(k, None)
                    self._decoded.pop(k, None)
                for dq in self._rates.values():
                    dq.clear()

    # -- callbacks ---------------------------------------------------------
    def _store(self, name):
        """Keep the bytes only.  Decoding here would run at the publish rate;
        vehicle_local_position alone arrives at 50 Hz and is read at 2 Hz."""
        def cb(raw):
            with self._lock:
                self._raw[name] = (self._now(), raw)
        return cb

    def _tick(self, topic, store=None):
        """Count arrivals, and optionally keep the last bytes.

        Storing is what lets the RC card put a goal in lidar_init: the odometry
        subscription already exists for its rate chip, so the pose is free --
        one decode per poll rather than one per message, like everything else.
        """
        dq = self._rates[topic]

        def cb(raw):
            now = self._now()
            with self._lock:
                dq.append(now)
                if store:
                    self._raw[store] = (now, raw)
        return cb

    @staticmethod
    def _now():
        return time.monotonic()

    # -- snapshot ----------------------------------------------------------
    def _get(self, name):
        """Return (msg, age_seconds) or (None, None).

        Deserialises at most once per snapshot: the cache is keyed on the
        arrival stamp, so a topic that has not published since the last poll
        costs nothing, and one that publishes at 50 Hz still costs one decode
        per poll rather than fifty.
        """
        entry = self._raw.get(name)
        if entry is None:
            return None, None
        stamp, raw = entry
        cached = self._decoded.get(name)
        if cached is not None and cached[0] == stamp:
            msg = cached[1]
        else:
            try:
                msg = deserialize_message(raw, self._types[name])
            except Exception:
                return None, None
            self._decoded[name] = (stamp, msg)
        return msg, self._now() - stamp

    def snapshot(self):
        self._last_poll = self._now()
        with self._lock:
            out = {
                'aircraft':  self._aircraft(),
                'battery':   self._battery(),
                'estimator': self._estimator(),
                'rates':     self._rate_table(),
                'dtp':       self._dtp_state(),
            }
        # count_publishers() takes the node's own lock; call it outside ours.
        out['presence'] = {t: self.count_publishers(t) for t in PRESENCE_TOPICS}
        out['link'] = out['aircraft']['fresh']
        out['hot'] = bool(self._hot)
        return out

    # -- remote control ----------------------------------------------------
    def rc_snapshot(self):
        """What the RC card polls at 5 Hz.  Also the "I am open" signal: the
        100 Hz attitude subscription exists only because this was called."""
        self._rc_poll = self._now()
        self._last_poll = self._now()      # keep lpos hot too: altitude, heading
        with self._lock:
            pose = self._odom_pose()
            att = self._attitude()
            status = self._goal_status()
            lp, lp_age = self._get('lpos')
        out = {'pose': pose, 'attitude': att,
               'altitude': None, 'heading': None, 'z_valid': None,
               'attitude_live': bool(self._rc_subs),
               'pose_ned': rc.to_px4_ned(pose),
               'goal_status': status,
               'status_available': GoalStatus is not None}
        if lp is not None and lp_age is not None and lp_age < STALE_AFTER:
            # PX4 is NED: down is positive, so altitude above the EKF2 origin
            # is -z. This is the FC's own idea of height, not FAST-LIO's.
            out['altitude'] = _f(-lp.z)
            out['heading'] = _f(lp.heading)
            out['z_valid'] = bool(lp.z_valid)
        return out

    def _dtp_state(self):
        """The follower's lock and mux strings, as last published.

        `fresh` is per topic: the lock state is published about once a second
        and the mux state many times a second, and either going quiet is
        itself worth showing (a stopped follower leaves both stale).
        """
        out = {}
        for name in DTP_STATE_TOPICS:
            m, age = self._get(name)
            out[name] = m.data if m is not None else None
            out[name + '_fresh'] = age is not None and age < 5.0
        out['available'] = self._param_cli.service_is_ready()
        return out

    def set_follow_enabled(self, value, timeout=3.0):
        """Set target_manager's follow_enabled over its parameter service.

        Called from the HTTP thread; the executor thread completes the
        future, so this only waits on it -- never spins.  Returns (ok, why).
        """
        if not self._param_cli.service_is_ready():
            return False, (f'{FOLLOW_PARAM_SERVICE} is not being served -- is '
                           'target_manager (the DTP follower) running?')
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name=FOLLOW_PARAM,
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                 bool_value=bool(value)))]
        fut = self._param_cli.call_async(req)
        deadline = time.monotonic() + timeout
        while not fut.done():
            if time.monotonic() > deadline:
                fut.cancel()
                return False, f'target_manager did not answer within {timeout:.0f} s'
            time.sleep(0.02)
        try:
            res = fut.result().results[0]
        except Exception as exc:
            return False, f'parameter service failed: {exc}'
        if not res.successful:
            return False, f'target_manager refused: {res.reason or "no reason given"}'
        return True, None

    def follow(self, on):
        """Engage (or disengage) FOLLOW the way field.sh arm does.

        An engage is always false -> true: target_manager re-designates on the
        rising edge, so a follower that is LOST (or already engaged and
        ACQUIRING) is re-armed by the same button.
        """
        if on:
            ok, why = self.set_follow_enabled(False)
            if not ok:
                return ok, why
            time.sleep(FOLLOW_REENGAGE_GAP_S)
        return self.set_follow_enabled(bool(on))

    def aircraft_only(self):
        """The armed/Offboard/landed block on its own.

        snapshot() also decodes the battery and the estimator and runs a
        count_publishers() graph query per presence topic; at the RC card's
        5 Hz that was most of the cost of the card. Three cached decodes here.
        """
        with self._lock:
            return self._aircraft()

    def _odom_pose(self):
        """The aircraft in lidar_init, from the odometry we already count."""
        msg, age = self._get('odom')
        if msg is None:
            return None
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        return {'x': _f(p.x), 'y': _f(p.y), 'z': _f(p.z),
                'yaw': _f(rc.yaw_from_quat(q.x, q.y, q.z, q.w)),
                'age': None if age is None else round(age, 2)}

    def _attitude(self):
        msg, age = self._get('att')
        if msg is None or age is None or age > STALE_AFTER:
            return None
        w, x, y, z = (float(v) for v in msg.q)
        # PX4's quaternion is FRD-in-NED, which is what an artificial horizon
        # wants: roll right positive, pitch up positive, yaw clockwise.
        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        return {
            'roll': _f(math.degrees(math.atan2(2.0 * (w * x + y * z),
                                               1.0 - 2.0 * (x * x + y * y)))),
            'pitch': _f(math.degrees(math.asin(sinp))),
            'yaw': _f(math.degrees(math.atan2(2.0 * (w * z + x * y),
                                              1.0 - 2.0 * (y * y + z * z)))),
            'age': round(age, 2),
        }

    def _goal_status(self):
        """What SUPER did with the last goal on the follow path.

        The topic has more than one publisher in the DTP stack and fsm answers
        EVERY message on it, so `requested` is the correlation key -- `adopted`
        is the PREVIOUS goal when a goal was DROPPED, which is exactly when a
        caller most wants to know it was theirs (GoalStatus.msg says so).
        """
        msg, age = self._get('gstatus')
        if msg is None or age is None:
            return None
        return {
            'status': int(msg.status),
            'name': GOAL_STATUS_NAME.get(int(msg.status), str(msg.status)),
            'adopted': {'x': _f(msg.goal_p.x), 'y': _f(msg.goal_p.y),
                        'z': _f(msg.goal_p.z)},
            'requested': {'x': _f(msg.requested_p.x), 'y': _f(msg.requested_p.y),
                          'z': _f(msg.requested_p.z)},
            'age': round(age, 2),
        }

    def publish_goal(self, goal, topic=None):
        """One PoseStamped -- the RViz click, from a browser.

        `topic` selects the path rc.choose_path() picked: the click goal, or
        SUPER's follow-goal input, which keeps the z and reports what it did.
        The message is identical; only the destination differs.

        Published straight from the HTTP thread, unlike subscription
        create/destroy which has to happen on the executor thread: publishing
        does not mutate the executor's entity set, and a goal that waited for
        the next 0.5 s timer tick would feel broken to press.
        """
        msg = PoseStamped()
        msg.header.frame_id = rc.GOAL_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(goal['x'])
        msg.pose.position.y = float(goal['y'])
        msg.pose.position.z = float(goal['z'])
        qx, qy, qz, qw = rc.quat_from_yaw(float(goal['yaw']))
        msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
        pub = (self._follow_pub if topic == rc.FOLLOW_GOAL_TOPIC
               else self._goal_pub)
        pub.publish(msg)

    def _cached(self, key, fn, ttl=1.0):
        """A graph query, memoised.  What these answer changes when a node
        starts, not between two polls a fifth of a second apart -- and at the
        RC card's 5 Hz an uncached one per poll is real CPU."""
        now = self._now()
        hit = self._graph.get(key)
        if hit is None or now - hit[0] > ttl:
            try:
                hit = (now, fn())
            except Exception:
                hit = (now, 0)
            self._graph[key] = hit
        return hit[1]

    def goal_listeners(self, topic=None):
        """Is anything actually listening for goals?  With no fsm_node running,
        a goal goes nowhere and the panel should say so rather than reporting
        success."""
        topic = topic or rc.GOAL_TOPIC_DEFAULT
        return self._cached('sub:' + topic, lambda: self.count_subscribers(topic))

    def _other_follow_publishers(self):
        """How many nodes OTHER THAN US publish goals on the follow topic.

        By node name, not `count_publishers() - 1`: subtracting our own assumes
        we have exactly one publisher there and that it is always registered,
        and being wrong by one either invents contention or -- the bad
        direction -- misses the DTP follower driving the same topic.
        """
        mine = self.get_name()
        try:
            return sum(1 for i in self.get_publishers_info_by_topic(
                rc.FOLLOW_GOAL_TOPIC) if i.node_name != mine)
        except Exception:
            # Some RMW implementations do not answer this query. Fall back to
            # the arithmetic, which is right whenever our publisher exists.
            return max(0, self.count_publishers(rc.FOLLOW_GOAL_TOPIC) - 1)

    def goal_graph(self):
        """The three counts rc.choose_path() needs to pick a path: who is
        listening on each goal topic, and whether anyone but us is publishing
        on the follow one."""
        return {
            'click_subs': self.goal_listeners(rc.GOAL_TOPIC_DEFAULT),
            'follow_subs': self.goal_listeners(rc.FOLLOW_GOAL_TOPIC),
            'follow_pubs_other': self._cached(
                'pubother:' + rc.FOLLOW_GOAL_TOPIC,
                self._other_follow_publishers),
        }

    def _aircraft(self):
        s, age = self._get('status')
        c, _ = self._get('ctrlmode')
        l, _ = self._get('land')
        if s is None:
            return {'fresh': False, 'mode': None, 'armed': None, 'age': None}
        return {
            'fresh': age is not None and age < STALE_AFTER,
            'age': round(age, 2),
            'mode': NAV.get(s.nav_state, str(s.nav_state)),
            'mode_intended': NAV.get(s.nav_state_user_intention,
                                     str(s.nav_state_user_intention)),
            'arming': ARMING.get(s.arming_state, str(s.arming_state)),
            'armed': s.arming_state == VehicleStatus.ARMING_STATE_ARMED,
            'failsafe': bool(s.failsafe),
            'preflight_ok': bool(s.pre_flight_checks_pass),
            'offboard': bool(c.flag_control_offboard_enabled) if c else None,
            'landed': bool(l.landed) if l else None,
        }

    def _battery(self):
        b, age = self._get('battery')
        if b is None:
            return {'present': False, 'reason': 'no /fmu/out/battery_status_v1 yet'}
        volts = _f(b.voltage_v)
        # voltage_v carries [@invalid 0]; on USB power with no pack the FC also
        # reports connected=False.  Say so rather than printing 0.0 V.
        if not b.connected or not volts:
            return {'present': False, 'fresh': age < STALE_AFTER,
                    'reason': 'no battery connected (USB power?)'}
        remaining = _f(b.remaining)
        return {
            'present': True,
            'fresh': age < STALE_AFTER,
            'age': round(age, 2),
            'voltage': volts,
            'current': _f(b.current_a),          # uncalibrated -- see below
            'remaining': None if remaining is None or remaining < 0 else remaining,
            'discharged_mah': _f(b.discharged_mah),
            'temperature': _f(b.temperature),
            'cells': int(b.cell_count),
            'time_remaining_s': _f(b.time_remaining_s),
            'warning': BATT_WARN.get(b.warning, str(b.warning)),
            # docs/ark-12s-payload-power-module.md: the INA238 shunt is the
            # datasheet value and was never checked against a watt meter.
            'current_uncalibrated': True,
        }

    def _estimator(self):
        p, page = self._get('lpos')
        e, _ = self._get('estflags')
        out = {'fresh': page is not None and page < STALE_AFTER,
               # The 50 Hz subscription is created on demand, so the first
               # poll after the page opens has nothing yet. Say "starting",
               # not "no data" -- the second looks like a fault and is not.
               'warming': not self._hot or page is None}
        if p is not None:
            out.update({
                'xy_valid': bool(p.xy_valid), 'z_valid': bool(p.z_valid),
                'dead_reckoning': bool(p.dead_reckoning),
                'n': _f(p.x), 'e': _f(p.y), 'd': _f(p.z),
                'heading_deg': None if _f(p.heading) is None
                               else round(math.degrees(p.heading), 1),
            })
        if e is not None:
            out.update({
                'cs_ev_pos': bool(e.cs_ev_pos),
                'cs_ev_yaw': bool(e.cs_ev_yaw),
                'cs_ev_hgt': bool(e.cs_ev_hgt),
                'rejects': sorted(k for k in dir(e)
                                  if k.startswith('reject_') and getattr(e, k)),
            })
        return out

    def topics(self, ttl=TOPIC_TTL):
        """Every topic with a live publisher: name, type, publisher count.

        For the bag recorder's picker, and the only place in this file that
        walks the whole graph -- which is why it is on demand and cached, and
        why /api/state never calls it.  One count_publishers() per topic is
        ~100 graph queries on this aircraft (65 /fmu/* alone).

        Filtered by publisher count for the reason camera.py gives: a topic
        listed with zero publishers is another machine's stale discovery cache
        (CLAUDE.md), and offering it for recording produces a bag with an empty
        topic in it -- or, with --include-unpublished-topics, a subscription at
        the wrong QoS.

        A topic with two publishers is reported honestly rather than hidden.
        On this aircraft that is a fault worth seeing before you record: a
        second publisher on /lidar_slam/odom is the exact failure mode
        docs/ImplementationDoc.md warns about for the M3 bridge.
        """
        now = time.monotonic()
        if now - self._topics['t'] < ttl:
            return self._topics['rows']
        rows = []
        try:
            for name, types in self.get_topic_names_and_types():
                n = self.count_publishers(name)
                if n:
                    rows.append({'topic': name, 'type': types[0] if types else '',
                                 'types': types, 'publishers': n})
        except Exception:
            rows = self._topics['rows']
        rows.sort(key=lambda r: r['topic'])
        self._topics.update(t=now, rows=rows)
        return rows

    def _rate_table(self):
        now = self._now()
        out = {}
        for topic, dq in self._rates.items():
            while dq and now - dq[0] > RATE_WINDOW:
                dq.popleft()
            span = (dq[-1] - dq[0]) if len(dq) > 1 else 0.0
            hz = (len(dq) - 1) / span if span > 0.05 else 0.0
            out[topic] = {'hz': round(hz, 1), 'expect': RATE_TOPICS[topic][2]}
        return out


class TelemetryThread:
    """Owns rclpy init/spin so server.py stays transport-agnostic."""

    def __init__(self):
        # NO signal handlers: rclpy.init() otherwise installs its own SIGINT
        # handler, which swallows KeyboardInterrupt and leaves serve_forever()
        # blocked -- Ctrl-C would appear to hang with the stack still running.
        # server.py installs explicit handlers instead, because teardown here
        # owns real process groups and must actually run.
        rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
        self.node = Telemetry()
        self._exec = rclpy.executors.SingleThreadedExecutor()
        self._exec.add_node(self.node)
        # The camera gets its OWN executor and thread. Its frames are ~170 kB
        # at up to 30 Hz while someone is watching the feed, and on a shared
        # single-threaded executor every one of those would sit between two
        # aircraft callbacks. Battery and flight mode must not go stale
        # because a phone opened the video. It costs one idle thread when
        # nobody is looking -- camera.py creates no subscription until then.
        self.camera = camera.CameraFeed()
        self._cam_exec = rclpy.executors.SingleThreadedExecutor()
        self._cam_exec.add_node(self.camera)
        self._thread = threading.Thread(target=self._spin, args=(self._exec,),
                                        daemon=True)
        self._thread.start()
        self._cam_thread = threading.Thread(target=self._spin,
                                            args=(self._cam_exec,), daemon=True)
        self._cam_thread.start()

    @staticmethod
    def _spin(executor):
        try:
            executor.spin()
        except Exception:
            pass

    def snapshot(self):
        return self.node.snapshot()

    def topics(self):
        return self.node.topics()

    def rc_snapshot(self):
        return self.node.rc_snapshot()

    def publish_goal(self, goal, topic=None):
        return self.node.publish_goal(goal, topic)

    def goal_listeners(self, topic=None):
        return self.node.goal_listeners(topic)

    def goal_graph(self):
        return self.node.goal_graph()

    def aircraft_only(self):
        return self.node.aircraft_only()

    def follow(self, on):
        return self.node.follow(on)

    def shutdown(self):
        for ex, node in ((self._cam_exec, self.camera), (self._exec, self.node)):
            try:
                ex.shutdown()
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
