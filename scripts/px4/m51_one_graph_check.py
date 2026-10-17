#!/usr/bin/env python3
"""M5.1 -- "one graph, props off": score a recorded bag against docs/m5-m6-plan.md SS2.2.

WHAT M5.1 IS
------------
The first time the whole chain runs as one graph: FAST-LIO + SUPER (follow_test_ros2.yaml) + the
DTP follower + BOTH PX4 bridges + the GCS, armed in Offboard with the props off, a person walking
in front of the rig. Every piece has passed alone; M5.1 asks whether they still do together, and
settles the "third writer to the FC" question (dtp-follower-stack.md open item 10) by test. The
runbook is docs/m5.1-one-graph-runbook.md. This script is the scorer: it reads the bag and prints
one line per threshold. It commands nothing and needs no FC -- a bag is all it reads.

THE FOUR THRESHOLDS (all must pass)
-----------------------------------
  S  SETPOINTS   >= 99 % of the bridge's TRACK setpoints equal a SUPER command under the M3
                 conversion (x, -y, -z), psi -> -psi. Bench M3: 910/913.
                 TRACK = PX4 says armed+Offboard AND a /planning/pos_cmd arrived < 0.3 s before
                 (the bridge's own cmd_timeout). IDLE samples (not engaged: the bridge latches the
                 live estimate; since 2026-09-28 sent only in disarmed Offboard) and HOLD samples
                 (SUPER silent: frozen point) are counted and shown, never scored -- neither is
                 SUPER's command by design.
  G  GOALSTATUS  >= 99 % of /planning/follow_goal messages answered on /planning/goal_status
                 within 0.2 s, with requested_p within 1 cm of the goal. Matched one-to-one on
                 requested_p, because the topic has several publishers (follow_planner via
                 command_mux, the GCS) and fsm_node answers every one of them.
  A  ABORT       the land-on-blindness abort (/dtp/abort) produces MAV_CMD_NAV_LAND, PX4 acks it
                 ACCEPTED <= 1.0 s after the abort fired (which is itself 2.0 s after the blind
                 condition began), nav_state reaches AUTO_LAND, and BOTH bridges were still
                 publishing across the event -- i.e. the third writer won against the other two.
                 For the offboard bridge that is its heartbeat, /fmu/in/offboard_control_mode:
                 since 2026-09-28 its trajectory_setpoint stops once PX4 leaves Offboard.
  T  TAKEOVER    every exit from OFFBOARD that our NAV_LAND did not cause is the pilot's ch11
                 switch; it WINS if PX4 stays out of OFFBOARD >= 2.0 s while the bridge keeps
                 its heartbeat up (nothing of ours takes the aircraft back). Needs 3/3.
                 Since 2026-09-28 a second line checks that the bridge's trajectory_setpoint
                 STOPS within SP_STOP_AFTER_EXIT_S of each exit; otherwise it is mixed into the
                 pilot's Position mode. A bridge from before 2026-09-28 fails that line. A bag
                 without the heartbeat falls back to trajectory_setpoint for "keeps streaming",
                 the pre-2026-09-28 test.

WHY THE TAKEOVER CANNOT BE LOST TO US, AND WHAT STILL SURPRISES (PX4 d6f12ad, read 2026-09-13)
---------------------------------------------------------------------------------------------
* ManualControl.cpp:221 acts on the Offboard switch only when its position CHANGES. A switch
  left ON after our NAV_LAND does not drag the aircraft back into Offboard; flipping it OFF and
  ON again does. After an ABORT, never re-engage ch11 without relaunching the follower: the mux
  is terminal and still publishes its HOLD goal (+0.75 m climb), which SUPER would fly to.
* NAV_LAND is a forced mode change (Commander.cpp:1093) -- always accepted unless the mode
  intention itself refuses, in which case the ack is TEMPORARILY_REJECTED and the mux retries.
* COM_RC_OVERRIDE = 1 (auto bit only): the sticks do NOTHING in Offboard, but DO take over (to
  Position) during the ABORT's AUTO_LAND (Commander.cpp:2946).

    source ~/super_ws/install/setup.bash
    python3 scripts/px4/m51_one_graph_check.py ~/dtp_datasets/<date>_m51
    python3 scripts/px4/m51_one_graph_check.py BAG --only S,G      # score a subset

Exit 0 = every requested section scored and passed; 1 = a check failed; 3 = nothing failed but a
requested section could not be scored from this bag (NOT a pass); 2 = the bag could not be read.
"""
import argparse
import bisect
import glob
import math
import os
import re
import sqlite3
import sys

# The plan's thresholds (SS2.2 M5.1)
SETPOINT_PASS_FRAC = 0.99
GOAL_STATUS_WINDOW_S = 0.2
GOAL_STATUS_PASS_FRAC = 0.99
REQUESTED_P_TOL_M = 0.01
RECORDER_INVERSION_S = 0.05
ABORT_ACK_MAX_S = 1.0
TAKEOVER_HOLD_S = 2.0
TAKEOVERS_NEEDED = 3

# The bridge's own freshness rule (px4_offboard_bridge.py cmd_timeout) and float32 round-off
CMD_TIMEOUT_S = 0.3
SP_POS_TOL_M = 1e-4
SP_YAW_TOL_RAD = 1e-4
# A setpoint is built from the newest command the BRIDGE had; the recorder can hear the next one
# first. Compare against the last few commands received up to just after the setpoint.
SP_MATCH_CMDS = 4
SP_MATCH_LATE_S = 0.02
# "both bridges publishing across the event": floors on the window around the abort
ABORT_WINDOW_S = (-2.0, 3.0)
ODOM_BRIDGE_FLOOR_HZ = 8.0     # /fmu/in/vehicle_visual_odometry, nominally 10 Hz
OFFBOARD_BRIDGE_FLOOR_HZ = 40.0  # the offboard bridge's heartbeat (was its setpoints), 50 Hz
OFFBOARD_HEARTBEAT = '/fmu/in/offboard_control_mode'
# 2026-09-28: the bridge stops trajectory_setpoint on the first vehicle_control_mode that says
# Offboard is off. That report left PX4 within ~15 ms of the switch in every 09-21/26 bag, plus
# one 20 ms bridge tick; 0.25 s leaves room for the recorder hearing the two topics apart.
SP_STOP_AFTER_EXIT_S = 0.25
NAV_LAND_EXPECT_S = 1.0        # nav_state should read AUTO_LAND this soon after the ack

NAV_OFFBOARD = 14
NAV_AUTO_LAND = 18
CMD_NAV_LAND = 21
ACK_ACCEPTED = 0
NAV_NAMES = {0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'AUTO_MISSION', 4: 'AUTO_LOITER',
             5: 'AUTO_RTL', 14: 'OFFBOARD', 15: 'STAB', 17: 'AUTO_TAKEOFF', 18: 'AUTO_LAND'}


def _rows(con, tid, topic):
    if topic not in tid:
        return []
    return con.execute('SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp',
                       (tid[topic],)).fetchall()


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _pct(v, q):
    if not v:
        return float('nan')
    s = sorted(v)
    k = (len(s) - 1) * q / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


class Verdicts:
    def __init__(self):
        self.lines = []

    def add(self, section, ok, what, detail):
        self.lines.append((section, ok, what, detail))
        print('  %s  [%s] %-52s %s' % ({True: 'PASS', False: 'FAIL', None: 'n/a '}[ok], section, what, detail))

    def failed(self):
        return [l for l in self.lines if l[1] is False]


def mode_timeline(con, tid, deser, control_mode_topic='/fmu/out/vehicle_control_mode'):
    """[(t, armed, offboard)] from vehicle_control_mode, and [(t, nav_state)] from vehicle_status."""
    from px4_msgs.msg import VehicleControlMode, VehicleStatus
    cm = [(ts * 1e-9, m.flag_armed, m.flag_control_offboard_enabled)
          for ts, d in _rows(con, tid, control_mode_topic)
          for m in [deser(bytes(d), VehicleControlMode)]]
    vs = [(ts * 1e-9, int(m.nav_state), int(m.arming_state))
          for ts, d in _rows(con, tid, '/fmu/out/vehicle_status_v1')
          for m in [deser(bytes(d), VehicleStatus)]]
    return cm, vs


def state_at(timeline, t, default=None, _cache={}):
    key = id(timeline)
    if key not in _cache or len(_cache[key]) != len(timeline):
        _cache[key] = [x[0] for x in timeline]
    i = bisect.bisect_right(_cache[key], t) - 1
    return timeline[i] if i >= 0 else default


def score_setpoints(con, tid, deser, cm, v, setpoint_topic='/fmu/in/trajectory_setpoint'):
    from mars_quadrotor_msgs.msg import PositionCommand
    from px4_msgs.msg import TrajectorySetpoint
    cmds = []
    for ts, d in _rows(con, tid, '/planning/pos_cmd'):
        m = deser(bytes(d), PositionCommand)
        cmds.append((ts * 1e-9, (m.position.x, -m.position.y, -m.position.z), _wrap(-m.yaw)))
    sps = [(ts * 1e-9, deser(bytes(d), TrajectorySetpoint)) for ts, d in _rows(con, tid, setpoint_topic)]
    print('\nS  setpoints: %d %s, %d /planning/pos_cmd, %d control-mode reports'
          % (len(sps), setpoint_topic, len(cmds), len(cm)))
    if not sps or not cmds or not cm:
        v.add('S', None, 'setpoint equality', 'needs trajectory_setpoint + pos_cmd + vehicle_control_mode in the bag')
        return
    cmd_t = [c[0] for c in cmds]
    n_idle = n_hold = n_track = n_eq = 0
    worst = 0.0
    bad_examples = []
    for t, sp in sps:
        st = state_at(cm, t)
        if st is None or not (st[1] and st[2]):
            n_idle += 1
            continue
        j = bisect.bisect_right(cmd_t, t + SP_MATCH_LATE_S) - 1
        if j < 0 or t - cmd_t[j] > CMD_TIMEOUT_S + SP_MATCH_LATE_S:
            n_hold += 1
            continue
        n_track += 1
        best = float('inf')
        for k in range(max(0, j - SP_MATCH_CMDS + 1), j + 1):
            _tc, pos, yaw = cmds[k]
            dp = max(abs(float(sp.position[i]) - pos[i]) for i in range(3))
            dy = abs(_wrap(float(sp.yaw) - yaw))
            best = min(best, max(dp, dy))
            if dp <= SP_POS_TOL_M and dy <= SP_YAW_TOL_RAD:
                n_eq += 1
                worst = max(worst, max(dp, dy))
                break
        else:
            if len(bad_examples) < 3:
                bad_examples.append('t+%.2f s off by %.3g' % (t - sps[0][0], best))
    print('   IDLE (not armed+Offboard) %d, HOLD (no pos_cmd for %.1f s) %d, TRACK %d'
          % (n_idle, CMD_TIMEOUT_S, n_hold, n_track))
    if n_track == 0:
        v.add('S', None, 'setpoint equality', 'no TRACK samples: never armed+Offboard with SUPER commanding')
        return
    frac = n_eq / n_track
    v.add('S', frac >= SETPOINT_PASS_FRAC, 'TRACK setpoints == SUPER command (x,-y,-z), -psi',
          '%d/%d = %.2f %% (need %.0f %%), worst matched residual %.2g%s'
          % (n_eq, n_track, 100 * frac, 100 * SETPOINT_PASS_FRAC, worst,
             ('; unmatched e.g. ' + ', '.join(bad_examples)) if bad_examples else ''))


def score_goal_status(con, tid, deser, v):
    from geometry_msgs.msg import PoseStamped
    from dtp_msgs.msg import GoalStatus
    goals = [(ts * 1e-9, (m.pose.position.x, m.pose.position.y, m.pose.position.z))
             for ts, d in _rows(con, tid, '/planning/follow_goal') for m in [deser(bytes(d), PoseStamped)]]
    stats = [(ts * 1e-9, (m.requested_p.x, m.requested_p.y, m.requested_p.z), int(m.status))
             for ts, d in _rows(con, tid, '/planning/goal_status') for m in [deser(bytes(d), GoalStatus)]]
    print('\nG  goal status: %d /planning/follow_goal, %d /planning/goal_status' % (len(goals), len(stats)))
    if not goals:
        v.add('G', None, 'GoalStatus answers', 'no /planning/follow_goal in the bag')
        return
    # One-to-one, in status order: each status answers the EARLIEST unanswered goal it echoes.
    # A status may be RECORDED up to RECORDER_INVERSION_S before its goal -- the recorder hears
    # the two topics on separate subscriptions, and fsm_node answers within a millisecond or two.
    # Measured on 20260909_closedloop_run5: 19 of 495 statuses landed in the bag 1-9 ms BEFORE the
    # goal they answer; a strict "goal first" match scored that run 95.4 % instead of 99.2 %.
    goal_t = [g[0] for g in goals]
    used = [False] * len(goals)
    lat, err = [], []
    answered = inversions = 0
    status_codes = {0: 0, 1: 0, 2: 0}
    for t, req, code in stats:
        lo = bisect.bisect_left(goal_t, t - GOAL_STATUS_WINDOW_S)
        hi = bisect.bisect_right(goal_t, t + RECORDER_INVERSION_S)
        for k in range(lo, hi):
            if used[k]:
                continue
            e = math.dist(req, goals[k][1])
            if e <= REQUESTED_P_TOL_M:
                used[k] = True
                inversions += t < goals[k][0]
                lat.append(max(0.0, t - goals[k][0]))
                err.append(e)
                answered += 1
                status_codes[code] = status_codes.get(code, 0) + 1
                break
    frac = answered / len(goals)
    v.add('G', frac >= GOAL_STATUS_PASS_FRAC, 'goals answered <= %.1f s, requested_p <= %.0f cm'
          % (GOAL_STATUS_WINDOW_S, 100 * REQUESTED_P_TOL_M),
          '%d/%d = %.2f %% (need %.0f %%); latency p50 %.0f ms p95 %.0f ms max %.0f ms; echo err max %.2g m; '
          'ACCEPTED %d SNAPPED %d DROPPED %d; %d recorded before their goal'
          % (answered, len(goals), 100 * frac, 100 * GOAL_STATUS_PASS_FRAC, 1e3 * _pct(lat, 50), 1e3 * _pct(lat, 95),
             1e3 * (max(lat) if lat else float('nan')), max(err) if err else float('nan'),
             status_codes.get(0, 0), status_codes.get(1, 0), status_codes.get(2, 0), inversions))


def score_abort(con, tid, deser, cm, vs, v):
    from std_msgs.msg import String
    from px4_msgs.msg import VehicleCommand, VehicleCommandAck
    aborts = [(ts * 1e-9, deser(bytes(d), String).data) for ts, d in _rows(con, tid, '/dtp/abort')]
    cmds = [(ts * 1e-9, deser(bytes(d), VehicleCommand)) for ts, d in _rows(con, tid, '/fmu/in/vehicle_command')]
    acks = [(ts * 1e-9, deser(bytes(d), VehicleCommandAck)) for ts, d in _rows(con, tid, '/fmu/out/vehicle_command_ack')]
    print('\nA  abort: %d /dtp/abort, %d /fmu/in/vehicle_command, %d acks' % (len(aborts), len(cmds), len(acks)))
    if not aborts:
        v.add('A', None, 'ABORT -> NAV_LAND', 'no /dtp/abort in the bag (was the blind condition induced?)')
        return
    t_abort, reason = aborts[0]
    print('   /dtp/abort at t+%.2f s: %s' % (t_abort - _bag_t0(con), reason[:140]))
    m = re.search(r'persisted ([0-9.]+) s >= ([0-9.]+) s', reason)
    bench = 'bench' in reason.lower() or 'require_offboard' in reason.lower()
    if m:
        v.add('A', float(m.group(1)) >= float(m.group(2)), 'fired only after its persistence',
              'held %s s >= %s s' % (m.group(1), m.group(2)))
    lands = [(t, c) for t, c in cmds if int(c.command) == CMD_NAV_LAND and t >= t_abort - 0.1]
    if not lands:
        v.add('A', False, 'NAV_LAND published', 'no VEHICLE_CMD_NAV_LAND on /fmu/in/vehicle_command after the abort')
        return
    t_send = lands[0][0]
    ok_acks = [(t, a) for t, a in acks if int(a.command) == CMD_NAV_LAND and int(a.result) == ACK_ACCEPTED and t >= t_send]
    any_acks = [(t, int(a.result)) for t, a in acks if int(a.command) == CMD_NAV_LAND and t >= t_send]
    if not ok_acks:
        v.add('A', False, 'PX4 ACCEPTED the NAV_LAND', 'acks for 21: %s' % (any_acks[:5] or 'none'))
        return
    t_ack = ok_acks[0][0]
    v.add('A', t_ack - t_abort <= ABORT_ACK_MAX_S, 'NAV_LAND ACCEPTED <= %.1f s after the abort' % ABORT_ACK_MAX_S,
          'fired -> first send %.0f ms -> ACCEPTED %.0f ms (%d send(s))'
          % (1e3 * (t_send - t_abort), 1e3 * (t_ack - t_abort), len(lands)))
    st_before = state_at(cm, t_abort)
    engaged = bool(st_before and st_before[1] and st_before[2])
    after = [(t, n) for t, n, _a in vs if t_ack <= t <= t_ack + NAV_LAND_EXPECT_S + 1.0]
    landed = [t for t, n in after if n == NAV_AUTO_LAND]
    if engaged:
        v.add('A', bool(landed) and landed[0] - t_ack <= NAV_LAND_EXPECT_S, 'nav_state -> AUTO_LAND',
              ('after %.0f ms' % (1e3 * (landed[0] - t_ack))) if landed else
              'never read AUTO_LAND (states after ack: %s)' % sorted({NAV_NAMES.get(n, n) for _t, n in after}))
    else:
        v.add('A', None, 'nav_state -> AUTO_LAND',
              'aircraft was NOT armed+Offboard when it fired%s -- a bench exercise of the path, not the M5.1 test'
              % (' (abort_require_offboard:=false)' if bench else ''))
    lo, hi = t_abort + ABORT_WINDOW_S[0], t_abort + ABORT_WINDOW_S[1]
    span = hi - lo
    # The offboard bridge's liveness is its heartbeat: since 2026-09-28 its setpoints stop as
    # soon as NAV_LAND takes PX4 out of Offboard, which is correct (AUTO_LAND's task owns them).
    ob_topic = OFFBOARD_HEARTBEAT if OFFBOARD_HEARTBEAT in tid else '/fmu/in/trajectory_setpoint'
    for topic, floor, who in (('/fmu/in/vehicle_visual_odometry', ODOM_BRIDGE_FLOOR_HZ, 'px4_odom_bridge'),
                              (ob_topic, OFFBOARD_BRIDGE_FLOOR_HZ, 'px4_offboard_bridge')):
        if topic not in tid:
            v.add('A', None if not engaged else False, '%s publishing across the abort' % who, '%s not recorded' % topic)
            continue
        n = con.execute('SELECT COUNT(*) FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ?',
                        (tid[topic], int(lo * 1e9), int(hi * 1e9))).fetchone()[0]
        v.add('A', n / span >= floor, '%s publishing across the abort' % who,
              '%s %.1f Hz over [%+.0f, %+.0f] s (floor %.0f Hz)'
              % (topic, n / span, ABORT_WINDOW_S[0], ABORT_WINDOW_S[1], floor))


def score_takeovers(con, tid, deser, vs, v):
    from px4_msgs.msg import VehicleCommand
    lands = [ts * 1e-9 for ts, d in _rows(con, tid, '/fmu/in/vehicle_command')
             if int(deser(bytes(d), VehicleCommand).command) == CMD_NAV_LAND]
    print('\nT  takeovers: %d vehicle_status reports' % len(vs))
    if not vs:
        v.add('T', None, 'RC ch11 takeovers', 'no /fmu/out/vehicle_status_v1 in the bag')
        return
    sp_t = [ts * 1e-9 for ts, _d in _rows(con, tid, '/fmu/in/trajectory_setpoint')]
    # "The bridge is still up" = its heartbeat (2026-09-28); its setpoints now stop on the exit.
    hb_t = [ts * 1e-9 for ts, _d in _rows(con, tid, OFFBOARD_HEARTBEAT)]
    alive_t = hb_t or sp_t
    if not hb_t:
        print('   (no %s in the bag: "streaming" falls back to trajectory_setpoint, the '
              'pre-2026-09-28 test)' % OFFBOARD_HEARTBEAT)
    exits = []
    for i in range(1, len(vs)):
        if vs[i - 1][1] == NAV_OFFBOARD and vs[i][1] != NAV_OFFBOARD:
            exits.append(i)
    wins = takeovers = stopped = 0
    end_t = vs[-1][0]
    for i in exits:
        t = vs[i][0]
        if any(t - 2.0 <= tl <= t for tl in lands) or vs[i][1] == NAV_AUTO_LAND:
            print('   t+%.1f s OFFBOARD -> %s: our NAV_LAND, not a takeover' % (t - _bag_t0(con), NAV_NAMES.get(vs[i][1], vs[i][1])))
            continue
        takeovers += 1
        back = next((vs[k][0] for k in range(i + 1, len(vs)) if vs[k][1] == NAV_OFFBOARD), None)
        out_for = (back if back is not None else end_t) - t
        n_alive = sum(1 for s in alive_t if t <= s <= t + min(out_for, TAKEOVER_HOLD_S))
        streaming = n_alive / max(1e-3, min(out_for, TAKEOVER_HOLD_S)) >= OFFBOARD_BRIDGE_FLOOR_HZ
        won = out_for >= TAKEOVER_HOLD_S and streaming
        wins += won
        # Setpoints after the stop margin, up to the margin before any re-engage (vehicle_status
        # is limited to 5 Hz on the link, so its edges can lag the bridge's own trigger).
        lo, hi = t + SP_STOP_AFTER_EXIT_S, t + min(out_for - SP_STOP_AFTER_EXIT_S, TAKEOVER_HOLD_S)
        n_late = sum(1 for s in sp_t if lo < s <= hi) if hi > lo else 0
        stopped += n_late == 0
        print('   t+%.1f s OFFBOARD -> %s, out for %.1f s%s, bridge %s -> %s; %s'
              % (t - _bag_t0(con), NAV_NAMES.get(vs[i][1], vs[i][1]), out_for,
                 '' if back is not None else ' (to end of bag)', 'streaming' if streaming else 'NOT streaming',
                 'WIN' if won else 'LOST',
                 'setpoints stopped' if n_late == 0 else
                 '%d setpoints after +%.2f s (mixed into %s)' % (n_late, SP_STOP_AFTER_EXIT_S,
                                                                NAV_NAMES.get(vs[i][1], vs[i][1]))))
    if takeovers == 0:
        v.add('T', None, 'RC ch11 takeovers', 'no pilot exit from OFFBOARD recorded')
        return
    v.add('T', takeovers >= TAKEOVERS_NEEDED and wins == takeovers,
          'pilot takeovers won (need %d/%d)' % (TAKEOVERS_NEEDED, TAKEOVERS_NEEDED),
          '%d/%d won' % (wins, takeovers))
    v.add('T', stopped == takeovers,
          'bridge setpoints stop <= %.2f s after each takeover' % SP_STOP_AFTER_EXIT_S,
          '%d/%d stopped%s' % (stopped, takeovers, '' if stopped == takeovers else
                               ' -- a bridge from before 2026-09-28, or setpoint_only_in_offboard:=false'))


_T0 = {}


def _bag_t0(con):
    if con not in _T0:
        _T0[con] = (con.execute('SELECT MIN(timestamp) FROM messages').fetchone()[0] or 0) * 1e-9
    return _T0[con]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('bag')
    ap.add_argument('--only', default='S,G,A,T', help='comma list of sections to score')
    ap.add_argument('--setpoint-topic', default='/fmu/in/trajectory_setpoint',
                    help='score a SANDBOXED bridge instead (bench: a second px4_offboard_bridge fed a '
                         'fake armed+Offboard, so S can be exercised with the real FC disarmed)')
    ap.add_argument('--control-mode-topic', default='/fmu/out/vehicle_control_mode',
                    help='the control-mode stream that bridge was fed')
    args = ap.parse_args(argv)
    sections = set(args.only.upper().split(','))
    if args.control_mode_topic != '/fmu/out/vehicle_control_mode' and sections != {'S'}:
        ap.error('--control-mode-topic is a sandbox override for section S only: A and T must read the '
                 'real PX4 control mode. Use --only S with it.')

    try:
        from rclpy.serialization import deserialize_message as deser
        import px4_msgs.msg  # noqa: F401
        import dtp_msgs.msg  # noqa: F401
        import mars_quadrotor_msgs.msg  # noqa: F401
    except ImportError as exc:
        print('cannot import message types (%s) -- source ~/super_ws/install/setup.bash' % exc)
        return 2
    files = sorted(glob.glob(os.path.join(args.bag, '*.db3')))
    if not files:
        print('no .db3 in %s' % args.bag)
        return 2
    con = sqlite3.connect(files[0])
    tid = {r[1]: r[0] for r in con.execute('SELECT id,name FROM topics').fetchall()}
    dur = (con.execute('SELECT MAX(timestamp) FROM messages').fetchone()[0] or 0) * 1e-9 - _bag_t0(con)
    print('=' * 100)
    print('M5.1 ONE-GRAPH CHECK   %s   (%.0f s)' % (os.path.basename(args.bag.rstrip(os.sep)), dur))
    print('=' * 100)
    v = Verdicts()
    cm, vs = mode_timeline(con, tid, deser, args.control_mode_topic)
    armed_off = sum(1 for _t, a, o in cm if a and o)
    print('control mode: %d reports, armed+Offboard in %d' % (len(cm), armed_off))
    if 'S' in sections:
        if args.setpoint_topic != '/fmu/in/trajectory_setpoint':
            print('\n*** S is scored on SANDBOX topics (%s, %s): bench evidence for the conversion under '
                  'full-graph load, NOT the armed M5.1 gate ***' % (args.setpoint_topic, args.control_mode_topic))
        score_setpoints(con, tid, deser, cm, v, args.setpoint_topic)
    if 'G' in sections:
        score_goal_status(con, tid, deser, v)
    if 'A' in sections:
        score_abort(con, tid, deser, cm, vs, v)
    if 'T' in sections:
        score_takeovers(con, tid, deser, vs, v)

    print()
    scored = [l for l in v.lines if l[1] is not None]
    unscored = sorted(sec for sec in sections if not any(l[0] == sec and l[1] is not None for l in v.lines))
    if v.failed():
        print('M5.1: FAIL -- %d of %d scored checks failed%s' % (len(v.failed()), len(scored),
              ('; not scorable from this bag: %s' % ', '.join(unscored)) if unscored else ''))
        return 1
    if unscored:
        # Not a pass. The gate is all four sections on one aircraft session; a bag that could only
        # score some of them must not exit 0 and read as green in a script.
        print('M5.1: INCOMPLETE -- %d scored check(s) pass, but section(s) %s could not be scored from '
              'this bag' % (len(scored), ', '.join(unscored)))
        return 3
    print('M5.1: PASS -- %d/%d checks across section(s) %s' % (len(scored), len(scored), ', '.join(sorted(sections))))
    return 0


if __name__ == '__main__':
    sys.exit(main())
