#!/usr/bin/env bash
# field.sh -- the Stage A field sequence in four words. Run ON THE NX.
#
#   bash scripts/field.sh prep     # after power-on: clocks, wifi, wait for Kevin's stack, launch ours, verify
#   bash scripts/field.sh arm      # walker at 4-6 m and moving: recorder, engage, LOCKED, pre-arm gate
#   bash scripts/field.sh watch    # during the flight: one line per second, tells you when to re-engage
#   bash scripts/field.sh reengage # after a LOST, walker back in the band: re-designate
#   bash scripts/field.sh land     # after landing + disarm: stop recorder, stop ours, show the bag
#   bash scripts/field.sh status   # where things stand, any time
#   bash scripts/field.sh check    # 10 s: is FAST-LIO stalled? anyone drawing its cloud over wifi?
#
# Every step prints what it expected and what it got, and STOPS on the first failure with the fix.
# It never arms, never commands motors, never restarts Kevin's stack. Those stay human.
#
# WHAT EACH STEP IS PROTECTING AGAINST -- all of it happened on 2026-09-12:
#   prep  clocks + wifi reset on every reboot (two "reboots" were wifi power-save under load);
#         FAST-LIO settles for 1-3 min and ours launched on 6.7 Hz went NO-GO; FAST-LIO started
#         with the aircraft in one place and moved to another ran off to 3.7 km; ours launched
#         BEFORE his restarted wedged its TF listener on 100 % of frames; a stale recorder kept
#         writing; the ros2 daemon died with the reboot; sensors_ok flickered in the last bag.
#   arm   the recorder died silently from a cold shell; the walker locked at 9.5 m and lost it
#         in 5 s; auto-reacquire is OFF so a LOST needs a human.
#   land  stopping the recorder by anything but SIGINT-by-PID has killed two recordings and one
#         SSH session; /tmp is cleared on reboot.
# AND ON 2026-09-13 (the first Stage A flights):
#   prep  FAST-LIO's odom kept coming at 10 Hz while each STAMP was up to 38 s old and growing:
#         a laptop RViz subscribed to /cloud_registered over wifi (-72 dBm) and blocked the
#         publisher. Every camera frame failed the world lift, PX4 was fed stale poses, and the
#         rate check passed. Same again after a Start at boot clocks (backlog). So: a stamp-lag
#         gate and a foreign-subscriber gate, in prep AND again in arm (it crept back on the ground).
#   watch ran `ros2 topic echo --once` four times a loop: one line per ~10 s, mostly '?', and it
#         called the landing disarm a 'drop'. Now one subscriber, one line per second.
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STATE=/home/ideas-ad/dtp_datasets/.field_state     # NOT /tmp: it must survive a reboot
NX_IP="${NX_IP:-192.168.8.130}"

say()  { printf '\n=== %s ===\n' "$1"; }
ok()   { printf '  ok   %s\n' "$1"; }
die()  { printf '  FAIL %s\n' "$1"; shift; for l in "$@"; do printf '       %s\n' "$l"; done; exit 1; }
note() { printf '  %s\n' "$1"; }

env_up() {
    set +u
    # shellcheck disable=SC1091
    source "$REPO/ros2_ws/dtp_env_jetson.sh" >/dev/null 2>&1
    set -u
    [ -n "${ROS_DISTRO:-}" ] || die "ROS env did not source" "is $REPO/ros2_ws/dtp_env_jetson.sh present?"
}

odom_rate() { timeout 10 ros2 topic hz /lidar_slam/odom --window 20 2>&1 | grep -m1 -oE "average rate: [0-9.]+" | grep -oE "[0-9.]+"; }
odom_pos()  { timeout 4 ros2 topic echo --once /lidar_slam/odom 2>/dev/null | python3 -c '
import sys,re
t=sys.stdin.read(); p=re.search(r"position:\s*\n\s*x:\s*([-\d.e+]+)\s*\n\s*y:\s*([-\d.e+]+)\s*\n\s*z:\s*([-\d.e+]+)",t)
print("%.3f %.3f %.3f" % tuple(float(v) for v in p.groups()) if p else "")'; }
lock_state() { timeout 4 ros2 topic echo --once --field data /dtp/lock_state 2>/dev/null | head -1; }
our_pgid()  { P=$(pgrep -f "lib/dtp_perception/perception_node" | head -1); [ -n "$P" ] && ps -o pgid= -p "$P" | tr -d ' '; }
rec_pids()  { ps -eo pid,args | grep "[r]os2 bag record" | grep -v "bash -c" | awk '{print $1}'; }
# FAST-LIO's STAMPS, not its rate: prints "p50 max" seconds of (now - header.stamp) over ~4 s of
# /lidar_slam/odom, or nothing if fewer than 10 messages arrived. See the 2026-09-13 note above.
odom_lag() { timeout 12 python3 - <<'PYEOF' 2>/dev/null
import rclpy, time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
rclpy.init(); n = Node('field_odom_lag'); v = []
def cb(m):
    v.append(n.get_clock().now().nanoseconds * 1e-9 - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
n.create_subscription(Odometry, '/lidar_slam/odom', cb, qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 4.0:
    rclpy.spin_once(n, timeout_sec=0.1)
if len(v) >= 10:
    v.sort(); print('%.3f %.3f' % (v[len(v) // 2], v[-1]))
PYEOF
}
# Subscribers to a topic other than the ones we expect ($2: node names, regex). On a FAST-LIO
# output that is a laptop over wifi, and that is what stalled FAST-LIO on 2026-09-13.
foreign_subs() { timeout 8 ros2 topic info -v "$1" 2>/dev/null | awk '/Subscription count/{s=1} s && /Node name:/{print $3}' | grep -vxE "$2" | tr '\n' ' ' | sed 's/ *$//'; }
# The two FAST-LIO gates. $1 = how many 15 s waits to allow for a start-up backlog to drain.
fastlio_gates() {
    F=$(foreign_subs /cloud_registered 'fsm_node'); F2=$(foreign_subs /livox/lidar 'laser_mapping')
    [ -z "$F$F2" ] && ok "nobody but fsm_node/laser_mapping on /cloud_registered and /livox/lidar" \
        || die "foreign subscriber(s) on FAST-LIO: ${F:+/cloud_registered <- $F  }${F2:+/livox/lidar <- $F2}" \
               "That is a laptop RViz over wifi. It stalls FAST-LIO (38 s of stamp lag on 2026-09-13) and" \
               "starves PX4 of poses. Close RViz on the laptop, wait 20 s, re-run. Never draw the cloud" \
               "over wifi while FAST-LIO is flying."
    LAG=""; TRIES="${1:-0}"
    for i in $(seq 0 "$TRIES"); do
        LAG=$(odom_lag); [ -n "$LAG" ] || die "could not measure odom stamp lag" "is /lidar_slam/odom publishing?"
        if awk "BEGIN{exit !(${LAG#* } < 0.30)}"; then break; fi
        if [ "$i" -lt "$TRIES" ]; then
            printf '  ... stamp lag p50 %s s, max %s s -- FAST-LIO catching up, waiting (%d/%d)\n' "${LAG% *}" "${LAG#* }" "$((i+1))" "$TRIES"; sleep 15
        fi
    done
    if awk "BEGIN{exit !(${LAG#* } < 0.30)}"; then ok "odom stamp lag p50 ${LAG% *} s, max ${LAG#* } s"
    else die "odom stamp lag p50 ${LAG% *} s, max ${LAG#* } s -- FAST-LIO is BEHIND (its rate can still read 10 Hz)" \
             "Every camera frame will fail the world lift and PX4 gets stale poses. Seen causes: a laptop RViz" \
             "on the cloud (close it, wait 20 s), or a Start at boot clocks (Kevin: Stop -> Start now that" \
             "clocks are pinned; aircraft untouched). Re-run when it reads < 0.3 s."; fi
}

# ---------------------------------------------------------------------------------------
cmd_prep() {
    say "1. reboot casualties"
    sudo jetson_clocks && sudo iw dev wlP1p1s0 set power_save off || die "sudo failed" "run the two sudo commands by hand, then re-run prep"
    G=$(( $(cat /sys/class/devfreq/17000000.gpu/cur_freq) / 1000000 ))
    [ "$G" -ge 900 ] && ok "GPU $G MHz" || die "GPU $G MHz after jetson_clocks" "expected 918"
    ok "wifi power_save $(iw dev wlP1p1s0 get power_save | awk '{print $NF}')"
    pgrep -x fastlio_mapping >/dev/null && note "FAST-LIO was already running: if it started at boot clocks it may be behind -- step 2 checks its stamps"
    env_up
    ros2 daemon stop >/dev/null 2>&1; sleep 1; ros2 daemon start >/dev/null 2>&1; sleep 2
    ok "ros2 daemon restarted (it dies with the reboot; the symptom is an xmlrpc traceback)"

    say "2. Kevin's stack -- and the aircraft does NOT move from here"
    pgrep -x fastlio_mapping >/dev/null || die "FAST-LIO is not running" \
        "Kevin: dashboard Start, with the LiDAR powered >= 90 s and the aircraft AT ITS TAKEOFF SPOT."
    note "FAST-LIO up since $(ps -o lstart= -C fastlio_mapping | head -1)"
    note "waiting for odom >= 9.5 Hz (settles in 1-3 min; do not launch ours before this)"
    R=""
    for i in $(seq 1 12); do
        R=$(odom_rate); R=${R:-0}
        if awk "BEGIN{exit !($R >= 9.5)}"; then break; fi
        printf '  ... %4.1f Hz, waiting (%d/12)\n' "$R" "$i"; sleep 15
    done
    awk "BEGIN{exit !($R >= 9.5)}" || die "odom $R Hz after 3 min -- that start is bad" \
        "Kevin: dashboard Stop -> wait 10 s -> Start. Do not move the aircraft. Then re-run prep."
    ok "odom $R Hz"
    P=$(odom_pos); [ -n "$P" ] || die "no odom position readable"
    set -- $P
    if awk "BEGIN{exit !(($1*$1+$2*$2) < 1.0)}"; then ok "position ($1, $2, $3) -- at the origin"
    else die "position ($1, $2, $3) -- FAST-LIO is NOT at the origin" \
        "It started somewhere else, or the aircraft was moved after it started (that is how it" \
        "reached 3.7 km today). Kevin: Stop -> place the aircraft -> Start. Then re-run prep."; fi
    fastlio_gates 4

    say "3. clear anything stale of ours"
    for p in $(rec_pids); do kill -INT "$p" 2>/dev/null && note "stopped old recorder $p"; done
    G=$(our_pgid)
    if [ -n "$G" ]; then kill -INT -- "-$G" 2>/dev/null; note "stopped old stack pgid $G"; sleep 6; fi
    [ "$(ps -eo comm | grep -cE '^(perception_node|command_mux_nod)$')" -eq 0 ] && ok "nothing of ours running" \
        || die "our nodes still present" "kill them by pgid (never pkill -f) and re-run prep"

    say "4. launch ours (after his, always)"
    ( cd "$REPO" && bash scripts/indoor_test.sh --keep --no-walk ) | grep -E "^  (ok|FAIL)|GO with|NO-GO|ALL CHECKS|CHECK\(S\) FAILED|pgid" | grep -vE "^  ok" || true
    G=$(our_pgid); [ -n "$G" ] || die "our stack is not up after indoor_test" "read its output above"
    printf 'OUR_PGID=%s\n' "$G" > "$STATE"
    ok "our stack up, pgid $G"

    say "5. the two checks the last bag showed we needed"
    LP=$(grep -h "launch point latched" "$(ls -t /tmp/dtp_indoor_*.log | head -1)" 2>/dev/null | tail -1 | grep -oE "\(-?[0-9.]+, -?[0-9.]+\)" | head -1)
    [ -n "$LP" ] || die "target_manager never latched a launch point" "is odom reaching it?"
    LX=$(echo "$LP" | tr -d '(),' | awk '{print $1}'); LY=$(echo "$LP" | tr -d '(),' | awk '{print $2}')
    if awk "BEGIN{exit !(($LX*$LX+$LY*$LY) < 1.0)}"; then ok "launch point $LP -- the 3 m ignore radius protects the real takeoff spot"
    else die "launch point $LP is NOT at the origin" "ours started against a stale FAST-LIO. Re-run prep."; fi
    S=$(timeout 10 ros2 topic echo --field data /dtp/sensors_ok 2>/dev/null | sort | uniq -c | tr -s ' ' | tr '\n' ' ')
    case "$S" in *False*) die "sensors_ok flickered in 10 s: $S" "the watchdog is unhappy with a sensor; ros2 topic echo --once /dtp/sensor_health" ;;
                 *True*)  ok "sensors_ok steady True for 10 s" ;;
                 *)       die "no sensors_ok in 10 s" "is sensor_watchdog up?" ;; esac

    say "PREP DONE"
    note "Now: walker to 4-6 m from the aircraft (tape measure), walking a slow left-right line."
    note "Nobody else in the band. Kevin behind the camera. Then:  bash scripts/field.sh arm"
}

# ---------------------------------------------------------------------------------------
cmd_arm() {
    env_up
    [ -n "$(our_pgid)" ] || die "our stack is not running" "bash scripts/field.sh prep"
    say "1. recorder"
    for p in $(rec_pids); do kill -INT "$p" 2>/dev/null; note "stopped a leftover recorder ($p)"; sleep 2; done
    OUT=$( (cd "$REPO" && bash scripts/stage_a_record.sh) 2>&1 ); echo "$OUT" | sed 's/^/  /'
    PID=$(echo "$OUT" | grep -oE "recorder pid [0-9]+" | grep -oE "[0-9]+"); BAG=$(echo "$OUT" | grep -oE "recording -> \S+" | cut -d' ' -f3)
    [ -n "$PID" ] && kill -0 "$PID" 2>/dev/null || die "recorder did not start" "read its output above"
    { grep -v '^RECORDER_PID=\|^BAG=' "$STATE" 2>/dev/null; echo "RECORDER_PID=$PID"; echo "BAG=$BAG"; } > "$STATE.new" && mv "$STATE.new" "$STATE"
    ok "recording, pid $PID -> $BAG"

    say "2. engage FOLLOW -- walker must be MOVING at 4-6 m"
    ros2 param set /target_manager follow_enabled false >/dev/null 2>&1; sleep 1
    ros2 param set /target_manager follow_enabled true  >/dev/null 2>&1 || die "could not set follow_enabled" "is target_manager up?"
    L=""
    for i in $(seq 1 8); do
        sleep 3; L=$(lock_state)
        case "$L" in LOCKED*) break ;; esac
        printf '  ... %s\n' "${L:-no lock_state}"
    done
    case "$L" in
        LOCKED*)
            RNG=$(echo "$L" | grep -oE "at [0-9.]+ m" | grep -oE "[0-9.]+")
            ok "$L"
            if [ -n "$RNG" ] && awk "BEGIN{exit !($RNG > 7.0)}"; then
                note "*** locked at $RNG m -- that is far. The last lock at 9.5 m lasted 5 s. Bring the walker to 4-6 m and re-run arm. ***"
            fi ;;
        *) die "not LOCKED after 25 s: ${L:-nothing}" \
             "ACQUIRING = engaged but nobody qualified: walker not moving 0.4 m in 6 s, outside 2.5-10 m," \
             "inside 3 m of the aircraft, or two people in the band. Fix the scene, re-run arm." ;;
    esac

    say "3. THE PRE-ARM GATE"
    fastlio_gates 0
    ( cd "$REPO" && python3 scripts/stage_a_preflight.py --require-record ) 2>&1 | grep -E "^\s*\[(FAIL|WARN)\]|^(GO|NO-GO)" | sed 's/^/  /'
    if ( cd "$REPO" && python3 scripts/stage_a_preflight.py --require-record ) >/dev/null 2>&1; then
        say "GO -- Kevin may arm"
        note "After arming, BEFORE takeoff:  bash scripts/field.sh watch   (it confirms it stays armed)"
    else
        die "NO-GO. Nobody arms." "Every FAIL line above names its fix. Fix it, re-run arm."
    fi
}

# ---------------------------------------------------------------------------------------
cmd_watch() {
    env_up
    note "one line per second: lock | mux | alt | odom stamp lag | target range | armed.   Ctrl-C to stop."
    AR=$(timeout 5 ros2 param get /target_manager auto_reacquire_after_lost 2>/dev/null | grep -oE 'True|False')
    case "$AR" in True) note "auto_reacquire is ON (Stage A only): a LOST re-acquires by itself" ;;
                *)    note "auto_reacquire is OFF: LOST -> walker to 4-6 m, then: field.sh reengage" ;; esac
    python3 - <<'PYEOF'
import rclpy, time
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from dtp_msgs.msg import TargetArray
try:
    from px4_msgs.msg import VehicleStatus
except ImportError:
    VehicleStatus = None
rclpy.init(); n = Node('field_watch')
s = {'lock': '?', 'mux': '?', 'z': None, 'lag': [], 'tgt': None, 'arm': None, 'armed_since': None, 'max_alt': 0.0}
def now(): return n.get_clock().now().nanoseconds * 1e-9
n.create_subscription(String, '/dtp/lock_state', lambda m: s.__setitem__('lock', m.data), 10)
n.create_subscription(String, '/mux/state', lambda m: s.__setitem__('mux', m.data), 10)
def od(m):
    s['z'] = m.pose.pose.position.z
    s['lag'].append(now() - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
n.create_subscription(Odometry, '/lidar_slam/odom', od, qos_profile_sensor_data)
def tg(m):
    s['tgt'] = ('%.1f m' % m.targets[0].distance_m) if m.targets else None
n.create_subscription(TargetArray, '/dtp/targets', tg, qos_profile_sensor_data)
if VehicleStatus is not None:
    n.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                          lambda m: s.__setitem__('arm', int(m.arming_state)), qos_profile_sensor_data)
last = None
try:
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.05)
        t = int(time.time())
        if t == last:
            continue
        last = t
        a = s['arm']
        if a == 2:                                   # PX4 ARMING_STATE_ARMED
            if s['armed_since'] is None:
                s['armed_since'] = t; s['max_alt'] = 0.0
            if s['z'] is not None:
                s['max_alt'] = max(s['max_alt'], s['z'])
            armed = 'ARMED %ds' % (t - s['armed_since'])
        elif a == 1:                                 # ARMING_STATE_DISARMED
            # A drop is a disarm within 8 s of arming WITHOUT ever leaving the ground. A landing
            # disarm (2026-09-13 19:27, called a drop by the old loop) is neither.
            if s['armed_since'] is not None and t - s['armed_since'] < 8 and s['max_alt'] < 0.3:
                armed = 'disarmed -- DROPPED %ds after arming, never left the ground: PX4 refused? ask Kevin' % (t - s['armed_since'])
            else:
                armed = 'disarmed'
            s['armed_since'] = None
        else:
            armed = 'arm ?' if VehicleStatus is not None else 'arm n/a (px4_msgs not on this PYTHONPATH)'
        lag = s['lag']; s['lag'] = []
        print('%s  %-22s %-16s alt %5s  lag %5s  tgt %-6s  %s' % (
            time.strftime('%H:%M:%S'),
            '|'.join(s['lock'].split('|')[:2])[:22],
            '|'.join(s['mux'].split('|')[:2])[:16],
            ('%.2f' % s['z']) if s['z'] is not None else '?',
            ('%.2f' % max(lag)) if lag else '?',
            s['tgt'] or 'none', armed), flush=True)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
PYEOF
}

# ---------------------------------------------------------------------------------------
cmd_reengage() {
    env_up
    note "walker must be back inside 4-6 m and MOVING"
    ros2 param set /target_manager follow_enabled false >/dev/null 2>&1; sleep 1
    ros2 param set /target_manager follow_enabled true  >/dev/null 2>&1; sleep 8
    L=$(lock_state); case "$L" in LOCKED*) ok "$L" ;; *) note "still: ${L:-?}  -- walker in band and moving? run reengage again" ;; esac
}

# ---------------------------------------------------------------------------------------
cmd_land() {
    env_up
    say "1. recorder -- SIGINT by PID so the bag gets its metadata"
    PID=$(grep -oE '^RECORDER_PID=[0-9]+' "$STATE" 2>/dev/null | cut -d= -f2); BAG=$(grep -oE '^BAG=.*' "$STATE" 2>/dev/null | cut -d= -f2)
    [ -z "$PID" ] && PID=$(rec_pids | head -1)
    if [ -n "$PID" ] && kill -0 "$PID" 2>/dev/null; then
        kill -INT "$PID"; for _ in $(seq 1 15); do kill -0 "$PID" 2>/dev/null || break; sleep 1; done
        kill -0 "$PID" 2>/dev/null && die "recorder $PID did not exit in 15 s" "kill -TERM $PID, then check metadata.yaml exists in $BAG"
        ok "recorder $PID stopped"
    else note "no recorder running"; fi
    [ -z "$BAG" ] && BAG=$(ls -dt /home/ideas-ad/dtp_datasets/*stageA 2>/dev/null | head -1)
    if [ -n "$BAG" ] && [ -f "$BAG/metadata.yaml" ]; then
        ok "bag $BAG"; timeout 30 ros2 bag info "$BAG" 2>/dev/null | grep -E "Duration|Messages:" | sed 's/^\s*/       /'
    else die "no playable bag at ${BAG:-?}" "metadata.yaml missing -- was the recorder stopped with something other than SIGINT?"; fi

    say "2. our stack"
    G=$(our_pgid)
    if [ -n "$G" ]; then kill -INT -- "-$G"; sleep 5; ok "stopped pgid $G"; else note "not running"; fi
    ros2 param set /target_manager follow_enabled false >/dev/null 2>&1

    say "LANDED. Copy the bag off the aircraft NOW, from the laptop:"
    note "scp -r ideas-ad@$NX_IP:$BAG ./"
    note "Kevin's stack is still up under the dashboard -- leave it unless you are done for the day."
}

# ---------------------------------------------------------------------------------------
cmd_status() {
    env_up
    say "status $(date +%H:%M:%S)"
    note "clocks $(( $(cat /sys/class/devfreq/17000000.gpu/cur_freq) / 1000000 )) MHz   wifi power_save $(iw dev wlP1p1s0 get power_save | awk '{print $NF}')"
    note "FAST-LIO: $(pgrep -x fastlio_mapping >/dev/null && echo "up since $(ps -o lstart= -C fastlio_mapping | head -1)" || echo DOWN)   odom $(odom_rate || echo 0) Hz   pos ($(odom_pos))"
    note "ours: pgid $(our_pgid || echo none)   recorder: $(rec_pids | tr '\n' ' ' | sed 's/ *$//' || true)${REPLY:-}"
    L=$(odom_lag); note "odom stamp lag: ${L:+p50 ${L% *} s, max ${L#* } s}${L:-unreadable}   foreign cloud subscribers: $(foreign_subs /cloud_registered fsm_node)"
    note "lock: $(lock_state | cut -c1-70)"
    note "mux : $(timeout 3 ros2 topic echo --once --field data /mux/state 2>/dev/null | head -1 | cut -c1-70)"
    note "armed: $(timeout 3 ros2 topic echo --once --field arming_state /fmu/out/vehicle_status_v1 2>/dev/null | head -1)  (1 no, 2 yes)"
    [ -f "$STATE" ] && note "state file: $(tr '\n' ' ' < "$STATE")"
}

cmd_check() { env_up; say "FAST-LIO gates $(date +%H:%M:%S)"; fastlio_gates 0; }

case "${1:-}" in
    prep) cmd_prep ;; arm) cmd_arm ;; watch) cmd_watch ;; reengage) cmd_reengage ;; land) cmd_land ;; status) cmd_status ;; check) cmd_check ;;
    *) awk '/^set -u/{exit} NR>1' "$0"; exit 2 ;;
esac
