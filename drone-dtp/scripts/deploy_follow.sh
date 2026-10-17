#!/usr/bin/env bash
# deploy_follow.sh -- the human-following stack, brought up lean and fast, and LEFT RUNNING.
#
#   bash drone-dtp/scripts/deploy_follow.sh            # full bring-up -> recorder running -> ready to fly
#   bash drone-dtp/scripts/deploy_follow.sh --observe   # same, but the follower commands nothing
#   bash drone-dtp/scripts/deploy_follow.sh --no-record # skip the recorder (bench / dry runs only)
#   bash drone-dtp/scripts/deploy_follow.sh --tracker=mpc  # px4_mpc_tracker.py INSTEAD of
#                                                        # px4_offboard_bridge.py (NOT flown;
#                                                        # docs/mpc-tracker-integration.md)
#   bash drone-dtp/scripts/deploy_follow.sh --observe --goal-mode=nmpc   # follow_planner's solver arm,
#                                                        # shadowed. With authority the pre-arm gate
#                                                        # says NO-GO until --nmpc-cleared is passed.
#   bash drone-dtp/scripts/deploy_follow.sh engage      # walker at 4-6 m and MOVING: engage, wait for LOCKED
#   bash drone-dtp/scripts/deploy_follow.sh stop        # tear down, recorder first so the bag stays playable
#   bash drone-dtp/scripts/deploy_follow.sh status      # what is up, right now
#
# WHY THIS EXISTS, next to field.sh and bench_bringup.sh:
#   field.sh prep      launches the follower through indoor_test.sh: OBSERVE-ONLY, unpinned, and
#                      it records with the stage_a preset. Right for Sortie 1, wrong for a
#                      person-following deployment.
#   bench_bringup.sh   tears the whole stack down again on exit (trap stop_all EXIT). It is a
#                      pre-check, not a bring-up.
#   this file          live follower, stage_b preset, lean node set, and it leaves everything up.
#
# WHAT IT DOES NOT START, and why each one is safe to drop (2026-09-15):
#   dtp_viz             `viz:=false`. Publishes /dtp/viz MarkerArray for a laptop RViz. The launch
#                       file's own docstring: "passive: it publishes /dtp/viz and nothing in the
#                       flight path subscribes". Nobody runs RViz at the field -- the field rule
#                       forbids subscribing the laptop to the cloud at all.
#   dtp_image_overlay   same switch. Draws YOLO boxes on the camera picture. Costs ~17 % of a core
#                       while a laptop watches, nothing when not -- but it is one more process
#                       holding a camera subscription in the flight path's namespace.
#   fusion_node         nothing to do -- it already costs nothing. Measured 2026-09-15: it starts,
#                       logs "DISABLED (enable:=false, the default) -- no depth subscription, no
#                       cloud", and EXITS ("process has finished cleanly"). Its own docstring:
#                       "an OFF-BY-DEFAULT debug view ... THIS NODE IS NOT IN THE PERCEPTION PATH".
#                       There is a `fusion:=false` arg, but only on follower.launch.py --
#                       dtp_system.launch.py neither declares nor forwards it, and that file
#                       carries a documented history of a silent forwarding bug that stopped the
#                       sensor_watchdog from ever starting. Not worth touching for a node that
#                       already exits on its own.
#   offboard_viz        never started here. It republishes the Offboard setpoint as markers so a
#                       laptop RViz without px4_msgs can draw it. Flight-test P5 only.
#   RViz                `rviz:=false` is already the default on both launch files.
#
# WHAT IT STILL RUNS, AND WILL NOT SKIP: the FAST-LIO gates and the Stage B pre-arm preflight.
# Those are not "unnecessary stuff" -- the preflight's 60 s estimator window (G0.2) is what stands
# between you and arming on a diverged FAST-LIO, which happened on 2026-09-12 at 3.74 km of error.
# It is also the floor on how fast this can be: ~60 s of the total is that window, deliberately.
#
# WHERE THE TIME WENT BEFORE: the bridges and the follower were started one after another. They
# share no state -- the bridges talk to PX4, the follower talks to the camera -- so here they start
# together and their warm-ups overlap. The recorder goes up BEFORE the preflight (field.sh arm's
# order), so the preflight can assert it with --require-record instead of warning that it is absent,
# and the preflight window lands inside the bag.
#
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
STATE="$HOME/dtp_datasets/.deploy_state"
FOLLOW_CONFIG=follow_test_ros2.yaml
LEAN_ARGS=(viz:=false)

say()  { printf '\n== %s\n' "$*"; }
ok()   { printf '   ok   %s\n' "$*"; }
note() { printf '        %s\n' "$*"; }
die()  { printf '   FAIL %s\n' "$1"; shift; for l in "$@"; do printf '        %s\n' "$l"; done; exit 1; }

env_up() {
    set +u
    source /opt/ros/humble/setup.bash
    source "$HOME/ws_livox/install/setup.bash"
    source "$HOME/super_ws/install/setup.bash"
    set -u
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
}

# Start `cmd...` in its own session, so pgid == pid and one kill takes the whole tree. Never match
# a pattern against a flattened command line to find it again -- that is how `pkill -f` kills the
# shell that ran it (CLAUDE.md, and it happened again on 2026-09-15).
start_group() {
    local name=$1 log=$2; shift 2
    setsid "$@" > "$log" 2>&1 < /dev/null &
    local pid=$!
    printf '%s_PGID=%s\n' "$name" "$pid" >> "$STATE"
    printf '   started %-10s pgid %-7s -> %s\n' "$name" "$pid" "$log"
}

pgid_of() { grep -m1 "^$1_PGID=" "$STATE" 2>/dev/null | cut -d= -f2; }

stop_group() {
    local name=$1 pgid; pgid=$(pgid_of "$name")
    [ -n "$pgid" ] || { note "$name: no pgid recorded"; return; }
    kill -0 -- "-$pgid" 2>/dev/null || { note "$name (pgid $pgid): already gone"; return; }
    printf '   SIGINT %-10s pgid %s' "$name" "$pgid"
    kill -INT -- "-$pgid" 2>/dev/null
    for _ in $(seq 1 40); do kill -0 -- "-$pgid" 2>/dev/null || break; sleep 0.5; done
    if kill -0 -- "-$pgid" 2>/dev/null; then printf ' ... SIGTERM'; kill -TERM -- "-$pgid" 2>/dev/null; sleep 3; fi
    if kill -0 -- "-$pgid" 2>/dev/null; then printf ' ... SIGKILL'; kill -KILL -- "-$pgid" 2>/dev/null; fi
    printf ' done\n'
}

odom_rate() { timeout 12 ros2 topic hz /lidar_slam/odom --window 20 2>&1 | grep -m1 -oE "average rate: [0-9.]+" | grep -oE "[0-9.]+"; }
bool_topic() { timeout "${2:-6}" ros2 topic echo --once --field data "$1" 2>/dev/null | head -1; }

# ---------------------------------------------------------------------------------------------
cmd_stop() {
    env_up
    say "stopping -- recorder first, so rosbag2 writes metadata.yaml"
    local rec bag; rec=$(pgid_of REC); bag=$(grep -m1 '^BAG=' "$STATE" 2>/dev/null | cut -d= -f2-)
    if [ -n "$rec" ] && kill -0 "$rec" 2>/dev/null; then
        kill -INT "$rec" 2>/dev/null
        for _ in $(seq 1 40); do [ -f "$bag/metadata.yaml" ] && break; sleep 0.5; done
        [ -f "$bag/metadata.yaml" ] && ok "metadata.yaml written -- $bag ($(du -sh "$bag" 2>/dev/null | cut -f1))" \
                                    || note "WARNING: no metadata.yaml. Recover: ros2 bag reindex '$bag' -s sqlite3"
    else
        note "no recorder running"
    fi
    # Reverse of start: follower, then the bridges, then the stack underneath them.
    for g in DTP OFFBOARD ODOM STACK; do stop_group "$g"; done
    sleep 2
    local left
    left=$(ps -eo pid,comm --no-headers | grep -cE "fsm_node|fastlio_mapping|livox_ros_drive|px4_odom_bridge|px4_offboard_bridge|px4_offboard_br|px4_mpc_tracker|perception_node|target_manager|follow_planner|command_mux_nod|static_calib_tf|sensor_watchdog")
    [ "$left" -eq 0 ] && ok "0 survivors" || { ps -eo pid,pgid,comm --no-headers | grep -E "fsm_node|fastlio|livox_ros_drive|perception_node|target_manager"; die "$left survivor(s)" "stop them by pgid"; }
    free -m | awk 'NR==3{printf "   ok   swap used %s MB\n", $3}'
    : > "$STATE"
}

cmd_status() {
    env_up
    printf '%-12s %-8s %s\n' NAME PGID STATE
    for g in STACK ODOM OFFBOARD DTP REC; do
        local p; p=$(pgid_of "$g")
        if [ -z "$p" ]; then printf '%-12s %-8s %s\n' "$g" - "not started"
        elif kill -0 -- "-$p" 2>/dev/null || kill -0 "$p" 2>/dev/null; then printf '%-12s %-8s %s\n' "$g" "$p" running
        else printf '%-12s %-8s %s\n' "$g" "$p" exited; fi
    done
    echo
    printf '   lock  %s\n' "$(timeout 4 ros2 topic echo --once --field data /dtp/lock_state 2>/dev/null | head -1)"
    printf '   mux   %s\n' "$(timeout 4 ros2 topic echo --once --field data /mux/state 2>/dev/null | head -1)"
}

cmd_engage() {
    env_up
    say "engage FOLLOW -- the walker must be MOVING, at 4-6 m, and alone in the band"
    ros2 param set /target_manager follow_enabled false >/dev/null 2>&1
    sleep 1
    ros2 param set /target_manager follow_enabled true >/dev/null 2>&1 || die "could not set follow_enabled" "is target_manager up?"
    local L=""
    for i in $(seq 1 9); do
        sleep 3; L=$(timeout 4 ros2 topic echo --once --field data /dtp/lock_state 2>/dev/null | head -1)
        case "$L" in LOCKED*) break ;; esac
        note "${L:-no lock_state}"
    done
    case "$L" in
        LOCKED*)
            ok "$L"
            local rng; rng=$(echo "$L" | grep -oE "at [0-9.]+ m" | grep -oE "[0-9.]+")
            [ -n "$rng" ] && awk "BEGIN{exit !($rng > 7.0)}" && \
                note "*** $rng m is far. The last lock at 9.5 m lasted 5 s. Bring them to 4-6 m and re-run. ***"
            [ -n "$rng" ] && awk "BEGIN{exit !($rng < 3.2)}" && \
                note "*** $rng m is close to the 3.0 m launch-ignore disc. Back them off to 4-6 m. ***"
            ;;
        *) die "not LOCKED after 27 s: ${L:-nothing}" \
             "ACQUIRING = engaged, nobody qualified: not moving 0.4 m in 6 s, outside 2.5-10 m," \
             "inside the 3 m launch-ignore disc, or two people in the band." ;;
    esac
}

# ---------------------------------------------------------------------------------------------
cmd_up() {
    local mode=live record=1 goal_mode="" nmpc_cleared=0 tracker=bridge
    for a in "$@"; do
        case "$a" in
            --goal-mode=*) goal_mode="${a#--goal-mode=}" ;;
            --nmpc-cleared) nmpc_cleared=1 ;;
            --tracker=*) tracker="${a#--tracker=}" ;;
            --observe)   mode=observe ;;
            --live)      mode=live ;;
            --no-record) record=0 ;;
            *) die "unknown option: $a" "see the header of this file" ;;
        esac
    done

    case "$tracker" in
        bridge) tracker_exe=px4_offboard_bridge.py ;;
        mpc)    tracker_exe=px4_mpc_tracker.py ;;
        *) die "unknown --tracker=$tracker" "bridge (the default) or mpc" ;;
    esac

    say "0. preconditions"
    ps -eo comm --no-headers | grep -qE "^(fsm_node|fastlio_mapping|perception_node)$" \
        && die "something of ours is already running" "bash drone-dtp/scripts/deploy_follow.sh stop"
    ping -c1 -W1 192.168.1.154 >/dev/null 2>&1 && ok "MID360 reachable" || die "MID360 unreachable" "check eno1 and the LiDAR power"
    lsusb | grep -q 2bc5 && ok "Gemini on USB" || die "no Gemini on USB" "the follower cannot start without it"
    sudo -n jetson_clocks 2>/dev/null && sudo -n iw dev wlP1p1s0 set power_save off 2>/dev/null
    local gpu; gpu=$(( $(cat /sys/class/devfreq/17000000.gpu/cur_freq) / 1000000 ))
    [ "$gpu" -ge 900 ] && ok "GPU pinned $gpu MHz" || note "WARNING: GPU $gpu MHz -- run: sudo jetson_clocks"
    local free_gb; free_gb=$(df -BG --output=avail "$HOME/dtp_datasets" | tail -1 | tr -dc '0-9')
    [ "$free_gb" -ge 10 ] && ok "$free_gb GB free" || die "only $free_gb GB free" "a stage_b bag runs ~17 MB/s; have 10 GB"

    mkdir -p "$HOME/dtp_datasets"
    : > "$STATE"
    local stamp; stamp=$(date +%Y%m%d_%H%M)
    local logdir="$HOME/dtp_datasets/${stamp}_deploy"; mkdir -p "$logdir"
    env_up
    ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1

    say "1. stack on $FOLLOW_CONFIG (FAST-LIO pinned 6,7; fsm_node 0-5; nonblocking DDS)"
    start_group STACK "$logdir/stack.log" env SUPER_CONFIG="$FOLLOW_CONFIG" RVIZ=false bash "$REPO/scripts/run.sh"
    note "waiting for /lidar_slam/odom >= 9.5 Hz (settles in 0-3 min; nothing downstream starts before it)"
    local r=0
    for i in $(seq 1 14); do
        r=$(odom_rate); r=${r:-0}
        awk "BEGIN{exit !($r >= 9.5)}" && break
        printf '        %4.1f Hz (%d/14)\n' "$r" "$i"; sleep 8
    done
    awk "BEGIN{exit !($r >= 9.5)}" || die "odom $r Hz -- that start is bad" \
        "Do not move the aircraft. stop, then re-run this script."
    ok "odom $r Hz"

    # The two bridges and the follower share no state, so their warm-ups overlap. This is the whole
    # speed-up: the bridges settle in ~8 s each and the follower takes ~40 s (TensorRT engine load,
    # the camera, and the 5-frame GPU-vs-numpy cloud check), so serially that was ~56 s of the
    # bring-up and in parallel it is ~40 s.
    say "2. bridges + follower ($mode), in parallel"
    start_group ODOM     "$logdir/odom_bridge.log"     taskset -c 0-5 ros2 run super_bringup px4_odom_bridge.py
    # --tracker=mpc runs the MPC tracker in the bridge's place (same group name, same log, so
    # stop/status are unchanged). It refuses to start if a bridge already publishes setpoints.
    start_group OFFBOARD "$logdir/offboard_bridge.log" taskset -c 0-5 ros2 run super_bringup "$tracker_exe"
    note "trajectory: $tracker_exe"
    # --goal-mode=nmpc selects follow_planner's solver arm (docs/fact-nmpc-follow-goal.md). On
    # the launch line, not in dtp_params.yaml, so the arm is in this log and the preflight's.
    local follower_args=("${LEAN_ARGS[@]}")
    [ -n "$goal_mode" ] && follower_args+=("goal_mode:=$goal_mode")
    start_group DTP      "$logdir/follower.log"        env DTP_MODE="$mode" SUPER_CONFIG="$FOLLOW_CONFIG" \
                             bash "$REPO/scripts/dtp_follower.sh" "${follower_args[@]}"
    note "lean node set: ${follower_args[*]} (no dtp_viz, no dtp_image_overlay)"

    note "waiting for perception"
    local pok=""
    for i in $(seq 1 30); do
        pok=$(bool_topic /dtp/perception_ok 4)
        [ "$pok" = "True" ] && break
        sleep 3
    done
    [ "$pok" = "True" ] || die "perception_ok never went True" "read $logdir/follower.log"
    ok "perception_ok True"
    [ "$(bool_topic /dtp/sensors_ok 8)" = "True" ] && ok "sensors_ok True" || die "sensors_ok not True" "ros2 topic echo --once /dtp/sensor_health"
    grep -q "guard state -> TRACKING" "$logdir/odom_bridge.log" && ok "odom bridge TRACKING" || note "WARNING: odom bridge has not latched TRACKING"

    say "3. FAST-LIO gates"
    bash "$REPO/drone-dtp/scripts/field.sh" check || die "FAST-LIO gates failed" "read the lines above"

    if [ "$record" = 1 ]; then
        say "4. recorder (stage_b preset, generated from scripts/gcs/bagger.py so it cannot drift)"
        local bag="$HOME/dtp_datasets/${stamp}_stageB" topics="$logdir/stage_b_topics.txt"
        python3 -c "
import sys; sys.path.insert(0, '$REPO/scripts/gcs')
import bagger; print(' '.join(bagger.PRESETS['stage_b']))" > "$topics" || die "could not read the stage_b preset"
        # shellcheck disable=SC2046
        start_group REC "$logdir/record.log" ros2 bag record -o "$bag" $(cat "$topics")
        printf 'BAG=%s\n' "$bag" >> "$STATE"
        sleep 6
        kill -0 "$(pgid_of REC)" 2>/dev/null || die "recorder died" "read $logdir/record.log"
        ok "recording -> $bag"
    else
        note "recorder skipped (--no-record)"
    fi

    say "5. pre-arm gate"
    local pf=("$REPO/drone-dtp/scripts/stage_a_preflight.py")
    if [ "$mode" = live ]; then
        pf+=(--stage-b --target person --max-vel 3.0 --waive-d21)
        note "--max-vel 3.0 matches max_vel in $FOLLOW_CONFIG (user, 2026-09-26); --waive-d21 is the single-IMU"
        note "waiver. Both are operator decisions and both are printed in the output below."
    fi
    [ "$record" = 1 ] && pf+=(--require-record)
    [ "$nmpc_cleared" = 1 ] && pf+=(--nmpc-cleared)
    # ONCE, not twice. The preflight carries a 60 s estimator window (G0.2), so running it a
    # second time just to read its exit code doubled the slowest step in this whole script --
    # which is the opposite of the point. Capture the output, print the lines that matter, and
    # take the exit code from PIPESTATUS.
    local pf_out rc
    pf_out=$(python3 "${pf[@]}" 2>&1); rc=$?
    printf '%s\n' "$pf_out" | grep -E "^\s*\[(FAIL|WARN)\]|^(GO|NO-GO)" | sed 's/^/   /'
    if [ "$rc" -eq 0 ]; then
        say "READY"
    else
        say "NO-GO -- read the FAIL lines above. Nothing is armed."
        # auto_reacquire_after_lost was set true on 2026-09-15 at the operator's instruction, and
        # --stage-b FAILs on it by design, every run. That FAIL is the record of that decision, not
        # a new problem. Say so here rather than letting it read as a fresh fault each bring-up.
        if printf '%s\n' "$pf_out" | grep -q "auto_reacquire"; then
            note "NOTE: the auto_reacquire_after_lost line is EXPECTED -- you set it true on"
            note "2026-09-15 and --stage-b refuses it by design. Check whether it is the ONLY FAIL."
        fi
        note "Everything is left running; fix and re-run just the gate:"
        note "  python3 ${pf[*]}"
    fi
    note "logs: $logdir"
    note "engage when the walker is at 4-6 m and moving:"
    note "  bash drone-dtp/scripts/deploy_follow.sh engage"
    note "stop, after landing and disarm:"
    note "  bash drone-dtp/scripts/deploy_follow.sh stop"
}

case "${1:-up}" in
    stop)   cmd_stop ;;
    status) cmd_status ;;
    engage) cmd_engage ;;
    up)     shift 2>/dev/null || true; cmd_up "$@" ;;
    --*)    cmd_up "$@" ;;
    *)      die "unknown command: $1" "one of: up | engage | stop | status" ;;
esac
