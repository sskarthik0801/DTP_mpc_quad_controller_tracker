#!/usr/bin/env bash
# indoor_test.sh -- the whole DTP stack up indoors, with the checks, in one command.
#
# Run this ON THE AIRCRAFT (the NX). It brings up sensors + SUPER + follower + watchdog + dtp_viz in
# OBSERVE-ONLY mode, waits for each layer to actually be producing, PROVES on the live graph that
# nothing of ours can command the aircraft, and then walks you through the one check that has never
# been done: that the person markers render with a human in frame.
#
#     bash scripts/indoor_test.sh                 # full bring-up + checks
#     bash scripts/indoor_test.sh --no-launch     # stack already running, just run the checks
#     bash scripts/indoor_test.sh --keep          # leave the stack up when the checks finish
#     bash scripts/indoor_test.sh --live          # DANGEROUS: goals reach SUPER. Read section 5.
#     bash scripts/indoor_test.sh --no-walk       # nobody available to be the subject: skip 6-7
#     WALK_SECONDS=120 bash scripts/indoor_test.sh   # longer lock window (default 90)
#
# ONE PERSON CAN RUN THIS. Sections 6 and 7 do not wait for a keypress: they announce what to
# do, count down, run a fixed window, and print the verdict for you to read when you sit back
# down. The first version asked you to "press ENTER when in position", which with one person
# means the subject is at the keyboard -- and on 2026-09-12 the camera frame showed exactly
# that: every check had run against a person SEATED at the desk. Timed windows work with two
# people too (the second one waits through the countdown), so there is no interactive mode.
#
# WHAT "OBSERVE-ONLY" MEANS HERE, AND WHY IT IS PASSED EXPLICITLY
# --------------------------------------------------------------
# It passes observe_only:=true, which makes follower.launch.py redirect BOTH outputs that reach
# fsm_node -- the arbitrated goal to /dtp/shadow/follow_goal and the yaw target to
# /dtp/shadow/yaw_target -- and set command_mux abort_enable false, which is what removes the
# /fmu/in/vehicle_command publisher that would otherwise sit armed to send MAV_CMD_NAV_LAND.
#
# *** THIS FLAG USED TO BE MISSING AND THE HEADER CLAIMED OTHERWISE. ***  An adversarial review on
# 2026-09-12 found that dtp_system.launch.py neither declared nor forwarded observe_only, so this
# script brought up a LIVE follow stack -- command_mux publishing real goals onto
# /planning/follow_goal, which the deployed click_smooth_ros2.yaml subscribes (follow_goal_en true
# since 2026-09-10) -- while telling the operator in capital letters that it commanded nothing.
# That is why section 5 below VERIFIES the property on the live graph instead of asserting it. An
# assertion in a comment is what produced the bug.
#
# Even in observe-only the aircraft still cannot move without YOU: nothing here arms, selects
# Offboard, or publishes PX4 setpoints. Throttle stays on your transmitter.
set -u

CONFIG_NAME="${CONFIG_NAME:-click_smooth_ros2.yaml}"
DO_LAUNCH=1
KEEP=0
OBSERVE_ONLY=true
NO_WALK=0
WALK_SECONDS="${WALK_SECONDS:-90}"
for arg in "$@"; do
    case "$arg" in
        --no-launch) DO_LAUNCH=0 ;;
        --keep) KEEP=1 ;;
        --live) OBSERVE_ONLY=false ;;
        --no-walk) NO_WALK=1 ;;
        -h|--help) awk '/^set -u/{exit} NR>1' "$0"; exit 0 ;;
        *) echo "unknown argument: $arg"; exit 2 ;;
    esac
done

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LAUNCH_PID=""
LOG="/tmp/dtp_indoor_$(date +%H%M%S).log"

# NEVER pkill -f. It matches this script's own command line and has killed the controlling SSH
# session four separate times on this project. Kill the process group we started, by PID.
cleanup() {
    # If we die mid-window (Ctrl-C, SSH drop), FOLLOW must not stay engaged. In observe-only
    # that is harmless -- goals are shadowed -- but a stack left with follow_enabled true and
    # then relaunched --live would acquire the first walker it saw with nobody having chosen.
    if [ "${FOLLOW_ENGAGED:-0}" -eq 1 ]; then
        timeout 5 ros2 param set /target_manager follow_enabled false >/dev/null 2>&1 \
            && echo "  FOLLOW disengaged" || echo "  WARNING: could not disengage FOLLOW"
    fi
    if [ -n "$LAUNCH_PID" ] && [ "$KEEP" -eq 0 ]; then
        echo ""
        echo "stopping the stack (pgid $LAUNCH_PID) ..."
        kill -INT -- "-$LAUNCH_PID" 2>/dev/null
        for _ in $(seq 1 20); do
            kill -0 "$LAUNCH_PID" 2>/dev/null || break
            sleep 0.5
        done
        kill -0 "$LAUNCH_PID" 2>/dev/null && kill -TERM -- "-$LAUNCH_PID" 2>/dev/null
        # A surviving stack is not cosmetic: its publishers make the NEXT run's duplicate-publisher
        # and identity checks fire against our own leftovers.
        sleep 1
        if kill -0 "$LAUNCH_PID" 2>/dev/null; then
            echo "WARNING: pgid $LAUNCH_PID still alive. Check with: ps -o pid,pgid,cmd -g $LAUNCH_PID"
        fi
    elif [ -n "$LAUNCH_PID" ]; then
        echo ""
        echo "stack LEFT RUNNING (pgid $LAUNCH_PID). Stop it with:  kill -INT -- -$LAUNCH_PID"
    fi
}
trap cleanup EXIT

say() { printf '\n=== %s ===\n' "$1"; }
ok()  { printf '  ok   %s\n' "$1"; }
bad() { printf '  FAIL %s\n' "$1"; FAILED=$((FAILED + 1)); }
FAILED=0

# ---------------------------------------------------------------------------------------
say "0. environment"
# ---------------------------------------------------------------------------------------
# *** SOURCE THE JETSON ENV OURSELVES. ***  Nothing else does: neither .bashrc nor .profile
# on the NX sources ros2_ws/dtp_env_jetson.sh, and that file is the ONLY thing that puts
# libcudss.so.0 (under ~/.local/.../nvidia/cu12/lib) on LD_LIBRARY_PATH. Run from any fresh
# shell -- an SSH session, a cold terminal -- perception_node dies on ImportError before it
# ever opens the camera, and the failure presents as "no /dtp/targets" a minute later. Found
# on the second live run, 2026-09-12. The env file also sources ROS, super_ws and ws_livox.
ENV_FILE="$REPO/ros2_ws/dtp_env_jetson.sh"
if [ -f "$ENV_FILE" ]; then
    # shellcheck disable=SC1090
    set +u; . "$ENV_FILE" >/dev/null 2>&1; set -u
    ok "sourced $(basename "$ENV_FILE")"
fi
if [ -z "${ROS_DISTRO:-}" ]; then
    echo "  ROS is not sourced and $ENV_FILE is missing. source ~/super_ws/install/setup.bash."
    exit 2
fi
if ! python3 -c "import ctypes; ctypes.CDLL('libcudss.so.0')" 2>/dev/null; then
    bad "libcudss.so.0 is not loadable -- perception_node WILL die on import. Is dtp_env_jetson.sh"
    echo "       pointing at the right ~/.local nvidia/cu12/lib, and is the file present?"
else
    ok "libcudss.so.0 loadable (perception_node can import)"
fi
ok "ROS_DISTRO=$ROS_DISTRO  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo "  NOTE: the laptop running RViz must be on this same ROS_DOMAIN_ID."

if [ "$OBSERVE_ONLY" = "true" ]; then
    ok "observe_only:=true -- goals and yaw go to /dtp/shadow/*, NAV_LAND abort disabled"
else
    printf '  *** --live: goals WILL reach SUPER and the NAV_LAND abort is ENABLED. ***\n'
    printf '  *** Only do this with the props off or the aircraft restrained.         ***\n'
    printf 'Type LIVE to continue: '
    read -r confirm || confirm=""
    [ "$confirm" = "LIVE" ] || { echo "  aborted."; exit 2; }
fi

# ---------------------------------------------------------------------------------------
say "1. the viz layer's artifacts (it is the newest code and it can kill the whole launch)"
# ---------------------------------------------------------------------------------------
# dtp_system.launch.py resolves dtp_perception lazily now, so viz:=false is a real escape hatch --
# but with viz ON (the default) a missing launch file or missing console script still raises out of
# the include and LaunchService tears down everything it has already started. A --packages-select
# rebuild of only dtp_bringup reproduces that exactly, which is the likely state of the aircraft.
VIZ_OK=1
PERC_PREFIX="$(ros2 pkg prefix dtp_perception 2>/dev/null || true)"
if [ -z "$PERC_PREFIX" ]; then
    bad "dtp_perception is not installed -- launch with viz:=false until it is built."
    VIZ_OK=0
else
    for f in "$PERC_PREFIX/share/dtp_perception/launch/dtp_viz.launch.py" \
             "$PERC_PREFIX/lib/dtp_perception/dtp_viz"; do
        if [ -e "$f" ]; then
            ok "present: ${f#"$PERC_PREFIX/"}"
        else
            bad "MISSING: $f"
            echo "       Rebuild BOTH packages:"
            echo "         colcon build --packages-select dtp_perception dtp_bringup"
            echo "       Or launch with viz:=false to bring the stack up without it."
            VIZ_OK=0
        fi
    done
fi

# ---------------------------------------------------------------------------------------
say "2. calibration and its identity gates"
# ---------------------------------------------------------------------------------------
CALIB=""
[ -n "$PERC_PREFIX" ] && CALIB="$PERC_PREFIX/share/dtp_perception/calib/drone_cam_livox.yaml"
if [ -z "$CALIB" ] || [ ! -f "$CALIB" ]; then
    CALIB="$REPO/ros2_ws/src/dtp_perception/calib/drone_cam_livox.yaml"
fi
if [ ! -f "$CALIB" ]; then
    bad "no calibration found -- the follower will refuse to start."
else
    ok "calibration $CALIB"
    grep -q '^camera_serial:' "$CALIB" \
        && ok "camera_serial recorded (wrong-body gate ARMED)" \
        || bad "no camera_serial -- calib_io cannot refuse a different camera body."
    grep -q '^lidar_ip:' "$CALIB" \
        && ok "lidar_ip recorded (foreign-LiDAR gate ARMED)" \
        || bad "no lidar_ip -- calib_io cannot refuse another robot's LiDAR."
    grep -q 'IDENTITY GATES WAIVED' "$CALIB" \
        && bad "this calibration was written with --allow_unidentified. Gates are OFF."
    grep -q 'MOTORS OFF' "$CALIB" \
        && echo "  NOTE: extrinsic is bench-solved, NOT motor-on re-validated."
fi

# ---------------------------------------------------------------------------------------
if [ "$DO_LAUNCH" -eq 1 ]; then
    say "3. bring-up"
    VIZ_ARG=true
    [ "$VIZ_OK" -eq 1 ] || { VIZ_ARG=false; echo "  viz:=false (its artifacts are missing, see above)"; }

    # *** IS A SENSOR + SUPER STACK ALREADY UP? RIDE ON IT. ***
    #
    # Found on the first live run: Kevin's dashboard (super-gcs.service -> scripts/run.sh)
    # brings up the upstream Livox driver + m1_super_lidar (FAST-LIO + fsm_node) at boot.
    # Launching our own sensors and SUPER on top of that puts a SECOND FAST-LIO and a second
    # fsm_node on the same topics -- two estimators fighting over /lidar_slam/odom. His stack
    # underneath and ours on top IS the field configuration, so this is the normal case, not
    # the exception. Detected by a live odom publisher, which only FAST-LIO produces.
    STACK_ARGS=()
    ODOM_PUBS=$(timeout 8 ros2 topic info /lidar_slam/odom 2>/dev/null \
                | sed -n 's/^Publisher count: *//p' | head -1)
    if [ "${ODOM_PUBS:-0}" -ge 1 ]; then
        # lidar:=false, NOT sensors:=false. Kevin's run.sh starts only the Livox driver;
        # the Orbbec is still ours to launch. The first version of this branch turned both
        # sensors off and then waited 60 s for images nobody was going to publish.
        STACK_ARGS=(lidar:=false super:=false)
        ok "FAST-LIO already publishing /lidar_slam/odom -- riding on the running stack"
        echo "       (lidar:=false super:=false: camera + follower + watchdog + viz are ours)"
        # The upstream Livox launch names its node livox_lidar_publisher, which is ALSO what
        # the foreign robot was called on 2026-09-07 -- so calib_io cannot verify the LiDAR
        # is ours and logs UNVERIFIED. That is a warning, not a refusal; the follower starts.
        LIVOX_NODE=$(timeout 6 ros2 node list 2>/dev/null | grep -E 'livox|dtp_livox' | head -1)
        case "$LIVOX_NODE" in
            */dtp_livox_driver) ok "LiDAR node is ours ($LIVOX_NODE): identity gate can verify" ;;
            *livox_lidar_publisher)
                echo "  NOTE LiDAR node is the UPSTREAM name ($LIVOX_NODE). calib_io will log the"
                echo "       LiDAR source as UNVERIFIED -- expected on Kevin's launch, not a fault." ;;
            *) echo "  NOTE could not identify the LiDAR node (${LIVOX_NODE:-none})" ;;
        esac
    else
        echo "  no /lidar_slam/odom publisher -- launching sensors + SUPER ourselves"
    fi

    echo "  config_name  := $CONFIG_NAME"
    echo "  observe_only := $OBSERVE_ONLY"
    echo "  Why not follow_test_ros2.yaml: it carries max_vel 5.0 and preflight now refuses it."
    echo "  log -> $LOG"
    setsid ros2 launch dtp_bringup dtp_system.launch.py \
        "config_name:=$CONFIG_NAME" \
        "observe_only:=$OBSERVE_ONLY" \
        "viz:=$VIZ_ARG" "${STACK_ARGS[@]}" > "$LOG" 2>&1 &
    LAUNCH_PID=$!

    # Poll, do not sample once. preflight has several independent refusal sites (wall clock, Livox
    # typesupport, follow path, speed envelope) and a single 2 s liveness probe sees almost none of
    # them -- the launch exits a few seconds later and every topic wait below then burns its full
    # timeout for nothing.
    DIED=0
    for _ in $(seq 1 12); do
        sleep 1
        kill -0 "$LAUNCH_PID" 2>/dev/null || { DIED=1; break; }
    done
    if [ "$DIED" -eq 1 ]; then
        bad "launch EXITED during start-up. Refusals found in the log:"
        grep -nE 'PreflightError|REFUSING|above this stack ceiling|cannot be read from|not found' \
            "$LOG" | head -n 10 | sed 's/^/       /'
        echo "       --- last 25 lines ---"
        tail -n 25 "$LOG" | sed 's/^/       /'
        exit 1
    fi
    ok "still running after 12 s (pgid $LAUNCH_PID)"

    if [ "${#STACK_ARGS[@]}" -gt 0 ]; then
        # super:=false, so our preflight never ran against a config -- the running fsm_node
        # is Kevin's, on the config HIS launch loaded. Verify the limits it is flying with
        # directly, from the file m1_super_lidar.launch.py hardcodes.
        KCFG="$HOME/super_ws/src/SUPER/super_planner/config/click_smooth_ros2.yaml"
        if [ -f "$KCFG" ]; then
            MV=$(grep -m1 '^ *max_vel:' "$KCFG" | grep -oE '[0-9.]+')
            echo "  running fsm_node is Kevin's; its config: $(grep -E '^ *(max_vel|max_acc|max_omg|yaw_dot_max|follow_goal_en):' "$KCFG" | tr -s ' ' | tr '\n' ' ')"
            if [ -n "$MV" ] && awk "BEGIN{exit !($MV <= 1.5)}"; then
                ok "deployed max_vel $MV <= 1.5 m/s ceiling"
            else
                bad "deployed max_vel ${MV:-?} exceeds the 1.5 m/s ceiling (or is unreadable)"
            fi
        else
            bad "cannot read $KCFG to verify the limits the running fsm_node loaded"
        fi
    elif grep -q 'speed envelope OK' "$LOG"; then
        ok "$(grep -m1 'speed envelope OK' "$LOG" | sed 's/.*\[dtp_bringup\] //')"
    else
        # Not fatal by itself, but it means the gate did not run -- usually because preflight could
        # not locate the SUPER config at all, which it reports as UNVERIFIED rather than failing.
        bad "no 'speed envelope OK' in the log: the max_vel gate did not run. Check:"
        grep -nE 'preflight|UNVERIFIED' "$LOG" | head -n 6 | sed 's/^/       /'
    fi
    if [ "$OBSERVE_ONLY" = "true" ]; then
        grep -q 'OBSERVE-ONLY' "$LOG" \
            && ok "follower confirms OBSERVE-ONLY in its own log" \
            || bad "follower did NOT log OBSERVE-ONLY -- observe_only may not have reached it."
    fi
else
    say "3. bring-up SKIPPED (--no-launch)"
fi

# ---------------------------------------------------------------------------------------
say "4. waiting for each layer to actually produce"
# ---------------------------------------------------------------------------------------
# Topics, not node liveness. A node that is up and publishing nothing is the failure mode this
# project keeps hitting -- the Orbbec kept its process, USB enumeration and topic registrations
# while publishing nothing at all (2026-09-07).
#
# 8 s per attempt, not 3: a cold `ros2 topic echo` on a loaded Orin NX pays for node creation and
# discovery before it can receive anything, and 3 s reports FAIL on healthy topics.
wait_topic() {
    local topic="$1" label="$2" budget="${3:-60}"
    local t0=$SECONDS
    while [ $((SECONDS - t0)) -lt "$budget" ]; do
        if timeout 8 ros2 topic echo --once "$topic" > /dev/null 2>&1; then
            ok "$label ($topic, $((SECONDS - t0))s)"
            return 0
        fi
        sleep 2
    done
    bad "$label -- nothing on $topic after ${budget}s"
    return 1
}

wait_topic /livox/lidar            "LiDAR publishing"            60
wait_topic /camera/color/image_raw "camera publishing"           60
wait_topic /lidar_slam/odom        "FAST-LIO has a pose"         90
wait_topic /dtp/viz               "dtp_viz publishing markers"   60

# Duplicate publishers AFTER our stack is up -- running this before the launch can only ever see
# zero, which made the original check decorative. Ours counts as one; two means a foreign driver,
# and on 2026-09-07 another lab's robot published under the upstream node name on this domain and
# our stack bound THEIR LiDAR, plausibly and silently.
LIVOX_PUBS=$(ros2 topic info /livox/lidar 2>/dev/null | sed -n 's/^Publisher count: *//p' | head -1)
LIVOX_PUBS="${LIVOX_PUBS:-0}"
if [ "$LIVOX_PUBS" -le 1 ]; then
    ok "$LIVOX_PUBS publisher on /livox/lidar"
else
    bad "$LIVOX_PUBS publishers on /livox/lidar -- at least one is NOT ours."
    echo "       ros2 topic info -v /livox/lidar    then stop the other or isolate the domain."
fi

# The frame everything is drawn in. It has been wrong in this project's configs twice
# (camera_init, odom), and a wrong Fixed Frame shows as an empty RViz with healthy topics.
FRAME=$(timeout 8 ros2 topic echo --once --field header.frame_id /lidar_slam/odom 2>/dev/null \
        | tr -d '\r"' | head -1)
if [ "$FRAME" = "lidar_init" ]; then
    ok "world frame is lidar_init (matches dtp_follow.rviz Fixed Frame)"
elif [ -n "$FRAME" ]; then
    bad "world frame is '$FRAME', but dtp_follow.rviz has Fixed Frame lidar_init -> empty RViz."
else
    bad "could not read header.frame_id from /lidar_slam/odom -- Fixed Frame UNVERIFIED."
fi

# ---------------------------------------------------------------------------------------
say "5. PROVING the stack cannot command the aircraft"
# ---------------------------------------------------------------------------------------
# The header's claim, checked against the live graph rather than asserted. This section is the
# direct result of that claim having been FALSE: observe_only was never forwarded, and no check
# existed that would have contradicted the comment.
if [ "$OBSERVE_ONLY" = "true" ]; then
    # A raw publisher count CANNOT tell ours from Kevin's: gcs_telemetry keeps a publisher
    # registered on /planning/follow_goal for the dashboard's Remote-control card, and on the
    # second live run that alone produced a FAIL here while stage_a_preflight, one line
    # later, correctly said "none of ours". So the count is INFORMATION; the verdict comes
    # from the gate that knows which node is which.
    for t in /planning/follow_goal /planning/yaw_target /fmu/in/vehicle_command; do
        n=$(ros2 topic info "$t" 2>/dev/null | sed -n 's/^Publisher count: *//p' | head -1)
        printf '  info %-30s %s publisher(s)\n' "$t" "${n:-0}"
    done
    if [ -f "$REPO/scripts/stage_a_preflight.py" ]; then
        echo "  running the repo's own observe-only gate (it knows whose publishers are whose):"
        PF_OUT=$(python3 "$REPO/scripts/stage_a_preflight.py" 2>&1)
        PF_RC=$?
        echo "$PF_OUT" | sed 's/^/    /'
        # The no-command property specifically, independent of the gate's other checks
        # (jetson_clocks, bag recording, camera rates) which are real but a different question.
        for t in /planning/follow_goal /planning/yaw_target /fmu/in/vehicle_command; do
            line=$(echo "$PF_OUT" | grep -F "publishers on $t" | head -1)
            case "$line" in
                *"none of ours"*) ok "nothing of ours publishes $t" ;;
                "")               bad "stage_a_preflight did not report on $t" ;;
                *)                bad "OURS publishes $t -- the stack CAN command the aircraft: ${line#*]}" ;;
            esac
        done
        if [ "$PF_RC" -eq 0 ]; then
            ok "stage_a_preflight GO overall"
        else
            echo "  NOTE stage_a_preflight NO-GO overall -- read its lines above. Its non-command"
            echo "       checks (jetson_clocks, bag recording, sensor rates) are pre-ARM items,"
            echo "       not indoor-test items; the command-path lines are what this section gates."
        fi
    else
        bad "scripts/stage_a_preflight.py missing -- cannot verify whose publishers these are"
    fi
else
    printf '  SKIPPED: running --live, so the stack CAN command. This is the mode you asked for.\n'
fi

# ---------------------------------------------------------------------------------------
say "6. person markers (timed -- no keypress)"
# ---------------------------------------------------------------------------------------
# countdown N "message": announce, then count down out loud so the subject can pace
# themselves from across the room. Prints one line per second; the whole point is that
# nobody has to be at the keyboard.
countdown() {
    local n="$1" msg="$2"
    echo "  $msg"
    for s in $(seq "$n" -1 1); do printf '  ... %2d\r' "$s"; sleep 1; done
    printf '  ... go  \n'
}
if [ "$NO_WALK" -eq 1 ]; then
    echo "  --no-walk: skipping. Person and lock markers are UNVERIFIED on this run."
else
    echo "  Baseline first, from wherever you are (a seated operator counts as a person here):"
    python3 "$REPO/scripts/verify_dtp_viz.py" --seconds 8 | grep -E "^(people|veto|status) |^PASS|^FAIL"
    echo ""
    countdown 15 "GET UP and STAND 3-4 m in front of the camera, facing it. 15 s to get there."
    python3 "$REPO/scripts/verify_dtp_viz.py" --seconds 20 --expect-person
    VIZ_RC=$?
    [ "$VIZ_RC" -eq 0 ] && ok "person markers render" || bad "person markers did NOT render (see above)"
fi

# ---------------------------------------------------------------------------------------
say "7. the lock path (timed -- WALK for $WALK_SECONDS s)"
# ---------------------------------------------------------------------------------------
# Done by hand on 2026-09-12 and it is the check that matters most: acquire, hold, forecast.
# Observe-only ONLY. With --live, engaging FOLLOW here would send real goals to SUPER on the
# operator's behalf while they are across the room -- that is the operator's call to make,
# at the keyboard, on purpose.
if [ "$NO_WALK" -eq 1 ]; then
    echo "  --no-walk: skipped."
elif [ "$OBSERVE_ONLY" != "true" ]; then
    echo "  --live: NOT engaging FOLLOW for you. Engage it yourself when you mean it:"
    echo "      ros2 param set /target_manager follow_enabled true"
else
    echo "  Acquisition needs a CONFIRMED track 2.5-10 m out, >3 m from the aircraft, MOVING"
    echo "  >= 0.4 m inside 6 s. Standing still is what prevents a lock. Walk the whole window."
    countdown 10 "WALK back and forth in front of the drone, 3-6 m out, the WHOLE time. 10 s."
    LOCK_LOG="/tmp/dtp_lock_$(date +%H%M%S).txt"
    ros2 param set /target_manager follow_enabled false >/dev/null 2>&1; sleep 1
    ros2 param set /target_manager follow_enabled true >/dev/null 2>&1 && FOLLOW_ENGAGED=1
    # stdbuf, or uniq buffers until EOF and the file is empty when we read it.
    ( timeout $((WALK_SECONDS + 2)) stdbuf -oL ros2 topic echo --field data /dtp/lock_state 2>/dev/null \
        | stdbuf -oL grep -v '^---' | stdbuf -oL uniq > "$LOCK_LOG" ) &
    python3 "$REPO/scripts/verify_dtp_viz.py" --seconds $((WALK_SECONDS - 2)) --expect-lock \
        | grep -E "^(people|forecast|standoff_goal) |^MarkerArray|^PASS|^FAIL --|^\s+\* "
    sleep 4
    ros2 param set /target_manager follow_enabled false >/dev/null 2>&1; FOLLOW_ENGAGED=0
    echo ""
    echo "  lock transitions:"
    sed 's/^/      /' "$LOCK_LOG"
    LOCKS=$(grep -c "^LOCKED" "$LOCK_LOG"); LOSTS=$(grep -c "^LOST" "$LOCK_LOG")
    REACQ=$(grep -c "re-acquired" "$LOCK_LOG")
    if [ "$LOCKS" -eq 0 ]; then
        bad "never LOCKED in $WALK_SECONDS s. Was anyone walking? (a seated person never passes the motion gate)"
    elif [ "$LOSTS" -eq 0 ] && [ "$REACQ" -eq 0 ]; then
        ok "LOCKED once and HELD for the window -- no coast, no loss"
    else
        bad "lock CYCLED: $LOSTS lost, $REACQ re-acquired. Every COASTING is an empty frame;"
        echo "       check empty-frame count in $LOG and the TF wait with scripts/odom_lag_probe.py."
    fi
fi

# ---------------------------------------------------------------------------------------
say "summary"
# ---------------------------------------------------------------------------------------
if [ "$FAILED" -eq 0 ]; then
    echo "  ALL CHECKS PASSED."
    echo ""
    echo "  Still NOT established, and none of it is checkable from here:"
    echo "    * that RViz on the laptop draws these markers -- copy dtp_follow.rviz over and look:"
    echo "        rviz2 -d <path>/dtp_follow.rviz    (dtp_perception is NOT installed on dtp-lab-ws)"
    echo "    * that marker POSITIONS are right. Tape-measure one range before trusting any."
    [ "$NO_WALK" -eq 1 ] && echo "    * person and lock markers (--no-walk). Re-run with a subject."
    echo "    * anything motor-on. The extrinsic is bench-solved."
else
    echo "  $FAILED CHECK(S) FAILED -- see above. Full launch log: $LOG"
fi
exit $((FAILED > 0))
