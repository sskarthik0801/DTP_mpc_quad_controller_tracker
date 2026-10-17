#!/usr/bin/env bash
# recalib.sh -- the two-tier camera/LiDAR recalibration, one stage per command.
#
#   bash scripts/recalib.sh which-tier <event>    # "I just did X -- what do I actually need?"
#   bash scripts/recalib.sh tier1                 # ON THE NX: the four re-dock assertions
#   bash scripts/recalib.sh capture [N]           # ON THE NX: N viewpoints (default 8)
#   bash scripts/recalib.sh push <run_dir>        # NX -> workstation
#   bash scripts/recalib.sh solve <run_dir>       # ON THE WORKSTATION: koide3 under xvfb
#   bash scripts/recalib.sh convert <run_dir>     # ON THE WORKSTATION: -> T_camera_lidar -> YAML
#   bash scripts/recalib.sh verify <run_dir>      # ON THE WORKSTATION: score + compare
#   bash scripts/recalib.sh deploy                # -> the NX, hash-verified
#   bash scripts/recalib.sh status                # where things currently stand
#
# *** THE TWO TIERS, AND WHY CONFLATING THEM HAS COST THIS PROJECT HOURS ***
#
# TIER 1 -- depth <-> colour.  The MANUFACTURER'S, burned into the camera body. Invalidated by
#   exactly ONE event: swapping the camera BODY. Re-docking, re-taping, knocking or dropping the
#   camera does NOT invalidate it -- the imagers and projector sit on one rigid substrate inside
#   the shell. There is nothing to re-solve here, only four ~2-minute ASSERTIONS that the thing
#   which breaks when hands go near the camera has not broken. `tier1` runs them.
#
# TIER 2 -- camera <-> LiDAR.  OURS, and invalidated by ANY mount change, either unit being
#   swapped, the first motor-on runs, and a flight-pack change. This is the expensive one:
#   capture -> solve -> convert -> verify -> deploy.
#
#   Tier 2 is load-bearing EVEN WITH depth_source:=camera. `static_calib_tf` derives the
#   body->camera transform from it, so every world position the follower computes goes through
#   it regardless of where the range came from. "We're using camera depth" is not an escape.
#
# WHY EACH STAGE IS ITS OWN COMMAND rather than one run-everything script: the stages live on
# DIFFERENT MACHINES (capture needs the sensors, solving needs GTSAM/Ceres/Iridescence and a
# display), they take minutes to tens of minutes, and the middle one is the one that fails. A
# single script would restart the 15-minute capture to retry a 30-second conversion.
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_HOST="${WS_HOST:-dtp-lab-ws}"
NX_HOST="${NX_HOST:-dtp-edge}"
CALIB_ENV="${CALIB_ENV:-$HOME/calib_ws/calib_env.sh}"
CALIB_YAML="$REPO/ros2_ws/src/dtp_perception/calib/drone_cam_livox.yaml"

die()  { printf '\nERROR: %s\n' "$1" >&2; exit 1; }
say()  { printf '\n=== %s ===\n' "$1"; }
note() { printf '  %s\n' "$1"; }

# ---------------------------------------------------------------------------------------
# Which machine are we on? Detected by CAPABILITY, not hostname: hostnames change, get
# re-imaged and get aliased, and a stage that runs on the wrong box fails slowly and
# confusingly (koide3 segfaults with no display; capture finds no camera). Override with
# DTP_HOST=nx|ws if the detection is ever wrong.
# ---------------------------------------------------------------------------------------
detect_host() {
    if [ -n "${DTP_HOST:-}" ]; then echo "$DTP_HOST"; return; fi
    # The solver workspace only exists where install_calib_solver.sh has run.
    if [ -f "$CALIB_ENV" ]; then echo ws; return; fi
    # An Orbbec body on USB means this is the machine with the sensors.
    for d in /sys/bus/usb/devices/*/idVendor; do
        [ -f "$d" ] || continue
        if [ "$(cat "$d" 2>/dev/null)" = "2bc5" ]; then echo nx; return; fi
    done
    echo unknown
}
HOST="$(detect_host)"

require_host() {
    local want="$1" what="$2"
    if [ "$HOST" = "$want" ]; then return 0; fi
    if [ "$HOST" = unknown ]; then
        die "cannot tell which machine this is, and '$what' must run on the $want.
  Detection looks for $CALIB_ENV (workstation) or an Orbbec body on USB (NX).
  If you know where you are, re-run with DTP_HOST=$want."
    fi
    die "'$what' must run on the $want, and this looks like the $HOST.
  On the NX:          bash scripts/recalib.sh tier1 | capture
  On the workstation: bash scripts/recalib.sh solve | convert | verify
  Override with DTP_HOST=$want if the detection is wrong."
}

require_ros() {
    [ -n "${ROS_DISTRO:-}" ] || die "ROS is not sourced. source your workspace setup.bash first."
}

# ---------------------------------------------------------------------------------------
cmd_which_tier() {
    local event="${1:-}"
    if [ -z "$event" ]; then
        cat <<'USAGE'
  bash scripts/recalib.sh which-tier <event>

  events:  redock  retape  knock  drop  camera-swap  lidar-swap
           motor-on  first-flight  pack-change  reboot  rebuild  new-airframe
USAGE
        return 2
    fi
    say "what '$event' actually invalidates"
    case "$event" in
        redock|remount|retape|knock|drop)
            note "TIER 1: NOT invalidated. depth<->colour is inside the camera body and did not"
            note "        move. Do NOT re-solve it. Run the four assertions:  recalib.sh tier1"
            note ""
            note "TIER 2: *** INVALIDATED. *** The camera moved relative to the LiDAR, which is"
            note "        exactly what tier 2 measures. Full sequence required."
            note "        Even with depth_source:=camera -- static_calib_tf derives body->camera"
            note "        from this, so every world position goes through it."
            ;;
        camera-swap)
            note "TIER 1: *** INVALIDATED. *** This is the ONE event that does it -- the factory"
            note "        depth<->colour belongs to the old body. The new body has its own."
            note "        Re-read intrinsics (the capture does this) and run: recalib.sh tier1"
            note ""
            note "TIER 2: *** INVALIDATED. *** New body, new mount geometry. Full sequence."
            note ""
            note "AND: the calibration's camera_serial now names the WRONG body, so calib_io will"
            note "     refuse to start until a new YAML is written. That refusal is correct."
            ;;
        lidar-swap)
            note "TIER 1: not invalidated -- nothing about the camera changed."
            note "TIER 2: *** INVALIDATED. *** Full sequence. Also update --lidar_ip: the new unit"
            note "        has a different IP and calib_io gates on it."
            ;;
        motor-on|first-flight)
            note "TIER 1: not invalidated."
            note "TIER 2: *** RE-VALIDATION REQUIRED (a red gate). *** Vibration shifts the"
            note "        extrinsic, and every solve so far is MOTORS OFF. Procedure: plan 19 s7."
            note "        Afterwards: compare_calib (p95 <= 1 px) and score_alignment."
            note "        Props on, RC bound, kill switch TESTED. Do not command motors from a"
            note "        script or over SSH -- throttle stays on the transmitter."
            ;;
        pack-change)
            note "TIER 1: not invalidated."
            note "TIER 2: invalidated in principle -- a different pack changes mass distribution"
            note "        and therefore vibration. Re-validate rather than re-solve: compare_calib"
            note "        against the current baseline and only re-solve if it fails."
            ;;
        reboot)
            note "NEITHER tier is invalidated by a reboot. BUT run the tier 1 body-identity"
            note "assertion anyway:  recalib.sh tier1"
            note ""
            note "On 2026-09-07 a reboot re-enumerated USB and the driver bound a DIFFERENT Orbbec"
            note "body than the one the calibration belongs to. Nothing downstream noticed."
            ;;
        rebuild)
            note "NEITHER. But note fsm_node reads SUPER's config from the SOURCE tree via the"
            note "compile-time ROOT_DIR, and colcon COPIES (no --symlink-install here), so"
            note "install/share can be arbitrarily stale. Verify by content hash, not presence:"
            note "  bash scripts/deploy_calib.sh   # idempotent, hashes both copies"
            ;;
        new-airframe)
            note "TIER 1: only if the camera body changed with it."
            note "TIER 2: *** INVALIDATED, and motor-on re-validation is required too. *** Treat as"
            note "        a fresh calibration. Use --note to record WHICH airframe -- that one"
            note "        field is what stops a v2 extrinsic being flown on a v3."
            ;;
        *)
            die "unknown event '$event'. Run with no argument for the list."
            ;;
    esac
    note ""
    note "Tier 2 full sequence:  tier1 -> capture -> push -> solve -> convert -> verify -> deploy"
}

# ---------------------------------------------------------------------------------------
cmd_tier1() {
    require_host nx tier1
    require_ros
    say "TIER 1 -- four assertions, no calibration"
    note "Needs the camera streaming. If it is not up:"
    note "  ros2 launch dtp_perception calib_sensors.launch.py enable_ir:=true"
    python3 "$REPO/scripts/tier1_camera_check.py" "$@"
}

# ---------------------------------------------------------------------------------------
cmd_capture() {
    require_host nx capture
    require_ros
    local pairs="${1:-8}"
    local run_dir="$HOME/calib_runs/$(date +%Y%m%d_%H%M)"
    say "TIER 2 stage 1 -- capture $pairs viewpoints"
    note "run_dir: $run_dir"
    note ""
    note "*** THE LiDAR MUST BE PUBLISHING PointCloud2, NOT CustomMsg. ***"
    note "calib_sensors.launch.py sets xfer_format:=0 for exactly this reason. FAST-LIO wants"
    note "xfer_format 1 (CustomMsg); koide3 cannot read it. A topic advertising BOTH types is"
    note "how three of eight viewpoints were silently lost on 2026-09-11 -- ros2 bag recorded"
    note "nothing while a typed subscriber read 10 Hz. capture_calib_data now REFUSES a"
    note "viewpoint whose bag has no LiDAR, so that failure is caught per-viewpoint."
    note ""
    note "Check before starting -- TWO types printed here means restart the driver AND the daemon:"
    ros2 topic type /livox/lidar 2>/dev/null | sed 's/^/    /' || note "    (no publisher yet)"
    note ""
    note "MOVE THE RIG between captures: vary yaw, pitch and standoff, keep real depth structure"
    note "in view. Near-identical viewpoints produce a confident and WRONG extrinsic."
    note ""
    mkdir -p "$run_dir"
    ros2 run dtp_perception capture_calib_data --ros-args \
        -p out_dir:="$run_dir" -p target_pairs:="$pairs" -p bag_secs:=15
    local rc=$?
    [ "$rc" -eq 0 ] || die "capture exited $rc"
    say "captured"
    note "pairs: $(ls "$run_dir"/pair_*.npz 2>/dev/null | wc -l)  bags: $(ls -d "$run_dir"/bags/pair_* 2>/dev/null | wc -l)"
    note "next:  bash scripts/recalib.sh push $run_dir"
}

# ---------------------------------------------------------------------------------------
cmd_push() {
    local run_dir="${1:-}"
    [ -n "$run_dir" ] || die "usage: recalib.sh push <run_dir>"
    [ -d "$run_dir" ] || die "no such run_dir: $run_dir"
    say "TIER 2 stage 2 -- transfer to $WS_HOST"
    note "bags are tens of GB; --partial --append-verify so an interrupted copy resumes."
    rsync -a --partial --append-verify --info=progress2 \
        "$run_dir/" "$WS_HOST:$(basename "$run_dir")_calib/" \
        || die "rsync failed"
    say "pushed"
    note "next, ON $WS_HOST:  bash scripts/recalib.sh solve ~/$(basename "$run_dir")_calib"
}

# ---------------------------------------------------------------------------------------
cmd_solve() {
    require_host ws solve
    local run_dir="${1:-}"
    [ -n "$run_dir" ] || die "usage: recalib.sh solve <run_dir>"
    [ -d "$run_dir" ] || die "no such run_dir: $run_dir"
    [ -f "$CALIB_ENV" ] || die "no $CALIB_ENV. Run: bash scripts/install_calib_solver.sh"
    # shellcheck disable=SC1090
    . "$CALIB_ENV"

    local out="$run_dir/koide3"
    local json="$out/calib.json"
    mkdir -p "$out"

    say "TIER 2 stage 3 -- koide3 solve"
    note "*** EVERY koide3 tool initialises GLFW and SEGFAULTS with no display, *** including the"
    note "non-interactive preprocess and calibrate. So everything below runs under xvfb-run."
    note "(This account has no X display of its own: :0 and :2 belong to other users.)"

    # *** A STALE RESULT MUST NOT BE REPORTABLE AS A FRESH ONE. ***
    # The ad-hoc solve.sh on this workstation read calib.json without checking exit codes, and
    # ~/solve.log shows it printing a confident 0.0922 m baseline from a PREVIOUS run after
    # calibrate had died when its Xvfb server dropped. So: remove the old result, and afterwards
    # require both a zero exit AND a file that did not exist before.
    rm -f "$json"
    local started_at="$out/.started"
    : > "$started_at"

    local XVFB=(xvfb-run -a -s "-screen 0 1920x1080x24")

    say "3a. preprocess"
    "${XVFB[@]}" ros2 run direct_visual_lidar_calibration preprocess "$run_dir/bags" "$out" -av \
        || die "preprocess failed (exit $?). Check that $run_dir/bags holds per-viewpoint bags
  WITH a LiDAR topic -- empty 99-byte .ply files downstream mean the bags had no cloud."

    # Guard the specific failure that produced NaN: viewpoints whose cloud never recorded become
    # near-empty .ply files, and calibrate then returns NaN rather than refusing.
    local tiny
    tiny=$(find "$out" -name '*.ply' -size -1k 2>/dev/null | wc -l)
    if [ "$tiny" -gt 0 ]; then
        die "$tiny near-empty .ply file(s) in $out -- those viewpoints recorded no cloud.
  calibrate would return NaN rather than refuse. Re-capture them; do not proceed."
    fi

    say "3b. initial guess (SuperGlue, automatic)"
    note "The manual GUI is unusable headless and gave nonsense on the previous airframe anyway."
    note "SuperGlue must run from its own checkout with it on PYTHONPATH, or it fails with"
    note "ModuleNotFoundError: No module named 'models'."
    ( cd "$HOME/SuperGluePretrainedNetwork" 2>/dev/null \
        && PYTHONPATH="$HOME/SuperGluePretrainedNetwork:${PYTHONPATH:-}" \
           "${XVFB[@]}" ros2 run direct_visual_lidar_calibration initial_guess_auto "$out" ) \
        || die "initial_guess_auto failed (exit $?). Is ~/SuperGluePretrainedNetwork present?"

    say "3c. calibrate"
    "${XVFB[@]}" ros2 run direct_visual_lidar_calibration calibrate "$out" \
        || die "calibrate failed (exit $?). Do NOT read $json -- any value in it is from an
  earlier run. This is the exact failure that reported a stale 0.0922 m as a fresh result."

    [ -f "$json" ] || die "calibrate exited 0 but wrote no $json. Treat as failure."
    if [ "$json" -ot "$started_at" ]; then
        die "$json is OLDER than the start of this run -- it is a leftover, not a result."
    fi
    rm -f "$started_at"

    say "solved"
    note "$json"
    note "next:  bash scripts/recalib.sh convert $run_dir"
}

# ---------------------------------------------------------------------------------------
cmd_convert() {
    require_host ws convert
    require_ros
    local run_dir="${1:-}"
    [ -n "$run_dir" ] || die "usage: recalib.sh convert <run_dir>"
    local json="$run_dir/koide3/calib.json"
    local txt="$run_dir/T_camera_lidar.txt"
    [ -f "$json" ] || die "no $json -- run 'solve' first."

    say "TIER 2 stage 4 -- convert"
    note "koide3 emits T_lidar_camera; we consume T_camera_lidar. import_koide3 inverts it."
    ros2 run dtp_perception import_koide3 --calib_json "$json" --out "$txt" || die "import failed"
    ros2 run dtp_perception validate_calib --run_dir "$run_dir" --calib_file "$txt" \
        || die "validate_calib failed -- do not write a YAML from this."

    say "stage 5 -- write the runtime YAML"
    note "*** write_calib_yaml REFUSES without --camera_serial and --lidar_ip. *** calib_io gates"
    note "every node that loads the result on those two; omitting them used to downgrade a"
    note "refuse-to-start gate to a log warning. Read them off the CAPTURE machine:"
    note "    ssh $NX_HOST 'cat /sys/bus/usb/devices/*/serial'"
    note "    grep '\"ip\"' <the Livox user_config_path JSON>"
    note ""
    local serial="${CAMERA_SERIAL:-}"
    local lidar_ip="${LIDAR_IP:-}"
    if [ -z "$serial" ] || [ -z "$lidar_ip" ]; then
        # Carry forward what the DEPLOYED calibration already claims, rather than making the
        # operator retype it -- but say so, because silently reusing identity across a hardware
        # change is precisely the mistake the gate exists to catch.
        [ -z "$serial" ] && serial=$(sed -n 's/^camera_serial: *//p' "$CALIB_YAML" 2>/dev/null)
        [ -z "$lidar_ip" ] && lidar_ip=$(sed -n 's/^lidar_ip: *//p' "$CALIB_YAML" 2>/dev/null)
        note "Reusing identity from the CURRENT deployed YAML:"
        note "    camera_serial = ${serial:-<none>}"
        note "    lidar_ip      = ${lidar_ip:-<none>}"
        note "IF EITHER UNIT WAS SWAPPED, STOP -- re-run with the right values:"
        note "    CAMERA_SERIAL=... LIDAR_IP=... bash scripts/recalib.sh convert $run_dir"
        note ""
    fi
    [ -n "$serial" ] || die "no camera serial. Set CAMERA_SERIAL=..."
    [ -n "$lidar_ip" ] || die "no lidar ip. Set LIDAR_IP=..."

    ros2 run dtp_perception write_calib_yaml \
        --run_dir "$run_dir" --calib_file "$txt" --out "$CALIB_YAML" \
        --camera_serial "$serial" --lidar_ip "$lidar_ip" \
        --note "${CALIB_NOTE:-recalib.sh $(date +%Y-%m-%d), MOTORS OFF (bench)}" \
        || die "write_calib_yaml refused (exit $?) -- read its message, it says which value."

    say "converted"
    note "next:  bash scripts/recalib.sh verify $run_dir"
}

# ---------------------------------------------------------------------------------------
cmd_verify() {
    require_host ws verify
    require_ros
    local run_dir="${1:-}"
    [ -n "$run_dir" ] || die "usage: recalib.sh verify <run_dir>"
    local txt="$run_dir/T_camera_lidar.txt"
    [ -f "$txt" ] || die "no $txt -- run 'convert' first."

    say "TIER 2 stage 6 -- independent check"
    note "score_alignment is the one that matters: it is INDEPENDENT of the solver's own cost."
    ros2 run dtp_perception score_alignment --run_dir "$run_dir" --calib_file "$txt"
    local score_rc=$?

    if [ -n "${BASELINE:-}" ]; then
        say "comparison against $BASELINE"
        note "*** compare_calib HARDCODES THE MOTOR-ON HYPOTHESIS. *** For a motors-off vs"
        note "motors-off comparison it will blame mount FLEX, which is the wrong cause (re-taping"
        note "DRIFT is the real one). Read its NUMBERS; ignore its explanation. Known defect."
        ros2 run dtp_perception compare_calib --baseline "$BASELINE" --test "$txt" \
            --run_dir "$run_dir"
    else
        note ""
        note "No BASELINE set, so no comparison was made. To compare against the previous"
        note "extrinsic (required after motor-on):"
        note "    BASELINE=<old T_camera_lidar.txt> bash scripts/recalib.sh verify $run_dir"
    fi

    [ "$score_rc" -eq 0 ] || die "score_alignment did not pass -- do NOT deploy this."
    say "verified"
    note "next:  bash scripts/recalib.sh deploy"
}

# ---------------------------------------------------------------------------------------
cmd_deploy() {
    say "TIER 2 stage 7 -- deploy to $NX_HOST"
    note "deploy_calib.sh is idempotent and verifies by CONTENT HASH, not presence. That check"
    note "earned itself on 2026-09-07: the NX was serving the PREVIOUS airframe's extrinsic at"
    note "the identical path, in source AND install. A presence check would have said OK."
    bash "$REPO/scripts/deploy_calib.sh" "$NX_HOST" || die "deploy failed"
    say "deployed"
    note "Then ON the NX: colcon build --packages-select dtp_perception, and re-run tier1."
    note "Motor-on re-validation is still owed if anything mechanical changed."
}

# ---------------------------------------------------------------------------------------
cmd_status() {
    say "detected host: $HOST"
    note "(override with DTP_HOST=nx|ws)"

    say "deployed calibration"
    if [ -f "$CALIB_YAML" ]; then
        # Whole file, not a line range: lidar_ip sits below any cutoff that still catches
        # camera_serial, and showing one identity key but not the other reads as a missing gate.
        grep -E '^# (generated|capture run|solved from|note|cam serial|lidar ip)|^camera_serial|^lidar_ip' \
            "$CALIB_YAML" | sed 's/^/  /'
        grep -q '^camera_serial:' "$CALIB_YAML" \
            && note "wrong-body gate ARMED" || note "wrong-body gate NOT ARMED (no camera_serial)"
        grep -q '^lidar_ip:' "$CALIB_YAML" \
            && note "foreign-LiDAR gate ARMED" || note "foreign-LiDAR gate NOT ARMED (no lidar_ip)"
        grep -q 'IDENTITY GATES WAIVED' "$CALIB_YAML" \
            && note "*** WRITTEN WITH --allow_unidentified: gates are OFF ***"
        grep -q 'MOTORS OFF' "$CALIB_YAML" \
            && note "bench-solved; motor-on re-validation still owed"
    else
        note "no $CALIB_YAML"
    fi

    say "capture runs on this machine"
    # Counted first. A `while` pipeline exits 0 on empty input, so the old
    # `... | while ...; done || note none` could never print its fallback -- the section just
    # rendered blank, which looks like a broken script rather than an empty directory.
    runs=$(ls -dt "$HOME"/calib_runs/*/ 2>/dev/null | wc -l)
    if [ "$runs" -eq 0 ]; then
        note "none under $HOME/calib_runs/"
    else
        ls -dt "$HOME"/calib_runs/*/ 2>/dev/null | head -5 | while read -r d; do
            printf '  %-44s pairs=%-3s bags=%s\n' "$d" \
                "$(ls "$d"/pair_*.npz 2>/dev/null | wc -l)" \
                "$(ls -d "$d"/bags/pair_* 2>/dev/null | wc -l)"
        done
    fi

    say "solver"
    [ -f "$CALIB_ENV" ] && note "present: $CALIB_ENV" \
        || note "absent (this is not the workstation, or install_calib_solver.sh has not run)"
}

# ---------------------------------------------------------------------------------------
SUB="${1:-}"
shift || true
case "$SUB" in
    which-tier) cmd_which_tier "$@" ;;
    tier1)      cmd_tier1 "$@" ;;
    capture)    cmd_capture "$@" ;;
    push)       cmd_push "$@" ;;
    solve)      cmd_solve "$@" ;;
    convert)    cmd_convert "$@" ;;
    verify)     cmd_verify "$@" ;;
    deploy)     cmd_deploy "$@" ;;
    status)     cmd_status "$@" ;;
    ''|-h|--help) awk '/^set -u/{exit} NR>1' "$0" ;;
    *)          die "unknown subcommand '$SUB'. Run with --help." ;;
esac
