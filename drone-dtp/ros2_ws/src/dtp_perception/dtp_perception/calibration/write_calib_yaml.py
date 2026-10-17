"""write_calib_yaml -- turn a solved extrinsic + captured intrinsics into the runtime calib file.

Stage 4: closes the loop. Produces exactly the schema fusion_node expects
(calib/drone_cam_livox.yaml -- T_camera_lidar 4x4 + intrinsics), so nobody has to hand-edit
YAML and silently transpose a matrix at 1am.

Stamps provenance into the file (when, from which capture run, how many pairs). A calibration
with no provenance is one nobody can later decide whether to trust -- which, given the mounts
change per airframe revision, is the failure mode this whole module exists to prevent.

REFUSES to write unless it can establish the camera serial and LiDAR IP. calib_io gates every node
that loads the result on those two keys -- a wrong camera BODY and a foreign LiDAR each refuse to
start -- and before 2026-09-12 this script accepted both flags and wrote NEITHER, so regenerating
the calibration silently turned both gates into log warnings. Sources, best first: the flags below,
<run_dir>/identity.yaml recorded by the capture, then autodetection of a single attached body.

Usage:
    python3 -m dtp_perception.calibration.write_calib_yaml \
        --run_dir ~/calib_runs/20260815_1200 \
        --calib_file ~/calib_runs/20260815_1200/T_camera_lidar.txt \
        --out ros2_ws/src/dtp_perception/calib/drone_cam_livox.yaml \
        --camera_serial CPC64630007M --lidar_ip 192.168.1.154 \
        --note "rig v2, Orin NX bench mount"
"""
import argparse
import glob
import os
import socket
import sys
import time

import numpy as np

# Same package, and deliberately the SAME function the runtime gate uses to enumerate camera
# bodies -- if the two ever disagreed about what a serial looks like, the gate would refuse a
# calibration this script had just declared valid. calib_io imports no ROS at module level, so
# this stays importable on the workstation where the solve actually runs.
from ..calib_io import attached_camera_serials
from .validate_calib import load_transform


def _read_identity_file(run_dir):
    """Identity recorded BY THE CAPTURE, if the capture recorded any.

    This is the most trustworthy source: it describes the hardware that actually produced the
    pairs, observed on the machine that observed them. A flag describes what a human typed at
    1am, and an autodetect describes whatever is plugged into the machine running THIS script
    -- which, for a koide3 solve, is a workstation with no camera attached at all.

    Hand-parsed line by line rather than via PyYAML, matching how camera_info.yaml is read a
    few lines below: the file is ours, flat, and two keys deep.
    """
    path = os.path.join(run_dir, 'identity.yaml')
    out = {}
    if not os.path.exists(path):
        return out, None
    for line in open(path):
        for key in ('camera_serial', 'lidar_ip'):
            if line.startswith(key + ':'):
                if _real_value(line.split(':', 1)[1]):
                    out[key] = line.split(':', 1)[1].strip()
    return out, path


# Placeholders that LOOK like a value and are not one. This list used to be applied only to
# identity.yaml, so a typed `--camera_serial unknown` was written into the calibration
# verbatim -- and because no attached body will ever report the serial "unknown",
# calib_io then REFUSES TO START on the aircraft. A typo became a field-side node failure
# with a message about a camera mismatch. One rule, every source.
_PLACEHOLDERS = ('', 'null', 'none', 'nil', 'nan', 'unknown', 'n/a', 'na', 'tbd', '?', '-',
                 'xxx', 'todo', 'fixme')


def _real_value(raw):
    """True when this is an actual identity and not a stand-in for not knowing."""
    if raw is None:
        return False
    return str(raw).strip().lower() not in _PLACEHOLDERS


def _looks_like_ipv4(value):
    """Dotted quad with every octet in range.

    Shape-checked rather than resolved: calib_io compares this string against the "ip" fields
    in the Livox driver's own config JSON, so a hostname or a typo like 192.168.1 does not
    fail here -- it fails later as a LIDAR MISMATCH on the aircraft, which reads like a wiring
    problem rather than a typing one.
    """
    parts = str(value).strip().split('.')
    if len(parts) != 4:
        return False
    for part in parts:
        if not part.isdigit() or not 0 <= int(part) <= 255:
            return False
    return True


def _autodetect_camera_serial():
    """The single attached Orbbec body, or None.

    Refuses to guess when more than one is attached. That is not caution for its own sake: the
    two-bodies case is the exact incident this gate exists for (2026-09-07, a Gemini 336 and a
    336L on one NX, the driver binding whichever it enumerated first), and calib_io documents
    that the driver's own serial_number parameter cannot be used to break the tie because it
    reports back whatever was asked for. So with two bodies there is no local source of truth,
    and a coin-flip written into a provenance field is worse than an honest blank.
    """
    attached = attached_camera_serials()
    return attached[0] if len(attached) == 1 else None


def _resolve_identity(a, run_dir):
    """Settle camera_serial and lidar_ip, and say where each came from.

    Priority: explicit flag > capture identity.yaml > local autodetect (camera only). An
    explicit flag that CONTRADICTS the capture is reported loudly rather than silently
    preferred -- a contradiction means either the flag is wrong or these pairs came from a
    different rig, and both are reasons to stop and look.
    """
    captured, captured_path = _read_identity_file(run_dir)
    conflicts = []
    resolved = {}

    rejected = []
    for key, flag in (('camera_serial', a.camera_serial), ('lidar_ip', a.lidar_ip)):
        if flag and not _real_value(flag):
            # Treated as NOT SUPPLIED, and said out loud. Accepting it would write a value
            # that no device can ever match.
            rejected.append('--%s %r is a placeholder, not an identity' % (key, flag))
            flag = None
        if flag and key == 'lidar_ip' and not _looks_like_ipv4(flag):
            rejected.append('--lidar_ip %r is not a dotted-quad IPv4 address' % (flag,))
            flag = None
        if flag:
            if captured.get(key) and captured[key] != flag:
                conflicts.append(
                    '%s: --%s says %s but the capture recorded %s (%s)'
                    % (key, key, flag, captured[key], captured_path))
            resolved[key] = (flag, 'explicit --%s' % key)
        elif captured.get(key):
            resolved[key] = (captured[key], 'recorded at capture (%s)'
                             % os.path.basename(captured_path))
        else:
            resolved[key] = (None, 'NOT RECORDED')

    if resolved['camera_serial'][0] is None:
        found = _autodetect_camera_serial()
        if found:
            resolved['camera_serial'] = (
                found, 'autodetected on %s (the only Orbbec attached)' % socket.gethostname())

    return resolved, conflicts + rejected


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--run_dir', required=True)
    ap.add_argument('--calib_file', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--note', default='')
    ap.add_argument('--lidar_ip', default=None,
                    help='IP of the LiDAR this calibration belongs to. Recorded so calib_io can '
                         'refuse a different device -- a point cloud carries no identity of its '
                         'own, and another robot on a shared DDS domain projects plausibly.')
    ap.add_argument('--camera_serial', default=None,
                    help='serial of the camera body this calibration belongs to. RECORD IT: two '
                         'Orbbec bodies on one NX let the driver bind the wrong one, and the '
                         'resulting calibration is structurally valid but belongs to a different '
                         'camera -- every range then comes out wrong with nothing to flag it. '
                         'calib_io refuses to start on a mismatch when this is present.')
    ap.add_argument('--allow_unidentified', action='store_true',
                    help='write the file even though the camera serial or LiDAR IP could not '
                         'be established. This DISABLES the corresponding calib_io '
                         'refuse-to-start gate for every node that loads the result, so the '
                         'file records that it was used.')
    a = ap.parse_args(argv if argv is not None else sys.argv[1:])

    run_dir = os.path.expanduser(a.run_dir)

    # Settled BEFORE any parsing or writing, because the failure is a refusal and there is no
    # reason to make the operator wait for it (or to leave a half-written YAML behind).
    identity, conflicts = _resolve_identity(a, run_dir)
    for line in conflicts:
        print('CONFLICT: %s' % line)
    if conflicts:
        print('         The flag wins, but one of the two is wrong about which rig these')
        print('         pairs came from. Resolve it before trusting this calibration.')

    missing = [k for k, (v, _) in identity.items() if v is None]
    if missing and not a.allow_unidentified:
        # *** A REFUSAL, NOT A WARNING. ***  This script used to print a warning here and exit
        # 0, which is how the deployed calibration came to carry hand-added identity keys that
        # no regeneration would ever reproduce. Silently handing back a file that turns two
        # refuse-to-start gates into log lines is the failure; the operator's inconvenience is
        # not comparable.
        print('REFUSING to write %s' % os.path.expanduser(a.out))
        print('  could not establish: %s' % ', '.join(missing))
        for key, (value, source) in sorted(identity.items()):
            print('    %-14s %-22s %s' % (key, value if value else '(unknown)', source))
        print('')
        print('  calib_io gates every node that loads this file on these two values: a camera')
        print('  body mismatch and a foreign LiDAR both refuse to start. Without them written')
        print('  here, both gates degrade to a log warning -- and a calibration belonging to a')
        print('  different camera is structurally valid, scores PASS, and makes every range to')
        print('  a person wrong with nothing to flag it.')
        print('')
        print('  Fix, best first:')
        print('    1. pass --camera_serial <serial> --lidar_ip <ip> explicitly')
        print('       camera: ls /sys/bus/usb/devices/*/serial on the capture machine')
        print('       lidar : the ip in the driver user_config_path JSON')
        print('    2. re-run on the capture machine, which can autodetect a single camera')
        print('    3. --allow_unidentified to proceed anyway, WITH the gates disabled')
        print('')
        print('  A placeholder is NOT a value. Writing \'unknown\' here would not waive the')
        print('  gate -- it would make calib_io refuse to start on the aircraft, because no')
        print('  attached camera will ever report that serial. Use --allow_unidentified.')
        return 2
    T = load_transform(os.path.expanduser(a.calib_file))
    n_pairs = len(glob.glob(os.path.join(run_dir, 'pair_*.npz')))

    ci = os.path.join(run_dir, 'camera_info.yaml')
    intr = {}
    res = None
    dist_model, dist = None, None
    for line in open(ci):
        for k in ('fx', 'fy', 'cx', 'cy'):
            if line.startswith(k + ':'):
                intr[k] = float(line.split(':')[1])
        if line.startswith('resolution:'):
            res = line.split(':', 1)[1].strip()
        if line.startswith('distortion_model:'):
            dist_model = line.split(':', 1)[1].strip()
        if line.startswith('d:'):
            dist = line.split(':', 1)[1].strip()

    out = os.path.expanduser(a.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w') as f:
        f.write("# camera<->LiDAR calibration -- GENERATED, do not hand-edit.\n")
        f.write("# Regenerate with dtp_perception.calibration (see\n")
        f.write("# plan/dtp-imp/19-camera-lidar-calibration-runbook.md).\n")
        f.write("#\n")
        f.write("# generated   : %s\n" % time.strftime('%Y-%m-%d %H:%M:%S %Z'))
        f.write("# capture run : %s (%d pairs)\n" % (run_dir, n_pairs))
        f.write("# solved from : %s\n" % a.calib_file)
        if a.note:
            f.write("# note        : %s\n" % a.note)
        # Short labels so these align with the generated/capture-run/solved-from headers above
        # -- this block is read by humans deciding whether to trust a calibration.
        for key, label in (('camera_serial', 'cam serial'), ('lidar_ip', 'lidar ip')):
            f.write("# %-12s: %s\n" % (label, identity[key][1]))
        f.write("#\n")

        # The two identity gates calib_io enforces at node start-up. Written as real keys,
        # not comments: load_calib reads doc['camera_serial'] and doc['lidar_ip'].
        cam, _ = identity["camera_serial"]
        ip, _ = identity["lidar_ip"]
        if cam:
            f.write("# This calibration belongs to ONE physical camera body."
                    " calib_io enforces it." + chr(10))
            f.write("camera_serial: %s\n" % cam)
            f.write("#\n")
        if ip:
            f.write("# ...and to ONE LiDAR. calib_io refuses a driver configured for another.\n")
            f.write("lidar_ip: %s\n" % ip)
            f.write("#\n")
        if not (cam and ip):
            # The artifact carries its own warning. Anyone who opens this file months from now
            # must be able to see that a gate was waived without reading the shell history of
            # whoever generated it.
            f.write("# !! ===================== IDENTITY GATES WAIVED ===================== !!\n")
            f.write("# !! Written with --allow_unidentified. The missing key(s) below are NOT\n")
            f.write("# !! recorded, so calib_io CANNOT refuse a wrong device and will only log\n")
            f.write("# !! a warning:\n")
            if not cam:
                f.write("# !!   camera_serial -- a different camera BODY will not be caught.\n")
            if not ip:
                f.write("# !!   lidar_ip      -- another robot's LiDAR will not be caught.\n")
            f.write("# !! Re-run write_calib_yaml with the missing value to restore the gate.\n")
            f.write("# !! ================================================================= !!\n")
            f.write("#\n")
        f.write("# !! THIS EXTRINSIC IS SPECIFIC TO ONE PHYSICAL MOUNT. Any change to how the\n")
        f.write("# !! camera or LiDAR is mounted INVALIDATES it. Re-run the calibration.\n")
        f.write("# !! Not yet re-validated MOTOR-ON unless the note above says so (vibration\n")
        f.write("# !! shifts the extrinsic -- plan/dtp-imp/05 Step 0.3.1b).\n\n")

        f.write("# 4x4 homogeneous transform, Livox Mid-360 frame -> camera optical frame.\n")
        f.write("T_camera_lidar:\n")
        for r in T:
            f.write("  - [%.9f, %.9f, %.9f, %.9f]\n" % tuple(r))
        f.write("\n")
        f.write("# Factory intrinsics read live from /camera/color/camera_info")
        if res:
            f.write(" at %s" % res)
        f.write(".\n")
        f.write("intrinsics:\n")
        for k in ('fx', 'fy', 'cx', 'cy'):
            f.write("  %s: %.6f\n" % (k, intr[k]))
        if dist_model:
            f.write("\ndistortion_model: %s\n" % dist_model)
        if dist:
            f.write("d: %s\n" % dist)

    for key, (value, source) in sorted(identity.items()):
        print('%-14s %-22s %s' % (key, value if value else '(UNKNOWN -- GATE WAIVED)', source))
    if missing:
        print("")
        print("*** %s not recorded. calib_io can no longer refuse a wrong device for this"
              % ", ".join(missing))
        print("*** calibration -- it will only log a warning. The file says so too.")
    print("wrote %s" % out)
    print(np.array2string(T, precision=5, suppress_small=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
