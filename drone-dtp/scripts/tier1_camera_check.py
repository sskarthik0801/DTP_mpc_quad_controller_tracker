#!/usr/bin/env python3
"""tier1_camera_check -- the four assertions a camera re-dock needs. NONE of them is a calibration.

*** WHY TIER 1 IS NOT A CALIBRATION, AND WHY THAT KEEPS GETTING CONFUSED ***

The depth<->colour registration inside a Gemini 336L is the MANUFACTURER'S, burned into the camera
body. Exactly one event invalidates it: swapping the camera BODY. Re-docking the camera, re-taping
it, knocking it, or dropping it does NOT -- the two imagers and the projector do not move relative
to each other, because they are all on one rigid substrate inside the same shell.

So after a re-dock there is nothing to re-solve at this tier. What there IS, is a set of
things that
quietly break when hands go near the camera, each of which has actually happened on this project:

  1. THE WRONG BODY GOT BOUND.  2026-09-07: two Orbbec bodies on one NX (a 336 on USB 2.0 and the
     336L on USB 3.0), no selector, and the driver took whichever it enumerated first. After a
     reboot it took the 336 -- while our calibration belongs to the 336L. Nothing downstream
     notices: the YAML is valid, score_alignment passes, overlays look plausible, and every range
     to a person is silently wrong.
  2. AN IR WINDOW IS COVERED.  2026-09-08: depth was 0.2 % valid because a strip of tape was over
     the LEFT IR window. Checked by STANDARD DEVIATION, never the mean -- a taped window is not
     dark, it is FLAT. A bright uniform grey has a perfectly healthy mean and a std near zero.
  3. THE depth->colour TF NEVER ARRIVED.  Without it, anything projecting depth into the colour
     frame either fails or silently uses an identity transform.
  4. DEPTH CAME BACK DEGRADED BUT NOT DEAD.  The reference is ~88.4 % valid pixels at 30 Hz
     (measured 2026-09-08 indoors). Half that is a fault; it is just not an obvious one.

Tier 2 (camera<->LiDAR, ours) is a different matter entirely and IS invalidated by any re-dock.
This script does not touch it. See scripts/recalib.sh.

    python3 scripts/tier1_camera_check.py                  # all four, ~90 s
    python3 scripts/tier1_camera_check.py --skip-ir        # if the IR streams are not enabled

Exit 0 = every assertion passed. Exit 1 = at least one failed, and the report says which and why.
"""
import argparse
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

# The reference from 2026-09-08, indoors, after the tape came off the left IR window.
REFERENCE_VALID_PCT = 88.4
# Below this, depth is degraded rather than merely noisy. Half the reference is generous: the
# honest reading of a 45 % result is "something is wrong", not "acceptable variation".
MIN_VALID_PCT = 60.0

# A taped window is FLAT, not dark. Tape over the left window produced a std near zero while the
# mean stayed unremarkable; this is the threshold that separates "looking at a scene" from
# "looking at the inside of a piece of tape".
MIN_IR_STD = 8.0

COLOR_FRAME = 'camera_color_optical_frame'
DEPTH_FRAME = 'camera_depth_optical_frame'


class Tier1(Node):
    def __init__(self):
        super().__init__('tier1_camera_check')
        self.depth = []
        self.left_ir = None
        self.right_ir = None
        self.color_info = None
        self.create_subscription(Image, '/camera/depth/image_raw', self._on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/left_ir/image_raw', self._on_left,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/right_ir/image_raw', self._on_right,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self._on_info, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    @staticmethod
    def _as_array(msg, dtype):
        return np.frombuffer(bytes(msg.data), dtype=dtype)

    def _on_depth(self, msg):
        # 16UC1 millimetres. Zero means "no return", which is what validity counts.
        if msg.encoding not in ('16UC1', 'mono16'):
            return
        a = self._as_array(msg, np.uint16)
        if a.size:
            self.depth.append(100.0 * float(np.count_nonzero(a)) / a.size)

    def _on_left(self, msg):
        self.left_ir = self._ir_std(msg)

    def _on_right(self, msg):
        self.right_ir = self._ir_std(msg)

    def _ir_std(self, msg):
        if msg.encoding in ('16UC1', 'mono16'):
            a = self._as_array(msg, np.uint16).astype(np.float32) / 257.0
        elif msg.encoding in ('mono8', '8UC1'):
            a = self._as_array(msg, np.uint8).astype(np.float32)
        else:
            return None
        return (float(np.mean(a)), float(np.std(a))) if a.size else None


def attached_serials():
    """Orbbec bodies on THIS machine, from sysfs.

    Read here rather than from the driver's `serial_number` parameter, which is the selector you
    ASKED for and not the device that was bound -- verified 2026-09-07, when it reported a serial
    with no camera plugged in at all. Gating on it would pass by construction.
    """
    out = []
    base = '/sys/bus/usb/devices'
    try:
        entries = os.listdir(base)
    except OSError:
        return out
    for name in entries:
        d = os.path.join(base, name)
        try:
            with open(os.path.join(d, 'idVendor')) as fh:
                if fh.read().strip().lower() != '2bc5':
                    continue
            with open(os.path.join(d, 'serial')) as fh:
                s = fh.read().strip()
            if s:
                out.append(s)
        except OSError:
            continue
    return sorted(set(out))


def calib_serial():
    """The camera_serial the deployed calibration claims, or None."""
    try:
        prefix = subprocess.check_output(['ros2', 'pkg', 'prefix', 'dtp_perception'],
                                         stderr=subprocess.DEVNULL).decode().strip()
    except (subprocess.CalledProcessError, OSError):
        return None, None
    path = os.path.join(prefix, 'share', 'dtp_perception', 'calib', 'drone_cam_livox.yaml')
    if not os.path.exists(path):
        return None, path
    with open(path, encoding='utf-8') as fh:
        for line in fh:
            if line.startswith('camera_serial:'):
                return line.split(':', 1)[1].strip(), path
    return None, path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=12.0)
    ap.add_argument('--skip-ir', action='store_true',
                    help='the IR streams are off (enable_ir:=false). Assertion 2 is then SKIPPED '
                         'and reported as such, never as passed.')
    a = ap.parse_args()

    results = []

    def record(name, ok, detail):
        results.append((name, ok, detail))
        mark = 'ok  ' if ok is True else ('SKIP' if ok is None else 'FAIL')
        print('%s  %-28s %s' % (mark, name, detail))

    print('TIER 1 -- depth<->colour is FACTORY. This verifies nothing moved that should not have.')
    print('')

    # ---- 1. the bound body -------------------------------------------------------------
    expected, path = calib_serial()
    attached = attached_serials()
    if expected is None:
        record('camera body identity', False,
               'the deployed calibration records NO camera_serial (%s) -- nothing '
               'can verify which body this is. Re-run write_calib_yaml with '
               '--camera_serial.'
               % (path or 'not found',))
    elif not attached:
        record('camera body identity', False,
               'no Orbbec body found in sysfs. Calibration expects %s. Is the camera plugged in?'
               % expected)
    elif expected not in attached:
        record('camera body identity', False,
               'WRONG BODY. calibration belongs to %s; attached: %s. The driver cannot have bound '
               'the right one, so every range will be wrong with nothing to flag it.'
               % (expected, ', '.join(attached)))
    elif len(attached) > 1:
        record('camera body identity', False,
               '%s IS attached, but so are %s. With no selector the driver takes whichever it '
               'enumerated first (2026-09-07). Unplug the others or pin serial_number.'
               % (expected, ', '.join(s for s in attached if s != expected)))
    else:
        record('camera body identity', True, '%s, the only one attached' % expected)

    # ---- live streams ------------------------------------------------------------------
    rclpy.init()
    node = Tier1()
    print('')
    print('watching the camera for %.0f s ...' % a.seconds)
    deadline = time.time() + a.seconds
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    print('')

    # ---- 2. the IR windows -------------------------------------------------------------
    if a.skip_ir:
        record('IR windows clear', None, 'skipped (--skip-ir). NOT verified.')
    else:
        for label, got in (('left', node.left_ir), ('right', node.right_ir)):
            if got is None:
                record('IR window (%s)' % label, False,
                       'no %s IR frames. Launch with enable_ir:=true, or pass --skip-ir if that '
                       'is deliberate.' % label)
            else:
                mean, std = got
                ok = std >= MIN_IR_STD
                record('IR window (%s)' % label, ok,
                       'mean %.1f std %.1f (need std >= %.1f -- tape is FLAT, not dark, so the '
                       'mean proves nothing)' % (mean, std, MIN_IR_STD))

    # ---- 3. the depth->colour TF -------------------------------------------------------
    try:
        node.tf_buffer.lookup_transform(COLOR_FRAME, DEPTH_FRAME, rclpy.time.Time())
        record('depth->colour TF', True, '%s -> %s present' % (DEPTH_FRAME, COLOR_FRAME))
    except Exception as exc:
        record('depth->colour TF', False,
               'no %s -> %s (%s). Anything projecting depth into the colour frame either fails or '
               'silently assumes identity.' % (DEPTH_FRAME, COLOR_FRAME, type(exc).__name__))

    # ---- 4. depth validity -------------------------------------------------------------
    if not node.depth:
        record('depth validity', False,
               'no depth frames on /camera/depth/image_raw at all.')
    else:
        pct = float(np.mean(node.depth))
        record('depth validity', pct >= MIN_VALID_PCT,
               '%.1f %% valid over %d frames (reference %.1f %%, floor %.1f %%)'
               % (pct, len(node.depth), REFERENCE_VALID_PCT, MIN_VALID_PCT))

    rclpy.shutdown()

    failed = [n for n, ok, _ in results if ok is False]
    skipped = [n for n, ok, _ in results if ok is None]
    print('')
    if failed:
        print('TIER 1 FAILED: %s' % ', '.join(failed))
        print('')
        print('NOTE: none of these is fixed by re-calibrating. They are things that moved,')
        print('got covered, or got mis-bound. Fix the cause, then re-run this.')
        return 1
    print('TIER 1 PASSED%s.' % (' (with %d SKIPPED)' % len(skipped) if skipped else ''))
    print('')
    print('This says the camera is the right body, sees through both windows, publishes its')
    print('depth->colour TF and returns depth at the expected rate. It says NOTHING about')
    print('camera<->LiDAR, which a re-dock DOES invalidate -- run tier 2 for that.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
