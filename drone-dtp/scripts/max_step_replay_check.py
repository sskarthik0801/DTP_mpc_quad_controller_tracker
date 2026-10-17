#!/usr/bin/env python3
"""Replay gate for perception_node's `max_step_m` and `smooth_alpha` (the predictor's per-frame
motion clamp and the EMA behind it).

    python3 drone-dtp/scripts/max_step_replay_check.py                 # the gate, from the fixture
    python3 drone-dtp/scripts/max_step_replay_check.py --regenerate    # rebuild the fixture (GPU)

WHAT IT TESTS. Flight 2026-10-02 20:03 (bags/stage_b_2026-10-02_200220, bag t 43-47): the walker
jogged away at ~2 m/s, the detector boxed them and the stereo depth read 4 -> 9.3 m, but the
PUBLISHED range read a constant 3.6-3.7 m for three seconds, until the track was dropped and
rebuilt at 9.4 m (flight-log.md). The cause is detection_pipeline/detection.py's position filter
on a confirmed track: `_limit_motion_step` clamps the world-frame move to `max_step_m`, then
`_smooth_xz` EMA-blends with `smooth_alpha`, so the published position moves at most
alpha * max_step per processed frame -- 0.0875 m at the library default 0.35, i.e. 0.9 m/s at
the ~10.5 Hz flown. The aircraft followed that lagging goal at the same speed, so the gap did
not change.

The fixture holds, per processed frame of that window, the aircraft's pose and the RAW world
position of the walker as the detector + depth measured it (made by --regenerate: the flown
detector on the bag's colour frames, the torso patch on the recorded depth, lifted through the
aircraft's odometry; the camera extrinsic is approximated as the body frame, which is fine for
a filter test). The gate feeds that series through the REAL filter methods of
`HumanXZPredictor` (no model is loaded) and checks:

  F1  with max_step_m 0.35 the filtered range stays flat (reproduces the flight);
  F2  with max_step_m 0.35 the filtered position ends > 3 m short of the raw one;
  F3  with the DEPLOYED value (dtp_params.yaml) the filtered position tracks the raw within
      1.0 m on every frame after the first five (EMA settling);
  F4  with the deployed value the final filtered range is within 0.5 m of the raw range;
  F5  with the deployed value a one-frame 4 m outlier moves the output by <= alpha*max_step,
      and the output is back within 0.3 m of the truth two frames later (the clamp still bounds
      a teleport; the node's accept_range gate is the first line of defence against those);
  F6  perception_node declares `max_step_m` and passes it to the predictor (static check);
  F7  dtp_params.yaml sets it, and the value gives >= 2.4 m/s at the configured detect_rate_hz
      and >= 3.0 m/s at the 10.5 Hz measured in flight (a running person);
  F8  the deployed smooth_alpha trails the raw position by less than the library's 0.25 does
      (the EMA lag is ~(1 - alpha) / alpha frames of the walker's motion);
  F9  the price of that alpha: a standing person's 0.10 m raw jitter is passed at no more than
      sqrt(alpha / (2 - alpha)) of itself (0.38x at 0.25, 0.58x at 0.5).

No ROS graph is needed, so no ROS_DOMAIN_ID. Read-only on the repo except --regenerate, which
rewrites the fixture next to this script. detection_pipeline/ is imported, never edited.
"""
import argparse
import collections
import json
import math
import os
import re
import sys

_CUDSS = os.path.expanduser('~/.local/lib/python3.10/site-packages/nvidia/cu12/lib')
if os.path.isdir(_CUDSS) and _CUDSS not in os.environ.get('LD_LIBRARY_PATH', '') \
        and not os.environ.get('MAX_STEP_CHECK_REEXEC'):
    # detection.py imports ultralytics -> torch, which needs libcudss from this pip-installed
    # directory. The follower launch has it on LD_LIBRARY_PATH; a plain shell does not.
    env = dict(os.environ, MAX_STEP_CHECK_REEXEC='1',
               LD_LIBRARY_PATH=_CUDSS + os.pathsep + os.environ.get('LD_LIBRARY_PATH', ''))
    os.execve(sys.executable, [sys.executable] + sys.argv, env)

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)                       # drone-dtp/
FIXTURE = os.path.join(HERE, 'max_step_fixture_20261002.json')
NODE_PY = os.path.join(REPO, 'ros2_ws', 'src', 'dtp_perception', 'dtp_perception',
                       'perception_node.py')
PARAMS = os.path.join(REPO, 'ros2_ws', 'src', 'dtp_bringup', 'config', 'dtp_params.yaml')
BAG = os.path.join(os.path.dirname(REPO), 'bags', 'stage_b_2026-10-02_200220')
T0_EPOCH = 1790985741.0619128                     # the bag's first message
WINDOW = (42.8, 47.0)                             # bag-relative seconds
FLOWN_RATE_HZ = 10.5                              # /dtp/targets median interval 0.096 s

sys.path.insert(0, REPO)


def run_filter(frames, max_step_m, smooth_alpha=0.25):
    """Feed raw world positions through the real HumanXZPredictor methods. Returns a list of
    (t, x_f, y_f, range_f, range_raw)."""
    from detection_pipeline.detection import HumanXZPredictor
    pred = HumanXZPredictor.__new__(HumanXZPredictor)      # no model load
    pred.smooth_alpha = smooth_alpha
    pred.one_minus_alpha = 1.0 - smooth_alpha
    pred.max_step_m = max_step_m
    pred.max_step_m_sq = max_step_m * max_step_m
    pred.tracks_xz = collections.defaultdict(list)
    out = []
    for f in frames:
        x_raw, y_raw = f['raw_x'], f['raw_y']
        x_c, y_c = pred._limit_motion_step(0, x_raw, y_raw)   # confirmed-track path
        x_s, y_s = pred._smooth_xz(0, x_c, y_c)
        pred.tracks_xz[0].append((x_s, y_s, f['t']))
        out.append((f['t'], x_s, y_s,
                    math.hypot(x_s - f['ac_x'], y_s - f['ac_y']),
                    math.hypot(x_raw - f['ac_x'], y_raw - f['ac_y'])))
    return out


def deployed_values():
    """(max_step_m, smooth_alpha, detect_rate_hz) from the perception_node block of
    dtp_params.yaml; None for any that is not set."""
    import yaml
    doc = yaml.safe_load(open(PARAMS))
    for key, node in doc.items():
        params = (node or {}).get('ros__parameters') if isinstance(node, dict) else None
        if isinstance(params, dict) and 'detect_rate_hz' in params:
            return (params.get('max_step_m'), params.get('smooth_alpha'),
                    params.get('detect_rate_hz'))
    return None, None, None


def regenerate():
    """Run the flown detector on the bag's frames and write the fixture. Needs the GPU, the
    TensorRT engine, rosbag2 and the libcudss path (see the LD_LIBRARY_PATH hint on failure)."""
    import numpy as np
    import cv2
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage, Image
    from nav_msgs.msg import Odometry
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        sys.exit('ultralytics import failed (%s). Try: export LD_LIBRARY_PATH=$HOME/.local/lib/'
                 'python3.10/site-packages/nvidia/cu12/lib:$LD_LIBRARY_PATH' % exc)
    src = open(NODE_PY).read()

    def const(name, default):
        m = re.search(r'^%s\s*=\s*([0-9.]+)' % name, src, re.M)
        return float(m.group(1)) if m else default
    cw, ch = const('PATCH_CENTRE_W_FRAC', 0.5), const('PATCH_CENTRE_H_FRAC', 0.65)
    pct, band = const('NEAR_SURFACE_PCT', 20.0), const('NEAR_SURFACE_BAND_M', 0.45)
    model = YOLO(os.path.expanduser('~/models/yolo11s.engine'))
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
    rd.set_filter(rosbag2_py.StorageFilter(topics=[
        '/camera/color/image_raw/compressed', '/camera/depth/image_raw', '/lidar_slam/odom',
        '/camera/depth/camera_info']))
    depth = None
    K = None
    pose = None
    frames = []
    next_t = WINDOW[0]
    while rd.has_next():
        tp, data, ts = rd.read_next()
        t = ts / 1e9 - T0_EPOCH
        if t > WINDOW[1]:
            break
        if tp == '/camera/depth/camera_info':
            if K is None:
                from sensor_msgs.msg import CameraInfo
                m = deserialize_message(data, CameraInfo)
                K = (m.k[0], m.k[4], m.k[2], m.k[5], m.width, m.height)
            continue
        if tp == '/lidar_slam/odom':
            m = deserialize_message(data, Odometry)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            pose = (p.x, p.y, p.z, yaw)
            continue
        if tp == '/camera/depth/image_raw':
            m = deserialize_message(data, Image)
            depth = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
            depth = depth.astype(np.float32) / 1000.0
            continue
        if t < next_t or depth is None or pose is None or K is None:
            continue
        next_t = t + 1.0 / FLOWN_RATE_HZ * 0.95
        m = deserialize_message(data, CompressedImage)
        img = cv2.imdecode(np.frombuffer(m.data, np.uint8), 1)
        H, W = img.shape[:2]
        r = model.track(img, persist=True, classes=[0], conf=0.25, imgsz=640, verbose=False)[0]
        if r.boxes is None or len(r.boxes) == 0:
            continue
        # the biggest box is the walker in this window (one person in view)
        b = max(r.boxes.xyxy.cpu().numpy(), key=lambda bb: (bb[2] - bb[0]) * (bb[3] - bb[1]))
        x1, y1, x2, y2 = map(int, b)
        bw, bh = max(1, x2 - x1), max(1, y2 - y1)
        cx, cy = int(x1 + cw * bw), int(y1 + ch * bh)
        hw, hh = max(1, int(0.08 * bw)), max(1, int(0.08 * bh))
        fx, fy, cx0, cy0, dw, dh = K
        sx, sy = dw / W, dh / H
        px1, px2 = int(max(0, cx - hw) * sx), int(min(W, cx + hw + 1) * sx)
        py1, py2 = int(max(0, cy - hh) * sy), int(min(H, cy + hh + 1) * sy)
        patch = depth[py1:max(py1 + 1, py2), px1:max(px1 + 1, px2)]
        v = patch[(patch > 0) & (patch <= 10.0)]
        if len(v) < 3:
            continue
        z_near = np.percentile(v, pct)
        z = float(np.median(v[v <= z_near + band]))
        # camera optical: x right, z forward; the camera looks along the body +x
        xc = (cx * sx - cx0) * z / fx
        ac_x, ac_y, _, yaw = pose
        fwd, left = z, -xc
        raw_x = ac_x + fwd * math.cos(yaw) - left * math.sin(yaw)
        raw_y = ac_y + fwd * math.sin(yaw) + left * math.cos(yaw)
        frames.append(dict(t=round(t, 3), ac_x=round(ac_x, 3), ac_y=round(ac_y, 3),
                           yaw_deg=round(math.degrees(yaw), 2), depth_z=round(z, 3),
                           raw_x=round(raw_x, 3), raw_y=round(raw_y, 3),
                           box_h_px=int(bh), n_valid=int(len(v))))
    json.dump(dict(bag=os.path.basename(BAG), t0_epoch=T0_EPOCH, window_s=list(WINDOW),
                   detector='yolo11s.engine imgsz 640 conf 0.25 BoT-SORT', frames=frames),
              open(FIXTURE, 'w'), indent=1)
    print('wrote %s: %d frames, depth %.2f -> %.2f m'
          % (FIXTURE, len(frames), frames[0]['depth_z'], frames[-1]['depth_z']))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--regenerate', action='store_true')
    args = ap.parse_args()
    if args.regenerate:
        regenerate()
        return 0
    fx = json.load(open(FIXTURE))
    frames = fx['frames']
    results = []

    def check(name, ok, detail):
        results.append(ok)
        print('[%s] %s: %s' % ('PASS' if ok else 'FAIL', name, detail))

    def raw_range(f):
        return math.hypot(f['raw_x'] - f['ac_x'], f['raw_y'] - f['ac_y'])
    raw_open = raw_range(frames[-1]) - raw_range(frames[0])
    print('fixture: %d frames over %.1f s, raw range opens by %.1f m'
          % (len(frames), frames[-1]['t'] - frames[0]['t'], raw_open))

    old = run_filter(frames, 0.35)
    rng_old = [r[3] for r in old]
    check('F1 flown value: range flat', max(rng_old) - min(rng_old) < 0.8 and raw_open > 4.0,
          'range %.2f..%.2f m while the raw range opened %.1f m'
          % (min(rng_old), max(rng_old), raw_open))
    short_old = math.hypot(old[-1][1] - frames[-1]['raw_x'], old[-1][2] - frames[-1]['raw_y'])
    check('F2 flown value: position falls behind', short_old > 3.0,
          'ends %.1f m short of the raw position' % short_old)

    dep, alpha, rate = deployed_values()
    if dep is None:
        check('F7 dtp_params.yaml sets max_step_m', False, 'not set under perception_node')
        dep = 0.35
    if alpha is None:
        check('F8 dtp_params.yaml sets smooth_alpha', False, 'not set under perception_node')
        alpha = 0.25
    alpha = float(alpha)
    new = run_filter(frames, dep, alpha)
    free = run_filter(frames, 1e6, alpha)   # the clamp switched off: the EMA alone
    clip = max(math.hypot(a[1] - b[1], a[2] - b[2]) for a, b in zip(new, free))
    check('F3 deployed %.2f: the clamp no longer binds' % dep, clip < 0.15,
          'deployed vs clamp-free filter differ by at most %.2f m over the window '
          '(raw per-frame steps reach %.2f m across detection gaps)'
          % (clip, max(math.hypot(frames[k]['raw_x'] - frames[k - 1]['raw_x'],
                                  frames[k]['raw_y'] - frames[k - 1]['raw_y'])
                       for k in range(1, len(frames)))))
    opened = new[-1][3] - new[0][3]
    check('F4 deployed %.2f: the range follows the walker' % dep,
          opened > 0.8 * raw_open and new[-1][3] > new[-1][4] - 1.5,
          'filtered range opens %.1f m of the raw %.1f m; final %.2f m vs raw %.2f m'
          % (opened, raw_open, new[-1][3], new[-1][4]))

    def trail(rows):
        b = [math.hypot(r[1] - f['raw_x'], r[2] - f['raw_y']) for r, f in zip(rows, frames)][5:]
        return sorted(b)[len(b) // 2], max(b)
    lib_med, lib_max = trail(run_filter(frames, dep, 0.25))
    dep_med, dep_max = trail(new)
    speeds = sorted(math.hypot(frames[k]['raw_x'] - frames[k - 1]['raw_x'],
                               frames[k]['raw_y'] - frames[k - 1]['raw_y'])
                    / max(1e-3, frames[k]['t'] - frames[k - 1]['t'])
                    for k in range(1, len(frames)))
    check('F8 deployed smooth_alpha %.2f: EMA lag' % alpha,
          dep_med <= lib_med + 1e-9 and (alpha == 0.25 or dep_med < 0.6 * lib_med),
          'trails the raw position by %.2f m median / %.2f m max (library 0.25: %.2f / %.2f) '
          'on a walker at %.1f m/s median raw speed'
          % (dep_med, dep_max, lib_med, lib_max, speeds[len(speeds) // 2]))
    # F9: the price -- a standing person with 0.10 m of white depth noise
    import random
    rnd = random.Random(7)
    noisy = [dict(t=i * 0.1, ac_x=0.0, ac_y=0.0, raw_x=4.0 + rnd.gauss(0, 0.10), raw_y=0.0)
             for i in range(400)]
    xs = [r[1] for r in run_filter(noisy, dep, alpha)][20:]
    mean = sum(xs) / len(xs)
    std = math.sqrt(sum((x - mean) ** 2 for x in xs) / len(xs))
    theory = math.sqrt(alpha / (2.0 - alpha))
    check('F9 deployed smooth_alpha %.2f: noise passed' % alpha, std / 0.10 < theory + 0.15,
          'a standing person with 0.10 m raw jitter publishes %.3f m (%.2fx; theory %.2fx, '
          'library 0.25 gives 0.38x)' % (std, std / 0.10, theory))

    # F5: a standing person, one 4 m outlier
    still = [dict(t=i * 0.1, ac_x=0.0, ac_y=0.0, raw_x=4.0, raw_y=0.0) for i in range(12)]
    still[6]['raw_x'] = 8.0
    o = run_filter(still, dep, alpha)
    jump = abs(o[6][1] - o[5][1])
    back = abs(o[8][1] - 4.0)
    check('F5 deployed %.2f / %.2f: one-frame outlier bounded' % (dep, alpha),
          jump <= alpha * dep + 1e-9 and back < 0.3,
          'moved %.3f m on the outlier (bound alpha*max_step %.3f), %.2f m off two frames later'
          % (jump, alpha * dep, back))

    src = open(NODE_PY).read()
    declared = "declare_parameter('max_step_m'" in src
    passed = re.search(r"max_step_m=float\(self\.get_parameter\('max_step_m'\)\.value\)",
                       src) is not None
    declared_a = "declare_parameter('smooth_alpha'" in src
    passed_a = re.search(r"smooth_alpha=float\(self\.get_parameter\('smooth_alpha'\)\.value\)",
                         src) is not None
    check('F6 node declares and passes max_step_m and smooth_alpha',
          declared and passed and declared_a and passed_a,
          'max_step_m declared=%s passed=%s; smooth_alpha declared=%s passed=%s'
          % (declared, passed, declared_a, passed_a))
    if rate:
        v_cfg = alpha * dep * float(rate)
        v_flown = alpha * dep * FLOWN_RATE_HZ
        check('F7 deployed value covers a run', v_cfg >= 2.4 and v_flown >= 3.0,
              'alpha %.2f x max_step_m %.2f -> %.2f m/s at the configured %.1f Hz, '
              '%.2f m/s at the flown %.1f Hz'
              % (alpha, dep, v_cfg, float(rate), v_flown, FLOWN_RATE_HZ))
    n_ok = sum(results)
    print('%d/%d' % (n_ok, len(results)))
    return 0 if n_ok == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
