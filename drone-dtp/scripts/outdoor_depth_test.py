#!/usr/bin/env python3
"""Does the 336L still give us depth OUTDOORS, in sunlight? This decides the follower architecture.

WHY THIS IS THE DECIDING TEST
-----------------------------
Indoors on 2026-09-08 the camera returned depth on 88.4 % of pixels at 30 Hz, against 13.5 % frame
coverage from the LiDAR, and it did so with NO calibration of ours -- the depth<->colour extrinsic
is factory and survives any remount. On that evidence the follower should take both RGB and depth
from the camera and leave the LiDAR to FAST-LIO and ROG-Map.

But every one of those numbers came from a room. The 336L is an ACTIVE stereo camera: an IR dot
projector paints texture on the scene and two IR cameras triangulate it. Direct sunlight contains
far more IR power than that projector, so outdoors the dots wash out and the camera falls back on
whatever natural texture exists. That is a different and much weaker regime.

If depth survives outdoors, the follower is camera-only and camera placement becomes a purely
mechanical question. If it collapses, the LiDAR path is mandatory and every remount costs a
recalibration. Nothing else we could measure this week changes the design as much.

WHY IT MEASURES DEPTH ON THE PERSON, NOT THE FRAME
--------------------------------------------------
Frame-wide validity is the wrong number. A sunlit scene of tarmac and foliage can score well
overall and still return nothing on a human at 3 m -- and the human is the entire point. So this
runs YOLO, finds people, and reports the depth return rate INSIDE the person box along with the
range it reports. A follower needs range-to-target, not a nice-looking depth map.

WHY IT LOGS THE IR IMAGES TOO
-----------------------------
The IR pair is the explanatory variable. If depth degrades, saturation in the IR cameras says why:
sunlight swamping the sensor. Without it a bad result is just a bad result with no cause, and we
would not know whether shading the camera, changing exposure, or a different sensor is the fix.
Enabling IR costs depth rate (30 -> ~20 Hz on USB) but does NOT affect depth VALIDITY, which is
what is being measured here.

IT RUNS UNATTENDED
------------------
WiFi will not follow the aircraft outside, so this detaches, writes a CSV plus periodic annotated
snapshots, and survives the SSH session dying. Start it, carry the aircraft out, walk in front of
it at a few distances in both sun and shade, bring it back, then collect the results.

    python3 outdoor_depth_test.py --minutes 12      # -> ~/dtp_datasets/<date>_outdoor_depth
    # needs the camera with its IR pair: ros2 launch dtp_bringup sensors.launch.py lidar:=false enable_ir:=true

Indoor and outdoor samples are separated automatically by IR saturation, so precise timing notes
are not needed -- though a rough log of what you did helps interpret the edge cases.
"""
import argparse
import os
import sys
import time

import numpy as np

DEPTH_SCALE_M = 0.001
PERSON_CLASS = 0
DEPTH_STRIDE = 2
# Follow standoff band -- the ranges the controller actually cares about.
STANDOFF_LO, STANDOFF_HI = 1.5, 6.0
SATURATION_LEVEL = 250
# *** THE SATURATED FRACTION DOES NOT DISCRIMINATE SUN FROM SHADE. MEASURED 2026-09-11. ***
#
# The original threshold here was 2 % of pixels at full scale, on the reasoning that indoors
# it would be ~0. It is not: measured in a dark room at night, the saturated fraction was
# 21.7 % on the left IR camera and 42.5 % on the right -- ten to twenty times OVER the
# threshold -- so every indoor sample was classified SUNLIT. Outdoors everything would also
# have been classified sunlit, and the sun-versus-shade comparison this test exists to make
# would have produced one bucket and no comparison.
#
# The cause is the sensor, not the code: this is an ACTIVE stereo camera. Its own IR projector
# paints bright dots on the scene, and those dots saturate pixels regardless of ambient light.
# So the saturated fraction largely measures the PROJECTOR.
#
# What sunlight changes, and the projector does not, is the FLOOR BETWEEN the dots. Indoors
# that floor is dark; in direct sun the whole field is lit, so the floor rises. A low
# percentile of the IR image therefore separates the two where the saturated fraction cannot.
IR_FLOOR_PCT = 10           # the percentile taken as 'the floor between the dots'
# PROVISIONAL, and it has to be: calibrating it needs an outdoor sample, which is the thing
# this test exists to collect. The only anchor available is the measured indoor floor.
#
# MEASURED 2026-09-11 over four indoor runs, 848x480 IR, and the spread is the point:
#     empty room, camera on far walls : floor L = 28, R = 53
#     person at 2 / 4 / 6 m           : floor L = 12, R =  9
# The floor moves by ~6x with the SCENE, because the IR cameras auto-expose. The L/R
# asymmetry is not a fixed sensor property either -- it REVERSED between those two
# conditions (R brighter by 25 counts in one, L brighter by 3 in the other), so an earlier
# version of this comment claiming the asymmetry was a property of the right camera was
# simply wrong.
#
# What survives: every indoor floor observed is <= 53, and 90 clears the worst of them by
# 37 counts. That is enough for the live flag to avoid calling a room sunlit, and it is NOT
# enough to trust the flag as data, because an absolute threshold on an auto-exposed signal
# is fragile by construction. Hence: the flag is a hint, the logged L_floor/R_floor columns
# are the evidence, and the operator's notes are what actually separate sun from shade.
#
# It is still a guess at the upper end, which is why the OFFLINE split on the logged
# L_floor/R_floor columns is authoritative and the live flag is only a hint. Re-fit this
# after the first outdoor run, record the outdoor floor here, and drop the word provisional.
SUNLIT_IR_FLOOR = 90.0
# Measured 2026-09-11, dark room at night, 848x480 IR, projector on. Printed at start-up so
# an operator can see what the live classification is being judged against.
INDOOR_BASELINE = 'floor 9-53 of 255 across four indoor runs (auto-exposure dependent)'


def _ensure_torch_libs():
    """dlopen CUDA libs that pip's nvidia wheels hide from the loader. See bench_yolo.py."""
    import ctypes
    import glob
    try:
        ctypes.CDLL('libcudss.so.0')
        return
    except OSError:
        pass
    roots = glob.glob(os.path.join(os.path.dirname(os.__file__), 'site-packages',
                                   'nvidia', '*', 'lib'))
    roots += glob.glob(os.path.expanduser('~/.local/lib/python3.*/site-packages/nvidia/*/lib'))
    for d in roots:
        for so in sorted(glob.glob(os.path.join(d, 'libcudss*.so*'))):
            try:
                ctypes.CDLL(so, mode=ctypes.RTLD_GLOBAL)
            except OSError:
                pass


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--minutes', type=float, default=12.0)
    # NOT /tmp: it is cleared on every reboot and has eaten a recording on this project already
    # (docs/m5-m6-plan.md M5.4 asked for this path).
    ap.add_argument('--out_dir', default=os.path.expanduser(
        '~/dtp_datasets/%s_outdoor_depth' % time.strftime('%Y%m%d')))
    ap.add_argument('--model', default=os.path.expanduser('~/models/yolo11s.engine'))
    ap.add_argument('--snapshot_every', type=float, default=20.0)
    ap.add_argument('--sample_every', type=float, default=1.0)
    ap.add_argument('--no_detect', action='store_true')
    args = ap.parse_args(argv)

    os.makedirs(args.out_dir, exist_ok=True)
    _ensure_torch_libs()

    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    import tf2_ros
    from sensor_msgs.msg import CameraInfo, Image

    from dtp_perception import calib_io, image_io      # noqa: F401  (gates + decode)

    model = None
    if not args.no_detect:
        try:
            from ultralytics import YOLO
            model = YOLO(args.model, task='detect')
            print('YOLO loaded: %s' % args.model, flush=True)
        except Exception as exc:
            print('WARNING: YOLO unavailable (%s) -- frame stats only, no person metric' % exc,
                  file=sys.stderr, flush=True)

    rclpy.init()
    node = Node('outdoor_depth_test')
    S = {}

    def mk(k):
        def cb(m):
            S[k] = m
        return cb

    topic, ttype = image_io.image_topic_and_type('/camera/color/image_raw', use_compressed=True)
    node.create_subscription(ttype, topic, mk('color'), qos_profile_sensor_data)
    node.create_subscription(Image, '/camera/depth/image_raw', mk('depth'), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/camera/depth/camera_info', mk('dinfo'),
                             qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/camera/color/camera_info', mk('cinfo'),
                             qos_profile_sensor_data)
    node.create_subscription(Image, '/camera/left_ir/image_raw', mk('L'), qos_profile_sensor_data)
    node.create_subscription(Image, '/camera/right_ir/image_raw', mk('R'), qos_profile_sensor_data)

    tf_buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buf, node)

    print('waiting for streams ...', flush=True)
    t0 = time.time()
    while time.time() - t0 < 30 and not all(k in S for k in ('color', 'depth', 'dinfo', 'cinfo')):
        rclpy.spin_once(node, timeout_sec=0.1)
    missing = [k for k in ('color', 'depth', 'dinfo', 'cinfo') if k not in S]
    if missing:
        print('ERROR: never received %s' % missing, file=sys.stderr)
        return 1
    # IR arrives a little after colour/depth, so give it its own short wait rather than latching
    # "absent" the instant the other four are up.
    t_ir = time.time()
    while time.time() - t_ir < 8.0 and not ('L' in S and 'R' in S):
        rclpy.spin_once(node, timeout_sec=0.1)
    if not ('L' in S and 'R' in S):
        print('NOTE: IR streams absent -- sun/shade classification will not work. Relaunch with '
              'enable_left_ir:=true enable_right_ir:=true before going outside.', flush=True)
    else:
        print('IR streams present -- sun/shade classification armed', flush=True)
        print('  classifying on the IR FLOOR (p%d) > %.0f of 255. PROVISIONAL: the threshold'
              % (IR_FLOOR_PCT, SUNLIT_IR_FLOOR), flush=True)
        print('  has never seen an outdoor sample. Indoor baseline: %s' % INDOOR_BASELINE,
              flush=True)
        print('  *** KEEP ROUGH NOTES OF WHEN YOU ARE IN SUN VS SHADE. *** The offline split',
              flush=True)
        print('  on L_floor/R_floor in the CSV is authoritative; this live one is a hint.',
              flush=True)

    kd = np.array(S['dinfo'].k, float).reshape(3, 3)
    kc = np.array(S['cinfo'].k, float).reshape(3, 3)
    dw, dh = S['dinfo'].width, S['dinfo'].height

    # lookup_transform's OWN timeout does not spin this node, so /tf_static is never processed
    # and the call fails with "frame does not exist" -- which is exactly what happened on the
    # first dry run, silently disabling the person metric, i.e. the whole point of the test.
    # Spin explicitly while waiting for the transform to become available.
    R_cd = t_cd = None
    tgt = 'camera_color_optical_frame'
    src = S['depth'].header.frame_id or 'camera_depth_optical_frame'
    deadline = time.time() + 12.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            if tf_buf.can_transform(tgt, src, rclpy.time.Time()):
                break
        except Exception:
            pass
    try:
        tf = tf_buf.lookup_transform(tgt, src, rclpy.time.Time())
        R_cd = quat_to_R(tf.transform.rotation)
        t_cd = np.array([tf.transform.translation.x, tf.transform.translation.y,
                         tf.transform.translation.z])
        print('depth->colour TF acquired (%s -> %s)' % (src, tgt), flush=True)
    except Exception as exc:
        print('WARNING: no depth->colour TF (%s); ROI metrics disabled -- DO NOT run the '
              'outdoor test until this is fixed, the person column is the result' % exc,
              flush=True)

    csv_path = os.path.join(args.out_dir, 'outdoor_depth.csv')
    fh = open(csv_path, 'w')
    fh.write('t_s,depth_valid_pct,standoff_valid_pct,median_range_m,'
             'L_mean,L_std,L_sat_pct,L_floor,R_mean,R_std,R_sat_pct,R_floor,'
             'n_person,person_depth_pct,person_range_m,centre_depth_pct,sunlit\n')
    fh.flush()

    print('logging %.0f min -> %s' % (args.minutes, csv_path), flush=True)
    end = time.time() + args.minutes * 60.0
    nxt_sample = 0.0
    nxt_snap = 0.0
    n_snap = 0
    rows = []

    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time()
        if now < nxt_sample:
            continue
        nxt_sample = now + args.sample_every
        el = now - t0

        d = S.get('depth')
        if d is None:
            continue
        draw = np.frombuffer(d.data, np.uint16).reshape(dh, dw)
        dm = draw.astype(np.float32) * DEPTH_SCALE_M
        valid = (draw > 0) & (draw < 65535)
        depth_pct = 100.0 * valid.mean()
        band = valid & (dm >= STANDOFF_LO) & (dm <= STANDOFF_HI)
        band_pct = 100.0 * band.mean()
        med_range = float(np.median(dm[valid])) if valid.any() else float('nan')

        irs = {}
        for k in ('L', 'R'):
            m = S.get(k)
            if m is None:
                irs[k] = (float('nan'),) * 4
                continue
            g = np.frombuffer(m.data, np.uint8)
            irs[k] = (float(g.mean()), float(g.std()),
                      100.0 * float((g >= SATURATION_LEVEL).mean()),
                      float(np.percentile(g, IR_FLOOR_PCT)))
        # Decide from the CURRENT sample, not from a flag latched before the IR streams had
        # arrived -- that bug pinned `sunlit` to 0 for a whole run, which would have silently
        # collapsed the sun-versus-shade comparison this test exists to make.
        # Classify on the FLOOR, not the saturated fraction -- see the constants above.
        floors = [irs[k][3] for k in ('L', 'R') if np.isfinite(irs[k][3])]
        sunlit = int(max(floors) > SUNLIT_IR_FLOOR) if floors else 0

        n_person, person_pct, person_rng = 0, float('nan'), float('nan')
        rgb = None
        boxes = []
        if model is not None and 'color' in S:
            try:
                rgb = image_io.decode(S['color'])
                res = model.predict(rgb, verbose=False, conf=0.35)[0]
                for b in res.boxes:
                    if int(b.cls[0]) != PERSON_CLASS:
                        continue
                    boxes.append([int(v) for v in b.xyxy[0].tolist()])
                n_person = len(boxes)
            except Exception:
                pass

        # A centre box stands in for a person-sized ROI when nobody is in frame. It keeps the
        # projection math exercised on every run (the dry run had no people, so without this the
        # ROI path would go untested until it mattered), and it gives a comparable
        # sun-versus-shade number even from footage with no human in it.
        centre_pct = float('nan')
        cw_, ch_ = (S['cinfo'].width, S['cinfo'].height)
        bw, bh = cw_ // 6, ch_ // 3
        centre_box = [cw_ // 2 - bw // 2, ch_ // 2 - bh // 2,
                      cw_ // 2 + bw // 2, ch_ // 2 + bh // 2]

        if R_cd is not None:
            s = DEPTH_STRIDE
            vv, uu = np.mgrid[0:dh:s, 0:dw:s]
            zz = dm[0:dh:s, 0:dw:s]
            ok = valid[0:dh:s, 0:dw:s] & (zz > 0)
            u, v, z = uu[ok].astype(np.float32), vv[ok].astype(np.float32), zz[ok]
            pts = np.stack([(u - kd[0, 2]) * z / kd[0, 0],
                            (v - kd[1, 2]) * z / kd[1, 1], z], axis=1)
            pc = pts @ R_cd.T + t_cd
            good = pc[:, 2] > 0.1
            pc = pc[good]
            cu = (kc[0, 0] * pc[:, 0] / pc[:, 2] + kc[0, 2]).astype(int)
            cv_ = (kc[1, 1] * pc[:, 1] / pc[:, 2] + kc[1, 2]).astype(int)
            cz = pc[:, 2]

            def eval_box(box):
                x1, y1, x2, y2 = box
                inb = (cu >= x1) & (cu < x2) & (cv_ >= y1) & (cv_ < y2)
                n_in = int(inb.sum())
                # Approximate how many DEPTH pixels look at this box: scale the colour-frame area
                # by the focal-length ratio, then by the subsample stride. Approximate on purpose
                # -- a coverage indicator, not a calibrated quantity.
                area_c = max((x2 - x1) * (y2 - y1), 1)
                expect = area_c * (kd[0, 0] / kc[0, 0]) ** 2 / (s * s)
                return (min(100.0, 100.0 * n_in / max(expect, 1.0)),
                        float(np.median(cz[inb])) if n_in else float('nan'))

            centre_pct = eval_box(centre_box)[0]
            best = None
            for box in boxes:
                pct, rng = eval_box(box)
                if best is None or pct > best[0]:
                    best = (pct, rng)
            if best:
                person_pct, person_rng = best

        rows.append((el, depth_pct, band_pct, person_pct, sunlit, n_person, centre_pct))
        fh.write('%.1f,%.2f,%.2f,%.3f,'
                 '%.1f,%.1f,%.3f,%.1f,'
                 '%.1f,%.1f,%.3f,%.1f,'
                 '%d,%.2f,%.3f,%.2f,%d\n'
                 % (el, depth_pct, band_pct, med_range,
                    irs['L'][0], irs['L'][1], irs['L'][2], irs['L'][3],
                    irs['R'][0], irs['R'][1], irs['R'][2], irs['R'][3],
                    n_person, person_pct, person_rng, centre_pct, sunlit))
        fh.flush()

        if now >= nxt_snap:
            nxt_snap = now + args.snapshot_every
            try:
                if rgb is None and 'color' in S:
                    rgb = image_io.decode(S['color'])
                if rgb is not None:
                    dv = np.zeros(dm.shape, np.uint8)
                    dv[valid] = np.clip((dm[valid] - 0.3) / 11.7 * 255, 0, 255).astype(np.uint8)
                    dvis = cv2.applyColorMap(dv, cv2.COLORMAP_TURBO)
                    dvis[~valid] = 0
                    dvis = cv2.resize(dvis, (rgb.shape[1], rgb.shape[0]),
                                      interpolation=cv2.INTER_NEAREST)
                    shot = rgb.copy()
                    for (x1, y1, x2, y2) in boxes:
                        cv2.rectangle(shot, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    panel = np.hstack([shot, dvis])
                    txt = ('t=%.0fs  depth %.1f%%  standoff-band %.1f%%  person %.0f%% @ %.2fm  '
                           'IRsat L%.2f%% R%.2f%%  %s'
                           % (el, depth_pct, band_pct, person_pct, person_rng,
                              irs['L'][2], irs['R'][2], 'SUNLIT' if sunlit else 'shade/indoor'))
                    cv2.rectangle(panel, (0, 0), (panel.shape[1], 34), (0, 0, 0), -1)
                    cv2.putText(panel, txt, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (255, 255, 255), 1, cv2.LINE_AA)
                    cv2.imwrite(os.path.join(args.out_dir, 'snap_%03d.png' % n_snap), panel)
                    n_snap += 1
            except Exception:
                pass

    fh.close()

    # ---- verdict ------------------------------------------------------------------------
    print()
    print('=' * 72)
    print('OUTDOOR DEPTH TEST -- %d samples, %d snapshots -> %s'
          % (len(rows), n_snap, args.out_dir))
    print('=' * 72)
    arr_sun = [r for r in rows if r[4] == 1]
    arr_shade = [r for r in rows if r[4] == 0]

    def summarise(name, group):
        if not group:
            print('  %-16s (no samples)' % name)
            return
        dp = np.array([g[1] for g in group])
        bp = np.array([g[2] for g in group])
        pp = np.array([g[3] for g in group if np.isfinite(g[3])])
        cc = np.array([g[6] for g in group if np.isfinite(g[6])])
        print('  %-16s n=%4d   frame %.1f%%   standoff band %.1f%%   centre-ROI %s   person %s'
              % (name, len(group), dp.mean(), bp.mean(),
                 ('%.0f%%' % cc.mean()) if cc.size else '--',
                 ('%.0f%% (n=%d)' % (pp.mean(), pp.size)) if pp.size else 'no detections'))

    summarise('SUNLIT', arr_sun)
    summarise('shade/indoor', arr_shade)
    print()
    if arr_sun and arr_shade:
        ds = np.mean([g[1] for g in arr_sun])
        dh_ = np.mean([g[1] for g in arr_shade])
        print('  frame depth in sun is %.0f%% of the shade figure.' % (100.0 * ds / max(dh_, 1e-6)))
        print('  READ THE PERSON COLUMN, NOT THE FRAME COLUMN -- a sunlit scene can score well')
        print('  overall and still return nothing on a human at follow range.')
    else:
        print('  Only one lighting condition was seen. If the aircraft did go into direct sun,')
        print('  IR saturation never rose, which itself is worth knowing -- check the snapshots.')
    print('=' * 72)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
