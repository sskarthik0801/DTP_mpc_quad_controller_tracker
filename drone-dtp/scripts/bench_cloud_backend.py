#!/usr/bin/env python3
"""bench_cloud_backend.py -- is the GPU organized cloud the same cloud, and what does it cost?

Replays real frames from a bag through BOTH backends perception_node can run
(`cloud_backend: numpy | cuda`), using the real depth_source / organized_cloud / cuda_cloud code
and the camera's own factory depth->colour extrinsic read off the bag's /tf_static:

  1. EQUIVALENCE, every frame: finite-mask disagreement and max |diff| of the CUDA cloud against
     the numpy cloud, judged by cuda_cloud.within_camera_tolerance -- the same test perception_node
     applies to its first frames in flight. Exits 1 if any frame is outside it.
  2. COST, per frame: wall ms and PROCESS CPU ms (all threads -- the numpy matmul fans out across
     OpenBLAS threads, so wall time alone hides most of its cost), and % of one core at the
     detection rate.
  3. With --yolo, the rest of the per-frame CPU budget for context: JPEG decode, YOLO predict, and
     track() with the default BoT-SORT (GMC sparseOptFlow) vs ByteTrack.

Touches no camera and no running node. Needs the DTP environment (torch + CUDA):

    source ~/super_ws/src/drone-dtp/ros2_ws/dtp_env_jetson.sh
    python3 scripts/bench_cloud_backend.py ~/dtp_datasets/20260910_flightlegal_bench --frames 160

First run, 2026-09-13 on the drone NX (160 frames of 20260910_flightlegal_bench): see the numbers
recorded in dtp_perception/cuda_cloud.py's docstring and docs/m5-m6-plan.md SS2.5.
"""
import argparse
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src', 'dtp_perception'))

import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

from dtp_perception import cuda_cloud, organized_cloud  # noqa: E402
from dtp_perception.depth_source import CameraDepthSource, quat_to_R  # noqa: E402

COLOUR = '/camera/color/image_raw/compressed'
DEPTH = '/camera/depth/image_raw'
COLOUR_INFO = '/camera/color/camera_info'
DEPTH_INFO = '/camera/depth/camera_info'
COLOUR_FRAME = 'camera_color_optical_frame'


def _storage_id(bag):
    meta = open(os.path.join(bag, 'metadata.yaml')).read()
    return meta.split('storage_identifier:')[1].split()[0]


def _static_tree(msgs):
    """child -> (parent, 4x4) from every /tf_static message seen."""
    tree = {}
    for m in msgs:
        for tf in m.transforms:
            T = np.eye(4)
            T[:3, :3] = quat_to_R(tf.transform.rotation)
            tr = tf.transform.translation
            T[:3, 3] = (tr.x, tr.y, tr.z)
            tree[tf.child_frame_id] = (tf.header.frame_id, T)
    return tree


def _to_root(tree, frame):
    T = np.eye(4)
    chain = [frame]
    while frame in tree:
        parent, T_pc = tree[frame]
        T = T_pc @ T
        frame = parent
        chain.append(frame)
    return frame, T, chain


def colour_from_depth(tree, depth_frame):
    root_c, T_rc, chain_c = _to_root(tree, COLOUR_FRAME)
    root_d, T_rd, chain_d = _to_root(tree, depth_frame)
    if root_c != root_d or len(chain_c) < 2 or len(chain_d) < 2:
        raise SystemExit('no /tf_static chain between %s and %s in this bag (%s | %s)'
                         % (COLOUR_FRAME, depth_frame, ' <- '.join(chain_c),
                            ' <- '.join(chain_d)))
    T_cr = np.eye(4)
    T_cr[:3, :3] = T_rc[:3, :3].T
    T_cr[:3, 3] = -T_rc[:3, :3].T @ T_rc[:3, 3]
    T = T_cr @ T_rd
    return T[:3, :3], T[:3, 3]


def read_bag(bag, n_frames):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=_storage_id(bag)),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    for need in (COLOUR, DEPTH, COLOUR_INFO, DEPTH_INFO):
        if need not in types:
            raise SystemExit('bag has no %s -- it cannot exercise the camera path' % need)
    want = [COLOUR, DEPTH, COLOUR_INFO, DEPTH_INFO] + (['/tf_static'] if '/tf_static' in types
                                                       else [])
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    statics, frames = [], []
    cinfo = dinfo = last_depth = None
    while r.has_next() and len(frames) < n_frames:
        topic, data, _ = r.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == '/tf_static':
            statics.append(msg)
        elif topic == COLOUR_INFO:
            cinfo = msg
        elif topic == DEPTH_INFO:
            dinfo = msg
        elif topic == DEPTH:
            last_depth = msg
        elif last_depth is not None and cinfo is not None and dinfo is not None:
            frames.append((bytes(msg.data), last_depth))
    if not frames:
        raise SystemExit('no colour frame with depth + both camera_infos before it')
    return frames, cinfo, dinfo, statics


def make_source(depth_msg, cinfo, dinfo, R_cd, t_cd, stride, max_range):
    """A real CameraDepthSource with its inputs injected (no ROS node needed to query it)."""
    src = object.__new__(CameraDepthSource)
    src._K = np.array(cinfo.k, dtype=np.float64).reshape(3, 3)
    src._size = (int(cinfo.width), int(cinfo.height))
    src._min_range, src._max_range = 0.3, float(max_range)
    src._buf, src._buf_key, src._buf_stamp = None, None, float('nan')
    src._warned = set()
    src._stride = int(stride)
    src._depth_msg, src._depth_seq = depth_msg, 1
    src._Kd = np.array(dinfo.k, dtype=np.float64).reshape(3, 3)
    src._depth_size = (int(dinfo.width), int(dinfo.height))
    src._R_cd, src._t_cd = R_cd, t_cd
    return src


def timed(fn, items, rate_hz, warm=10, sync=None):
    for x in items[:warm]:
        fn(x)
    wall, cpu = [], []
    for x in items[warm:]:
        t0, c0 = time.perf_counter(), time.process_time()
        fn(x)
        if sync is not None:
            sync()
        wall.append((time.perf_counter() - t0) * 1e3)
        cpu.append((time.process_time() - c0) * 1e3)
    wall, cpu = np.array(wall), np.array(cpu)
    return wall.mean(), np.percentile(wall, 95), cpu.mean(), cpu.mean() * rate_hz / 10.0


def row(label, stats):
    w, w95, c, core = stats
    print('  %-46s wall %6.1f ms (p95 %6.1f)   CPU %6.1f ms = %5.1f %% of a core'
          % (label, w, w95, c, core))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--frames', type=int, default=160)
    ap.add_argument('--rate', type=float, default=8.0, help='detection rate for %% of a core')
    ap.add_argument('--stride', type=int, default=1, help="perception_node's depth_stride")
    ap.add_argument('--max-range', type=float, default=15.0)
    ap.add_argument('--yolo', action='store_true', help='also time decode + YOLO + trackers')
    ap.add_argument('--model', default='~/models/yolo11s.engine')
    args = ap.parse_args()

    frames, cinfo, dinfo, statics = read_bag(os.path.expanduser(args.bag), args.frames)
    depth_frame = frames[0][1].header.frame_id or 'camera_depth_optical_frame'
    tree = _static_tree(statics)
    R_cd, t_cd = colour_from_depth(tree, depth_frame)
    print('frames %d | colour %dx%d | depth %dx%d %s | stride %d | %s <- %s baseline %.4f m'
          % (len(frames), cinfo.width, cinfo.height, dinfo.width, dinfo.height,
             frames[0][1].encoding, args.stride, COLOUR_FRAME, depth_frame,
             float(np.linalg.norm(t_cd))))

    cc = cuda_cloud.CudaCloud()
    cc.warm_up((dinfo.width, dinfo.height), (cinfo.width, cinfo.height))
    import torch
    sources = [make_source(d, cinfo, dinfo, R_cd, t_cd, args.stride, args.max_range)
               for _, d in frames]
    K = sources[0]._K

    print('\nEQUIVALENCE (cuda vs numpy, every frame; tolerance mask %.0e, value %.0e m)'
          % (cuda_cloud.CAMERA_MASK_TOL_FRAC, cuda_cloud.CAMERA_VALUE_TOL_M))
    masks, diffs, bad, finite = [], [], 0, []
    for src in sources:
        ref = organized_cloud.build(src.range_image(), K)
        got = src.organized_cloud_cuda(cc)
        m, d = cuda_cloud.compare(ref, got)
        masks.append(m)
        diffs.append(d)
        finite.append(int(np.isfinite(ref[:, :, 2]).sum()))
        bad += not cuda_cloud.within_camera_tolerance(m, d)
    masks, diffs = np.array(masks), np.array(diffs)
    npx = cinfo.width * cinfo.height
    print('  finite pixels per frame  mean %d' % int(np.mean(finite)))
    print('  mask mismatch            mean %.2e  worst %.2e  (= %d of %d px)  exact frames %d/%d'
          % (masks.mean(), masks.max(), int(round(masks.max() * npx)), npx,
             int(np.sum(masks == 0)), len(masks)))
    print('  max |diff| where finite  mean %.2e m  worst %.2e m' % (diffs.mean(), diffs.max()))
    print('  frames outside tolerance %d' % bad)

    print('\nCOST per frame at %.1f Hz' % args.rate)
    row('numpy: register + range image + build',
        timed(lambda s: organized_cloud.build(s.range_image(), K),
              [make_source(d, cinfo, dinfo, R_cd, t_cd, args.stride, args.max_range)
               for _, d in frames], args.rate))
    row('cuda:  same chain, fresh numpy array out',
        timed(lambda s: s.organized_cloud_cuda(cc), sources, args.rate,
              sync=torch.cuda.synchronize))

    if args.yolo:
        import cv2
        from ultralytics import YOLO
        jpegs = [j for j, _ in frames]
        imgs = [cv2.imdecode(np.frombuffer(j, np.uint8), cv2.IMREAD_COLOR) for j in jpegs]
        row('JPEG decode (cv2.imdecode)',
            timed(lambda j: cv2.imdecode(np.frombuffer(j, np.uint8), cv2.IMREAD_COLOR), jpegs,
                  args.rate))
        kw = dict(classes=[0], conf=0.25, imgsz=640, verbose=False)
        model = os.path.expanduser(args.model)
        m1 = YOLO(model, task='detect')
        row('YOLO predict only', timed(lambda im: m1.predict(im, **kw), imgs, args.rate))
        m2 = YOLO(model, task='detect')
        row('YOLO track, botsort.yaml (GMC sparseOptFlow)',
            timed(lambda im: m2.track(im, persist=True, **kw), imgs, args.rate))
        m3 = YOLO(model, task='detect')
        row('YOLO track, bytetrack.yaml (no GMC)',
            timed(lambda im: m3.track(im, persist=True, tracker='bytetrack.yaml', **kw), imgs,
                  args.rate))

    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
