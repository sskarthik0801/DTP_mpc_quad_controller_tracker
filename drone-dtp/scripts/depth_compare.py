#!/usr/bin/env python3
"""Put the two depth sources in the SAME image and see whether they agree.

THE QUESTION THIS ANSWERS
-------------------------
We can get depth two ways: the Livox projected through our hand-solved extrinsic, or the Gemini
336L's own stereo depth. Before choosing where the camera goes on the airframe -- or whether the
LiDAR is needed for the follower at all -- it helps to look at both rendered into the same colour
frame, at the same scale, with the same colormap.

THE ASYMMETRY THAT MATTERS, AND IT IS EASY TO MISS
--------------------------------------------------
These two projections do NOT have the same failure mode, and the difference is the whole point:

  * **camera depth -> colour** uses the factory depth<->colour extrinsic, read live from `/tf_static`.
    It is baked into the camera body. Move the camera, tape it, remount it, drop it -- this
    transform is unchanged, because both sensors moved together inside one housing.

  * **LiDAR -> colour** uses OUR solved extrinsic from `calib/drone_cam_livox.yaml`. It describes
    one physical mounting of two separate devices. Any change to how either is held invalidates it.

So when the mount is provisional (taped, say), the camera-depth overlay stays correct while the
LiDAR overlay is wrong by however much the camera moved. Disagreement between the two panels is
therefore a measure of MOUNT DRIFT, not of sensor quality. Read it that way.

WHAT TO LOOK AT
---------------
Edges, not coverage. Depth discontinuities must land on image edges -- a doorway, a chair back,
the outline of a person. A projection that is shifted will put its depth step beside the object's
outline rather than on it, consistently, in one direction. That consistent direction is the signal;
scattered noise is not.

The printed agreement statistic compares the two sources only where BOTH have a return. With a
good extrinsic that median lands near zero. A large median with a tight spread means a rigid
offset -- i.e. an extrinsic error, which is fixable by recalibration. A large spread means
something else is wrong.

OUTPUTS
-------
  <prefix>_B_camera_depth.png   raw depth, and depth projected into the colour frame
  <prefix>_C_merged.png         both sources over the same RGB, plus the agreement banner
"""
import argparse
import sys
import time

import numpy as np

# Depth is 16UC1 in millimetres on this camera; ROS convention for 16UC1 depth is mm.
DEPTH_SCALE_M = 0.001
# Ignore returns beyond this: the 336L's stereo depth degrades with range and long tails would
# dominate the colour scale, hiding structure in the near field where following actually happens.
DEFAULT_MAX_RANGE = 12.0
# Subsample the depth image when projecting. 848x480 is 407k points; every 2nd pixel is plenty
# for a visual and keeps the projection interactive.
DEPTH_STRIDE = 2


def _colorize(depth_m, vmin, vmax):
    """Range -> BGR via a perceptually ordered colormap. Invalid stays black."""
    import cv2
    valid = np.isfinite(depth_m) & (depth_m > 0)
    norm = np.zeros(depth_m.shape, np.uint8)
    if valid.any():
        scaled = (depth_m - vmin) / max(vmax - vmin, 1e-6)
        norm[valid] = np.clip(scaled[valid] * 255.0, 0, 255).astype(np.uint8)
    out = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
    out[~valid] = 0
    return out


def _banner(img, lines):
    """Caption block. Rendered onto a darkened strip so it stays readable over any scene."""
    import cv2
    h = 26 * len(lines) + 14
    strip = img[:h].astype(np.float32) * 0.25
    img[:h] = strip.astype(np.uint8)
    for i, text in enumerate(lines):
        cv2.putText(img, text, (12, 26 + i * 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.62, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--prefix', default='/tmp/vis')
    ap.add_argument('--max_range', type=float, default=DEFAULT_MAX_RANGE)
    ap.add_argument('--accumulate', type=float, default=1.0,
                    help='seconds of LiDAR sweeps to stack (the Mid-360 is sparse per sweep)')
    ap.add_argument('--timeout', type=float, default=30.0)
    args = ap.parse_args(argv)

    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    import tf2_ros
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    import sensor_msgs_py.point_cloud2 as pc2

    from dtp_perception import calib_io, image_io

    rclpy.init()
    node = Node('depth_compare')

    # Same gates a real node passes: camera serial, LiDAR source, structural validation. If
    # another robot's LiDAR is on the domain this refuses rather than drawing a plausible lie.
    T_cam_lidar, intr = calib_io.load_for_node(node, None)

    state = {'color': None, 'depth': None, 'dinfo': None, 'cinfo': None, 'sweeps': []}
    topic, ttype = image_io.image_topic_and_type('/camera/color/image_raw', use_compressed=True)
    node.create_subscription(ttype, topic,
                             lambda m: state.__setitem__('color', m), qos_profile_sensor_data)
    node.create_subscription(Image, '/camera/depth/image_raw',
                             lambda m: state.__setitem__('depth', m), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/camera/depth/camera_info',
                             lambda m: state.__setitem__('dinfo', m), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/camera/color/camera_info',
                             lambda m: state.__setitem__('cinfo', m), qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/livox/lidar',
                             lambda m: state['sweeps'].append(m), qos_profile_sensor_data)

    tf_buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buf, node)

    print('listening: %s + /camera/depth/image_raw + /livox/lidar' % topic)
    deadline = time.time() + args.timeout
    started = None
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        have = all(state[k] is not None for k in ('color', 'depth', 'dinfo', 'cinfo'))
        if have and started is None:
            started = time.time()
        if have and (time.time() - started) >= args.accumulate and state['sweeps']:
            break
    missing = [k for k in ('color', 'depth', 'dinfo', 'cinfo') if state[k] is None]
    if missing:
        print('ERROR: never received %s' % ', '.join(missing), file=sys.stderr)
        return 1

    # ---- the two intrinsics -------------------------------------------------------------
    kd = np.array(state['dinfo'].k, float).reshape(3, 3)
    kc = np.array(state['cinfo'].k, float).reshape(3, 3)
    dw, dh = state['dinfo'].width, state['dinfo'].height
    rgb = image_io.decode(state['color'])
    ch, cw = rgb.shape[:2]

    # The colour CameraInfo is published for the streaming resolution; if RGB was decoded at a
    # different size the intrinsics must be scaled or every projection lands in the wrong place.
    if (cw, ch) != (state['cinfo'].width, state['cinfo'].height):
        sx, sy = cw / float(state['cinfo'].width), ch / float(state['cinfo'].height)
        kc = kc * np.array([[sx, 1, sx], [1, sy, sy], [1, 1, 1]])

    # ---- depth -> 3-D in the depth optical frame ----------------------------------------
    depth_raw = np.frombuffer(state['depth'].data, np.uint16).reshape(dh, dw)
    depth_m = depth_raw.astype(np.float32) * DEPTH_SCALE_M
    depth_m[depth_m <= 0] = np.nan

    s = DEPTH_STRIDE
    vv, uu = np.mgrid[0:dh:s, 0:dw:s]
    zz = depth_m[0:dh:s, 0:dw:s]
    ok = np.isfinite(zz) & (zz > 0) & (zz < args.max_range)
    u, v, z = uu[ok].astype(np.float32), vv[ok].astype(np.float32), zz[ok]
    pts_depth = np.stack([(u - kd[0, 2]) * z / kd[0, 0],
                          (v - kd[1, 2]) * z / kd[1, 1],
                          z], axis=1)

    # ---- depth optical -> colour optical, straight from the driver's own TF --------------
    try:
        tf = tf_buf.lookup_transform('camera_color_optical_frame',
                                     state['depth'].header.frame_id or 'camera_depth_optical_frame',
                                     rclpy.time.Time(),
                                     timeout=rclpy.duration.Duration(seconds=4.0))
    except Exception as exc:
        print('ERROR: no TF depth->colour (%s)' % exc, file=sys.stderr)
        return 1
    q = tf.transform.rotation
    t = tf.transform.translation
    x, y, zq, w = q.x, q.y, q.z, q.w
    R_cd = np.array([
        [1 - 2 * (y * y + zq * zq), 2 * (x * y - zq * w),     2 * (x * zq + y * w)],
        [2 * (x * y + zq * w),      1 - 2 * (x * x + zq * zq), 2 * (y * zq - x * w)],
        [2 * (x * zq - y * w),      2 * (y * zq + x * w),      1 - 2 * (x * x + y * y)],
    ])
    pts_c_from_depth = pts_depth @ R_cd.T + np.array([t.x, t.y, t.z])

    # ---- LiDAR -> colour optical, through OUR solved extrinsic ---------------------------
    pts_l = []
    for msg in state['sweeps']:
        arr = np.array([[p[0], p[1], p[2]] for p in
                        pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)],
                       dtype=np.float32)
        if arr.size:
            pts_l.append(arr)
    pts_l = np.vstack(pts_l) if pts_l else np.zeros((0, 3), np.float32)
    T = np.asarray(T_cam_lidar, float)
    pts_c_from_lidar = pts_l @ T[:3, :3].T + T[:3, 3]

    def project(pts):
        z_ = pts[:, 2]
        keep = (z_ > 0.1) & (z_ < args.max_range)
        p = pts[keep]
        uu_ = kc[0, 0] * p[:, 0] / p[:, 2] + kc[0, 2]
        vv_ = kc[1, 1] * p[:, 1] / p[:, 2] + kc[1, 2]
        inb = (uu_ >= 0) & (uu_ < cw) & (vv_ >= 0) & (vv_ < ch)
        return uu_[inb].astype(int), vv_[inb].astype(int), p[:, 2][inb]

    ud, vd, zd = project(pts_c_from_depth)
    ul, vl, zl = project(pts_c_from_lidar)

    vmin, vmax = 0.3, args.max_range

    # ---- agreement, where BOTH sources have a return -------------------------------------
    # Rasterise each into a nearest-return buffer, then compare only cells both filled.
    def raster(uu_, vv_, zz_):
        buf = np.full((ch, cw), np.nan, np.float32)
        order = np.argsort(-zz_)              # nearer wins: write it last
        buf[vv_[order], uu_[order]] = zz_[order]
        return buf
    buf_d, buf_l = raster(ud, vd, zd), raster(ul, vl, zl)
    import cv2 as _cv
    # Dilate the sparse LiDAR raster so 'both present' is not defeated by sub-pixel sparsity.
    mask_l = _cv.dilate((np.isfinite(buf_l)).astype(np.uint8), np.ones((5, 5), np.uint8))
    buf_l_d = _cv.dilate(np.nan_to_num(buf_l, nan=0.0), np.ones((5, 5), np.uint8))
    both = (mask_l > 0) & np.isfinite(buf_d) & (buf_l_d > 0)
    n_both = int(both.sum())
    if n_both:
        diff = buf_l_d[both] - buf_d[both]
        med, p16, p84 = np.median(diff), np.percentile(diff, 16), np.percentile(diff, 84)
    else:
        med = p16 = p84 = float('nan')

    # ---- B: raw depth, and depth projected into the colour frame -------------------------
    raw_vis = _colorize(np.nan_to_num(depth_m, nan=0.0), vmin, vmax)
    raw_vis = cv2.resize(raw_vis, (cw, ch), interpolation=cv2.INTER_NEAREST)
    _banner(raw_vis, ['B1  camera depth, native %dx%d (depth optical frame)' % (dw, dh),
                      'valid %.1f%% of pixels   stereo IR, computed ON-CAMERA'
                      % (100.0 * np.isfinite(depth_m).mean())])

    dep_on_rgb = rgb.copy()
    cvals = _colorize(zd.reshape(1, -1), vmin, vmax).reshape(-1, 3)
    dep_on_rgb[vd, ud] = (0.35 * dep_on_rgb[vd, ud] + 0.65 * cvals).astype(np.uint8)
    _banner(dep_on_rgb, ['B2  camera depth projected into the COLOUR frame',
                         'via factory TF -- UNAFFECTED by how the camera is mounted',
                         '%d points in view' % len(zd)])
    out_b = np.hstack([raw_vis, dep_on_rgb])
    cv2.imwrite('%s_B_camera_depth.png' % args.prefix, out_b)

    # ---- C: both sources on the same RGB --------------------------------------------------
    merged = rgb.copy()
    merged[vd, ud] = (0.55 * merged[vd, ud] + 0.45 * cvals).astype(np.uint8)   # camera: wash
    lvals = _colorize(zl.reshape(1, -1), vmin, vmax).reshape(-1, 3)
    for du in (-1, 0, 1):                                                       # LiDAR: fat dots
        for dv in (-1, 0, 1):
            uu2 = np.clip(ul + du, 0, cw - 1)
            vv2 = np.clip(vl + dv, 0, ch - 1)
            merged[vv2, uu2] = lvals
    _banner(merged, [
        'C  camera depth (wash) + LiDAR (dots), same colormap, 0.3-%.0f m' % args.max_range,
        'camera %d pts   LiDAR %d pts   overlap %d px' % (len(zd), len(zl), n_both),
        'LiDAR minus camera: median %+.3f m   16-84%%  %+.3f .. %+.3f m' % (med, p16, p84),
    ])
    cv2.imwrite('%s_C_merged.png' % args.prefix, merged)

    print('camera depth : %d points in view, %.1f%% of depth pixels valid'
          % (len(zd), 100.0 * np.isfinite(depth_m).mean()))
    print('LiDAR        : %d points in view (from %d sweeps)' % (len(zl), len(state['sweeps'])))
    print('overlap      : %d px where both have a return' % n_both)
    if n_both:
        print('LiDAR - camera depth: median %+.3f m,  16-84%% spread %+.3f .. %+.3f m'
              % (med, p16, p84))
        print('  A large median with a TIGHT spread = rigid extrinsic offset (recalibrate).')
        print('  A large SPREAD = not a simple offset; look at the images before concluding.')
    print('wrote %s_B_camera_depth.png and %s_C_merged.png' % (args.prefix, args.prefix))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
