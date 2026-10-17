#!/usr/bin/env python3
"""Project the LIVE LiDAR into the LIVE camera frame, right now, with the deployed calibration.

WHAT THIS IS FOR
----------------
`validate_calib` works on a captured calibration run. This answers a different, more immediate
question: with the sensors where they are at this moment, does the extrinsic we are actually
shipping put LiDAR points where they belong in the image?

It loads the calibration through `dtp_perception.calib_io`, so the same gates a real node passes
apply here -- structural validation, provenance logging, and the camera-serial check. If the
wrong camera body is connected this refuses to run, exactly as `fusion_node` would.

HOW TO READ THE OUTPUT
----------------------
Colour is LiDAR range. What matters is not coverage but EDGES: depth discontinuities must land
on image edges -- a door frame, a table lip, the outline of a person. Points that stop short of
an object's boundary, or spill past it, are the signature of a bad extrinsic.

Expect roughly 12 % of points in view on this rig and no points in the lower third of the frame.
Neither is a fault: the Mid-360 sweeps 360x59 deg against the camera's ~86x55 deg, and its
vertical FOV floor is -7 deg, so it simply cannot see the ground close in.

This is a QUALITATIVE check. For a number, use `score_alignment` on a capture run -- it sweeps
pixel offsets and requires the alignment score to peak at zero.
"""
import argparse
import sys
import time


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--lidar_topic', default='/livox/lidar')
    ap.add_argument('--image_topic', default='/camera/color/image_raw')
    ap.add_argument('--raw', action='store_true', help='subscribe to raw instead of compressed')
    ap.add_argument('--out', default='/tmp/live_overlay.png')
    ap.add_argument('--accumulate', type=float, default=1.0,
                    help='seconds of LiDAR to accumulate. A single Mid-360 sweep is sparse on a '
                         'person; a short window is what fusion_node will do anyway.')
    ap.add_argument('--max_range', type=float, default=15.0)
    ap.add_argument('--timeout', type=float, default=25.0)
    args = ap.parse_args(argv)

    try:
        import numpy as np
        import matplotlib
        matplotlib.use('Agg')          # headless: no DISPLAY on the aircraft
        import matplotlib.pyplot as plt
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py import point_cloud2
        from dtp_perception import calib_io, image_io
    except ImportError as e:
        print("ERROR: %s\n  Source dtp_env_jetson.sh and the workspace." % e, file=sys.stderr)
        return 2

    rclpy.init()
    node = Node('live_overlay')

    # Same load path a real node uses: structural checks, provenance, camera-serial gate.
    T, intr = calib_io.load_for_node(node, None)
    fx, fy, cx, cy = intr['fx'], intr['fy'], intr['cx'], intr['cy']

    img_topic, img_type = image_io.image_topic_and_type(args.image_topic, use_compressed=not args.raw)
    state = {'img': None, 'clouds': []}
    node.create_subscription(img_type, img_topic,
                             lambda m: state.__setitem__('img', m), qos_profile_sensor_data)
    node.create_subscription(PointCloud2, args.lidar_topic,
                             lambda m: state['clouds'].append(m), qos_profile_sensor_data)
    print("listening: %s  +  %s" % (img_topic, args.lidar_topic))

    t0 = time.time()
    while time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if state['img'] is not None and state['clouds'] and (time.time() - t0) > args.accumulate:
            break

    if state['img'] is None:
        print("ERROR: no image on %s" % img_topic, file=sys.stderr)
        return 1
    if not state['clouds']:
        print("ERROR: no cloud on %s" % args.lidar_topic, file=sys.stderr)
        return 1

    img = image_io.decode(state['img'])          # BGR
    pts = []
    for c in state['clouds']:
        # read_points, not read_points_numpy: a Livox cloud mixes datatypes (float32 xyz
        # alongside uint8 intensity/tag/line) and the numpy variant asserts they are uniform.
        arr = point_cloud2.read_points(c, field_names=('x', 'y', 'z'), skip_nans=True)
        if arr is None or len(arr) == 0:
            continue
        a = np.asarray(arr)
        if a.dtype.names:                      # structured -> plain Nx3
            a = np.stack([a['x'], a['y'], a['z']], axis=-1)
        pts.append(np.asarray(a, dtype=float).reshape(-1, 3))
    node.destroy_node()
    rclpy.shutdown()

    P = np.vstack(pts)
    print("accumulated %d sweeps -> %d points" % (len(state['clouds']), len(P)))

    cam = (T[:3, :3] @ P.T).T + T[:3, 3]
    z = cam[:, 2]
    keep = z > 0.3
    cam, z = cam[keep], z[keep]
    u = fx * cam[:, 0] / z + cx
    v = fy * cam[:, 1] / z + cy
    rng = np.linalg.norm(cam, axis=1)

    H, W = img.shape[:2]
    inview = (u >= 0) & (u < W) & (v >= 0) & (v < H) & (rng < args.max_range)
    n_in = int(inview.sum())
    print("in view: %d / %d points (%.1f%%)  -- ~12%% is EXPECTED on this rig, not a fault"
          % (n_in, len(P), 100.0 * n_in / max(len(P), 1)))
    if n_in == 0:
        print("\n*** NOTHING PROJECTS INTO THE IMAGE. Either the extrinsic is grossly wrong, or")
        print("*** this LiDAR topic is not the body the calibration belongs to.", file=sys.stderr)
        return 1

    fig, ax = plt.subplots(figsize=(13, 7.5))
    ax.imshow(img[:, :, ::-1])                    # BGR -> RGB for display
    s = ax.scatter(u[inview], v[inview], c=rng[inview], s=2.0, cmap='turbo', alpha=0.65)
    plt.colorbar(s, ax=ax, label='LiDAR range (m)')
    ax.set_title('LIVE LiDAR -> camera projection   |   %d pts in view (%.1f%%)   |   %s'
                 % (n_in, 100.0 * n_in / len(P), time.strftime('%H:%M:%S')))
    ax.axis('off')
    fig.savefig(args.out, dpi=110, bbox_inches='tight')
    print("wrote %s" % args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
