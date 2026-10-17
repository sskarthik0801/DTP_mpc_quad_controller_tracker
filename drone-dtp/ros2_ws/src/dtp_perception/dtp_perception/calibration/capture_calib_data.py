"""capture_calib_data -- record synchronised image + point-cloud pairs for extrinsic calibration.

Stage 1 of the reusable calibration workflow (see calibration/__init__.py). Solver-agnostic:
it writes a plain, self-describing directory that any solver can consume.

Run it, then move the rig to a new viewpoint and press ENTER for each capture. Most targetless
solvers want 5-10 well-separated viewpoints with visible depth structure (walls at different
ranges, box edges, doorways) -- a set of near-identical views is what produces a confident but
WRONG extrinsic, so the tool nags about pose diversity rather than silently accepting it.

Also records camera_info (factory intrinsics) alongside, so the calibration set is
self-contained and reproducible months later on a different machine.

Usage:
    ros2 run dtp_perception capture_calib_data --ros-args \
        -p out_dir:=~/calib_runs/$(date +%Y%m%d_%H%M) -p target_pairs:=8
"""
import glob
import os
import shutil
import signal
import sqlite3
import struct
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract Nx3 float32 XYZ, honouring the actual field offsets rather than assuming
    a packed x,y,z layout -- Livox CustomMsg-derived clouds carry intensity/tag/line too."""
    off = {f.name: f.offset for f in msg.fields}
    if not {'x', 'y', 'z'} <= set(off):
        return np.empty((0, 3), dtype=np.float32)
    n = msg.width * msg.height
    buf = bytes(msg.data)
    step = msg.point_step
    out = np.empty((n, 3), dtype=np.float32)
    for i in range(n):
        base = i * step
        out[i, 0] = struct.unpack_from('<f', buf, base + off['x'])[0]
        out[i, 1] = struct.unpack_from('<f', buf, base + off['y'])[0]
        out[i, 2] = struct.unpack_from('<f', buf, base + off['z'])[0]
    return out


class CaptureNode(Node):
    def __init__(self):
        super().__init__('capture_calib_data')
        self.declare_parameter('out_dir', os.path.expanduser('~/calib_runs/latest'))
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('info_topic', '/camera/color/camera_info')
        self.declare_parameter('cloud_topic', '/livox/lidar')
        # The Gemini 336L computes depth on its own ASIC, so capturing one depth frame per
        # viewpoint costs ~0.8 MB and no meaningful CPU. It buys an INDEPENDENT numeric check:
        # projected LiDAR range vs the camera's own depth at the same pixels, in metres. Without
        # it, adding that check later means physically redoing every viewpoint.
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        # Accumulate several sweeps per capture: a single Mid-360 sweep is sparse
        # (non-repetitive scan), and solvers do much better with a denser static cloud.
        self.declare_parameter('sweeps_per_capture', 10)
        self.declare_parameter('target_pairs', 8)
        # Optional rosbag2 recording per viewpoint, for solvers that consume bags rather than
        # our .npz (koide3/direct_visual_lidar_calibration wants one bag per viewpoint with the
        # sensor STATIONARY for 10-15 s). 0 disables. Recording runs concurrently with the .npz
        # accumulation, so a single dwell feeds both our validator and the solver.
        self.declare_parameter('bag_secs', 0)

        g = lambda k: self.get_parameter(k).value
        self.out_dir = os.path.expanduser(g('out_dir'))
        self.sweeps = int(g('sweeps_per_capture'))
        self.target_pairs = int(g('target_pairs'))
        self.bag_secs = int(g('bag_secs'))
        self.topics = [g('image_topic'), g('info_topic'), g('cloud_topic')]
        os.makedirs(self.out_dir, exist_ok=True)

        self.image = None
        self.info = None
        self.depth = None
        self.clouds = []
        self.collecting = False

        self.create_subscription(Image, g('image_topic'), self._on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, g('info_topic'), self._on_info, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, g('cloud_topic'), self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(Image, g('depth_topic'), self._on_depth, qos_profile_sensor_data)

    def _on_image(self, msg):
        self.image = msg

    def _on_info(self, msg):
        self.info = msg

    def _on_depth(self, msg):
        self.depth = msg

    def _on_cloud(self, msg):
        if self.collecting:
            self.clouds.append(cloud_to_xyz(msg))

    # -- helpers -------------------------------------------------------------
    def wait_for_streams(self, timeout=20.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.image is not None and self.info is not None:
                return True
        return False

    def save_intrinsics(self):
        m = self.info
        path = os.path.join(self.out_dir, 'camera_info.yaml')
        with open(path, 'w') as f:
            f.write("# Factory intrinsics captured live from %s\n" %
                    self.get_parameter('info_topic').value)
            f.write("resolution: [%d, %d]\n" % (m.width, m.height))
            f.write("fx: %.6f\nfy: %.6f\ncx: %.6f\ncy: %.6f\n" %
                    (m.k[0], m.k[4], m.k[2], m.k[5]))
            f.write("distortion_model: %s\n" % m.distortion_model)
            f.write("d: [%s]\n" % ", ".join("%.8f" % v for v in m.d))
        return path

    def _start_bag(self, idx):
        """Record a rosbag2 for this viewpoint. Shells out to `ros2 bag record` rather than
        using rosbag2_py directly: the CLI already gets QoS negotiation and metadata.yaml right,
        and a malformed bag fails at solve time -- long after the rig has been moved."""
        if self.bag_secs <= 0:
            return None
        bag_dir = os.path.join(self.out_dir, 'bags', 'pair_%03d' % idx)
        os.makedirs(os.path.dirname(bag_dir), exist_ok=True)
        return subprocess.Popen(['ros2', 'bag', 'record', '-o', bag_dir] + self.topics,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _verify_bag(self, idx):
        """Refuse a viewpoint whose bag did not actually record the LiDAR.

        *** THIS EXISTS BECAUSE SILENCE HERE COSTS THE WHOLE CAPTURE. ***

        MEASURED 2026-09-11: an 8-viewpoint run produced THREE bags containing no
        /livox/lidar topic at all -- only camera. Nothing complained. The operator moved the
        rig eight times, 8.4 GB was transferred to the workstation, koide3 preprocess wrote
        three 99-byte header-only .ply files, and `calibrate` then died on them:

            use automatically estimated initial guess
            nannannan nan nan
            line_search_minimizer.cc:468 Terminating: Initial cost and jacobian
                evaluation failed.

        The cause was a DDS type ambiguity, not a sensor fault: `/livox/lidar` was
        advertising BOTH livox_ros_driver2/msg/CustomMsg (a stale discovery entry from a
        driver that had been killed) and sensor_msgs/msg/PointCloud2 (the live one).
        `ros2 bag record` resolves the message type through discovery, so on some
        viewpoints it resolved the stale type and recorded nothing. On a shared DDS domain
        the stale entry can be held in another host's cache, which we cannot clear -- so
        this is not a condition we can prevent, only one we can detect.

        Detect it HERE, while the rig is still at the viewpoint and re-taking it costs
        fifteen seconds. The alternative is discovering it after the capture, when the
        only remedy is redoing all of it.
        """
        if self.bag_secs <= 0:
            return True, ''
        bag_dir = os.path.join(self.out_dir, 'bags', 'pair_%03d' % idx)
        dbs = glob.glob(os.path.join(bag_dir, '*.db3'))
        if not dbs:
            return False, 'no .db3 written (did the recorder fail to start?)'
        try:
            con = sqlite3.connect(dbs[0])
            rows = dict(con.execute(
                'SELECT t.name, count(m.id) FROM topics t '
                'LEFT JOIN messages m ON m.topic_id = t.id GROUP BY t.id').fetchall())
            con.close()
        except sqlite3.Error as exc:
            return False, 'bag unreadable (%s)' % exc
        # Every topic we asked for must be present AND non-empty. The cloud matters most,
        # but a missing image is equally fatal to the solve, so all are checked.
        missing = [tp for tp in self.topics if rows.get(tp, 0) == 0]
        if missing:
            have = ', '.join('%s=%d' % (k.split('/')[-1], v) for k, v in sorted(rows.items()))
            return False, 'recorded NOTHING on %s (bag has: %s)' % (', '.join(missing), have)
        return True, ', '.join('%s=%d' % (tp.split('/')[-1], rows[tp]) for tp in self.topics)

    def _stop_bag(self, proc):
        """SIGINT, not kill -- `ros2 bag record` only flushes metadata.yaml on a clean shutdown,
        and a bag without it is unreadable."""
        if proc is None:
            return
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            proc.kill()
            print("  WARNING: bag recorder did not exit cleanly; bag may be unusable")

    def capture_pair(self, idx):
        bag = self._start_bag(idx)
        bag_t0 = time.time()
        self.clouds = []
        self.collecting = True
        t0 = time.time()
        while len(self.clouds) < self.sweeps and time.time() - t0 < 10.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.collecting = False
        # Hold the remaining dwell so the bag gets its full stationary window.
        while bag is not None and (time.time() - bag_t0) < self.bag_secs:
            rclpy.spin_once(self, timeout_sec=0.05)
        self._stop_bag(bag)
        # Verify AFTER the recorder has flushed metadata, BEFORE the operator moves on.
        bag_ok, bag_note = self._verify_bag(idx)
        if not self.clouds or self.image is None:
            return None

        cloud = np.vstack(self.clouds)
        img = np.frombuffer(bytes(self.image.data), dtype=np.uint8)
        img = img.reshape(self.image.height, self.image.width, -1)

        extra = {}
        if self.depth is not None:
            # 16UC1 = uint16 millimetres. Kept in raw units; converting to metres here would
            # silently lose the "0 means no return" sentinel that depth validity depends on.
            d = np.frombuffer(bytes(self.depth.data), dtype=np.uint16)
            extra['depth'] = d.reshape(self.depth.height, self.depth.width)
            extra['depth_encoding'] = self.depth.encoding

        np.savez_compressed(
            os.path.join(self.out_dir, 'pair_%03d.npz' % idx),
            image=img, encoding=self.image.encoding, cloud=cloud,
            sweeps=len(self.clouds),
            stamp=self.image.header.stamp.sec + self.image.header.stamp.nanosec * 1e-9,
            **extra,
        )
        return cloud.shape[0], img.shape, ('depth' in extra), bag_ok, bag_note


def main(args=None):
    rclpy.init(args=args)
    n = CaptureNode()
    print("waiting for camera + LiDAR streams ...")
    if not n.wait_for_streams():
        print("ERROR: no image/camera_info. Are the camera and LiDAR drivers running?")
        n.destroy_node(); rclpy.shutdown(); return 1

    print("intrinsics -> %s" % n.save_intrinsics())
    print("output dir : %s" % n.out_dir)
    if n.bag_secs > 0:
        print("bag record : %d s per viewpoint -> %s/bags/" % (n.bag_secs, n.out_dir))
        # Raw RGB dominates bag size (1280x720x3 at ~30 Hz is ~83 MB/s), so a full run is
        # tens of GB. Surface that up front: a disk-full mid-run corrupts bags silently,
        # and the set still has to be copied to the workstation to be solved.
        img_bytes = n.image.height * n.image.width * max(1, n.image.step // max(1, n.image.width))
        est_gb = img_bytes * 30.0 * n.bag_secs * n.target_pairs / 1e9
        free_gb = shutil.disk_usage(n.out_dir).free / 1e9
        print("estimated  : ~%.1f GB of bags (%.0f GB free)" % (est_gb, free_gb))
        if free_gb < est_gb * 1.5:
            print("WARNING: low free space. A disk-full mid-run leaves unusable bags.")
    else:
        print("bag record : OFF (set -p bag_secs:=15 if solving with koide3)")
    print("")
    print("Capture %d pairs. MOVE THE RIG BETWEEN CAPTURES -- vary yaw, pitch and standoff," % n.target_pairs)
    print("and keep real depth structure in view (walls at different ranges, box edges, a")
    print("doorway). Near-identical viewpoints yield a confident but WRONG extrinsic.")
    print("ENTER = capture, 'q' = finish early.")

    idx = 0
    while idx < n.target_pairs:
        try:
            key = input("\n[pair %d/%d] reposition, then ENTER: " % (idx + 1, n.target_pairs))
        except EOFError:
            break
        if key.strip().lower() == 'q':
            break
        res = n.capture_pair(idx)
        if res is None:
            print("  capture FAILED (no cloud/image) -- retry")
            continue
        pts, shape, has_depth, bag_ok, bag_note = res
        print("  saved pair_%03d.npz  points=%d  image=%s  depth=%s"
              % (idx, pts, shape, "yes" if has_depth else "NO"))
        if not bag_ok:
            # DO NOT advance. The .npz is fine but the BAG is what koide3 solves from, and a
            # bag missing a topic is worthless -- see CaptureNode._verify_bag. Retaking the
            # viewpoint now costs 15 s; finding this at solve time costs the whole capture.
            print("  *** BAG REJECTED: %s" % bag_note)
            print("  NOT counting this viewpoint. Stay where you are and press ENTER to")
            print("  retake it. If it keeps failing, the likely cause is a DDS type")
            print("  ambiguity on the cloud topic -- check:")
            print("      ros2 topic type %s" % n.topics[-1])
            print("  and if it prints TWO types, restart the LiDAR driver and the ros2")
            print("  daemon before continuing.")
            shutil.rmtree(os.path.join(n.out_dir, 'bags', 'pair_%03d' % idx),
                          ignore_errors=True)
            continue
        if bag_note:
            print("  bag OK: %s" % bag_note)
        idx += 1

    print("\n%d pairs in %s" % (idx, n.out_dir))
    if idx < 5:
        print("WARNING: fewer than 5 pairs -- most solvers will be under-constrained.")
    print("Next: solve for T_camera_lidar, then validate with validate_calib.")
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
