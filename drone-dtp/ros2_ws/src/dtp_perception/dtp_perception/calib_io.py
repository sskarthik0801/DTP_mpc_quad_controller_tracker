"""Locate, load and validate the camera<->LiDAR calibration at RUNTIME.

WHY THIS EXISTS
---------------
`fusion_node` and `lidar_overlay_node` both declared `calib_file` as the *relative* path
`'calib/drone_cam_livox.yaml'`. A relative path resolves against the process working
directory, not the package share directory -- so it only ever worked if you happened to
launch from the package source tree. From the install space on the aircraft, which is how it
actually runs, it resolves to nothing.

That was latent only because both nodes are M0.1 scaffolds that store the parameter and never
open the file. The moment Phase 1 implements the loader it becomes a real failure, and the
dangerous version of that failure is the quiet one: fall back to an identity extrinsic and the
camera and LiDAR are treated as co-located. Every projected range is then wrong, in-view
percentages look normal, and overlays look *plausible* -- the same class of error
`score_alignment` exists to catch at calibration time.

So this module resolves against the package share directory, and **raises** rather than
defaulting. A follow controller with no calibration must refuse to start, not improvise.

WHAT IT VALIDATES
-----------------
Structure, not accuracy -- accuracy is `score_alignment`'s job at calibration time:
  * a 4x4 homogeneous transform with an exact [0,0,0,1] bottom row,
  * a rotation block that is actually a rotation (orthonormal, det +1), because a transposed
    or mangled YAML block still parses as a 4x4 and then silently mirrors every range,
  * intrinsics present and positive,
  * and -- see check_camera_serial -- that the calibration belongs to the camera actually
    connected.

It also surfaces the provenance `# note` line, so the log records WHICH calibration is flying
-- in particular whether it is still a MOTORS-OFF bench solve (plan/dtp-imp/05 Step 0.3.1b).
"""
import os

import numpy as np

PACKAGE = 'dtp_perception'
DEFAULT_CALIB = 'drone_cam_livox.yaml'

# A rotation block that drifts further than this from orthonormal is a mangled file, not
# float noise. Solver output is orthonormal to ~1e-9; YAML round-trips at 9 decimals.
ORTHONORMAL_TOL = 1e-4

# Orbbec's USB vendor id, used to find attached camera bodies in sysfs.
ORBBEC_VENDOR_ID = '2bc5'

# The Livox driver node our launches create. Queried by name because topic endpoint metadata
# reports _NODE_NAME_UNKNOWN_ under FastDDS for cross-process endpoints -- i.e. this fallback
# is the NORMAL path, not a rare one, so a wrong value here disables the gate below entirely.
#
# *** IT WAS WRONG FROM 2026-09-11 TO 2026-09-12. *** It read '/livox_lidar_publisher', the
# upstream default, which both our launches stopped using on 2026-09-11 (they now name the node
# 'dtp_livox_driver', because ANOTHER LAB'S ROBOT was publishing under the upstream default on
# our DDS domain and we bound THEIR LiDAR). With this constant naming a node that no longer
# exists, _remote_string_param returned None, check_lidar_source logged "LiDAR source
# UNVERIFIED" and returned -- so for one day the gate that refuses a foreign LiDAR was off.
#
# DO NOT add '/livox_lidar_publisher' back as a second accepted name. That name is now exactly
# what the FOREIGN driver is called, so accepting it would re-open the hole the rename closed.
LIVOX_DRIVER_NODE = '/dtp_livox_driver'


class CalibError(RuntimeError):
    """Raised when the calibration is missing, unusable, or belongs to a different camera."""


def resolve_calib_path(value=None):
    """Turn a `calib_file` parameter into an absolute path that exists, or raise.

    Accepts an absolute path (used as-is), a path relative to the CWD (honoured if it
    actually exists, so a bench workflow can still point at a scratch file), or a bare
    filename resolved against the installed package share directory -- which is the case
    that was broken.
    """
    value = (value or DEFAULT_CALIB).strip()
    expanded = os.path.expanduser(value)

    if os.path.isabs(expanded):
        if os.path.exists(expanded):
            return expanded
        raise CalibError("calibration not found at absolute path: %s" % expanded)

    # An explicit relative path that resolves against the CWD is honoured, but only if it is
    # really there -- never as a silent fallback.
    if os.path.exists(expanded):
        return os.path.abspath(expanded)

    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory(PACKAGE)
    except Exception as exc:  # package not found / ament unavailable
        raise CalibError(
            "cannot locate package '%s' to resolve calibration %r: %s" % (PACKAGE, value, exc))

    # Bare filename -> share/<pkg>/calib/<name>; a relative path is joined as given.
    candidate = os.path.join(share, 'calib', os.path.basename(expanded)) \
        if os.path.dirname(expanded) == '' else os.path.join(share, expanded)
    if os.path.exists(candidate):
        return candidate

    raise CalibError(
        "calibration %r not found.\n"
        "  tried (cwd)   : %s\n"
        "  tried (share) : %s\n"
        "  Run the calibration workflow (plan/dtp-imp/19) and deploy the YAML with\n"
        "  scripts/deploy_calib.sh, or pass an absolute calib_file."
        % (value, os.path.abspath(expanded), candidate))


def _read_note(path):
    """Return the provenance '# note' line, if the generator wrote one."""
    try:
        with open(path, encoding='utf-8') as f:
            for line in f:
                if line.startswith('# note'):
                    return line.split(':', 1)[-1].strip()
                if line and not line.startswith('#'):
                    break
    except OSError:
        pass
    return None


def load_calib(path):
    """Load and structurally validate a calibration. Returns (T_camera_lidar, intrinsics, note).

    T_camera_lidar is a 4x4 float ndarray mapping LiDAR frame -> camera optical frame.
    intrinsics is a dict with fx, fy, cx, cy (plus d/distortion_model/camera_serial when the
    file records them).
    """
    import yaml
    try:
        with open(path, encoding='utf-8') as f:
            doc = yaml.safe_load(f)
    except Exception as exc:
        raise CalibError("could not parse %s: %s" % (path, exc))
    if not isinstance(doc, dict):
        raise CalibError("%s is not a YAML mapping" % path)

    if 'T_camera_lidar' not in doc:
        raise CalibError(
            "%s has no T_camera_lidar key. NOTE koide3 emits T_LIDAR_CAMERA -- the INVERSE of "
            "what we consume. Convert with dtp_perception.calibration.import_koide3, never by "
            "hand: a mirrored extrinsic still projects in-view and fails as a wrong range on a "
            "person." % path)

    T = np.array(doc['T_camera_lidar'], dtype=float)
    if T.shape != (4, 4):
        raise CalibError("T_camera_lidar in %s is %s, expected 4x4" % (path, T.shape))
    if not np.allclose(T[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise CalibError("T_camera_lidar bottom row is %s, expected [0,0,0,1]" % (T[3],))

    R = T[:3, :3]
    if not np.allclose(R @ R.T, np.eye(3), atol=ORTHONORMAL_TOL):
        raise CalibError(
            "T_camera_lidar rotation block is not orthonormal (max deviation %.2e). The file is "
            "mangled or the matrix was hand-edited; a non-rotation still projects and silently "
            "distorts every range." % np.abs(R @ R.T - np.eye(3)).max())
    det = float(np.linalg.det(R))
    if det < 0:
        raise CalibError(
            "T_camera_lidar rotation has determinant %.4f (< 0) -- this is a REFLECTION, not a "
            "rotation. Almost always a hand-copied or transposed matrix; it mirrors the scene "
            "and reports confident wrong ranges." % det)
    if abs(det - 1.0) > ORTHONORMAL_TOL:
        raise CalibError("T_camera_lidar rotation determinant is %.6f, expected 1.0" % det)

    intr = doc.get('intrinsics')
    if not isinstance(intr, dict) or not all(k in intr for k in ('fx', 'fy', 'cx', 'cy')):
        raise CalibError("%s has no usable `intrinsics:` block (need fx, fy, cx, cy)" % path)
    for k in ('fx', 'fy'):
        if float(intr[k]) <= 0:
            raise CalibError("%s intrinsic %s is %s, must be positive" % (path, k, intr[k]))

    out = {k: float(intr[k]) for k in ('fx', 'fy', 'cx', 'cy')}
    if 'd' in doc:
        out['d'] = list(doc['d'])
    if 'distortion_model' in doc:
        out['distortion_model'] = doc['distortion_model']
    if doc.get('camera_serial'):
        out['camera_serial'] = str(doc['camera_serial']).strip()
    if doc.get('lidar_ip'):
        out['lidar_ip'] = str(doc['lidar_ip']).strip()

    return T, out, _read_note(path)


def attached_camera_serials(vendor=ORBBEC_VENDOR_ID):
    """Serials of every Orbbec body attached to THIS machine, from sysfs.

    NOT read from the driver's `serial_number` parameter, which looks like the obvious source
    and is worthless here: that parameter is the *selector you asked for*, not the device that
    was bound. Verified 2026-09-07 -- it read `CPC64630007M` with **no camera plugged in at
    all**. Gating on it would be circular: launch with `serial_number:=X` and the parameter
    reads X by construction, so the check passes no matter what actually bound.

    sysfs is Linux-specific and describes the LOCAL machine. That is correct here because the
    driver runs on the same box as the camera; it would need rethinking for a networked camera.
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
            with open(os.path.join(d, 'idVendor')) as f:
                if f.read().strip().lower() != vendor:
                    continue
            with open(os.path.join(d, 'serial')) as f:
                s = f.read().strip()
            if s:
                out.append(s)
        except OSError:
            continue
    return sorted(set(out))


def check_camera_serial(node, expected):
    """Refuse to run this calibration against a DIFFERENT camera body.

    WHY: on 2026-09-07 this NX had two Orbbec bodies attached -- a Gemini 336 on USB 2.0 and
    the Gemini 336L on USB 3.0 -- and with no selector the driver bound whichever it
    enumerated first. After a reboot it took the 336. Our calibration belongs to the 336L.

    Nothing else catches that. The YAML is structurally valid, `score_alignment` and
    `compare_calib` both confirm the extrinsic is internally consistent, and overlays still
    look plausible -- while every range to a person is silently wrong, because the intrinsics
    and the mount belong to a different camera.

    A DEFINITE mismatch is fatal. "Cannot determine" is only a warning: the camera driver may
    legitimately start after this node, and hard-failing on that would make start-up order
    brittle for no safety gain. That distinction is deliberate -- do not collapse it into a
    single "unverified -> refuse", or nodes become unlaunchable in half the valid orderings.
    """
    if not expected:
        node.get_logger().warn(
            "  calibration records no camera_serial -- cannot verify it belongs to the camera "
            "actually connected. Re-run write_calib_yaml with --camera_serial.")
        return None

    attached = attached_camera_serials()

    if not attached:
        node.get_logger().warn(
            "  no Orbbec camera is attached to this machine. This calibration expects %s -- "
            "UNVERIFIED." % expected)
        return None

    if expected not in attached:
        # Definite and fatal: the body this calibration was solved on is not even plugged in,
        # so whatever the driver bound is certainly the wrong one.
        raise CalibError(
            "CAMERA MISMATCH -- refusing to start.\n"
            "  calibration belongs to : %s\n"
            "  attached instead       : %s\n"
            "  That body is not connected, so the driver cannot have bound it. The intrinsics\n"
            "  and extrinsic here are for a different physical camera.\n"
            "  Nothing downstream would notice: the file is valid and overlays look plausible,\n"
            "  but every range to a person would be wrong.\n"
            "  Fix: connect %s, or re-calibrate against the camera you intend to fly."
            % (expected, ', '.join(attached), expected))

    if len(attached) > 1:
        # The expected body IS present, but so is another -- and nothing exposed by the driver
        # says which one it opened (see attached_camera_serials). Do not claim verification we
        # do not have; the launch-file `serial_number` pin is what makes this deterministic.
        node.get_logger().warn(
            "  AMBIGUOUS: %d Orbbec bodies attached (%s). The calibration's %s is among them, "
            "but nothing reports which one the driver bound. Pin it in the launch "
            "(serial_number:=%s) and unplug the others."
            % (len(attached), ', '.join(attached), expected, expected))
        return None

    node.get_logger().info(
        "  camera verified: %s is the only Orbbec attached and matches the calibration" % expected)
    return True


def check_lidar_source(node, topic='/livox/lidar', expected_ip=None, timeout_sec=5.0):
    """Verify the LiDAR data comes from OUR device, not another machine on the DDS domain.

    WHY: on 2026-09-07 `/livox/lidar_192_168_1_146` and `_160` were publishing 20k-point clouds
    at 10 Hz, and a live overlay built on one of them looked entirely plausible. They were a
    DIFFERENT ROBOT's LiDAR -- a driver on another host, sharing our DDS domain because
    ROS_LOCALHOST_ONLY=0, configured from `/home/lprc/...` with two LiDAR IPs that do not exist
    on our wired link. Meanwhile the drone's own Mid-360 was not being published at all.

    A topic existing is not evidence that it is yours. This is the LiDAR counterpart of
    check_camera_serial, and it matters more, because a point cloud carries no identity of its
    own: any indoor cloud projected through a plausible extrinsic looks vaguely scene-like.

    The discriminator is the publishing driver's `user_config_path`: a path belonging to a
    driver on another machine is not readable here. That single fact separates the cases
    cleanly, and the configured LiDAR IP then confirms WHICH local device.
    """
    import json

    # Wait for DDS discovery. This runs during node construction, and a freshly created node has
    # not yet discovered anything -- querying immediately reports "no publisher" even when the
    # topic is publishing at 10 Hz, which is a false negative that would train people to ignore
    # this warning.
    # Wait for a publisher that also carries a usable NODE NAME. Waiting only for the endpoint to
    # appear is not enough: the endpoint shows up first and its node name resolves a moment later,
    # so an immediate read returns '_NODE_NAME_UNKNOWN_' and sends us down the inference fallback
    # below -- which on 2026-09-08 meant falling back to a name owned by a FOREIGN driver and
    # refusing our own perfectly good LiDAR. Waiting for the name costs nothing when it is already
    # there and removes a whole class of spurious refusal when it is not.
    import time as _time

    def _named(endpoints):
        return [e for e in endpoints
                if (e.node_name or '').strip() not in ('', '_NODE_NAME_UNKNOWN_')]

    deadline = _time.time() + timeout_sec
    infos = []
    while _time.time() < deadline:
        try:
            infos = node.get_publishers_info_by_topic(topic)
        except Exception:
            break
        if _named(infos):
            break
        _time.sleep(0.2)
    if not infos:
        node.get_logger().warn(
            "  no publisher on %s after %.0fs -- LiDAR source UNVERIFIED (driver not up yet?)"
            % (topic, timeout_sec))
        return None

    # Identify the node that ACTUALLY publishes the topic we are about to consume.
    #
    # This used to query a hardcoded node NAME, which was wrong in both directions:
    #   * FALSE POSITIVE (hit 2026-09-08) -- our own driver running under any other name, with a
    #     foreign /livox_lidar_publisher merely present on the domain: the gate read the FOREIGN
    #     node's config and refused perfectly good local data.
    #   * FALSE NEGATIVE, and this is the dangerous one -- ROS 2 does not enforce unique node
    #     names. With our local driver also called livox_lidar_publisher, the parameter query can
    #     be answered by the LOCAL node while the FOREIGN one publishes the topic we consume. The
    #     gate then reports "verified" while the exact hazard it exists to catch is happening.
    #
    # So resolve identity from the topic's own endpoint. Endpoint metadata CAN come back without
    # a usable node name under FastDDS; when it does we fall back to the constant, but we say the
    # identity is inferred rather than presenting a guess as a verified fact.
    if len(infos) > 1:
        names = sorted({('%s/%s' % ((i.node_namespace or '').rstrip('/'), i.node_name)).lstrip('/')
                        for i in infos})
        raise CalibError(
            "MORE THAN ONE PUBLISHER ON %s -- refusing to start.\n"
            "  publishers : %s\n"
            "  Which cloud a subscriber receives is undefined, and at least one of these is not\n"
            "  ours. Stop the extra driver, or isolate discovery (ROS_LOCALHOST_ONLY=1 / a\n"
            "  distinct ROS_DOMAIN_ID)."
            % (topic, ', '.join(names)))

    pub_name = (infos[0].node_name or '').strip() if infos else ''
    inferred = pub_name in ('', '_NODE_NAME_UNKNOWN_')
    if inferred:
        driver = LIVOX_DRIVER_NODE
        node.get_logger().warn(
            "  DDS did not report a node name for the publisher on %s; falling back to %s. "
            "The source identity below is INFERRED, not confirmed." % (topic, driver))
    else:
        driver = '%s/%s' % ((infos[0].node_namespace or '').rstrip('/'), pub_name)

    cfg_path = _remote_string_param(node, driver, 'user_config_path', timeout_sec)
    if cfg_path is None:
        # Name the most likely cause rather than only the symptom. The usual one is that the
        # LiDAR was started from the UPSTREAM launch (livox_ros_driver2 msg_MID360_launch.py,
        # which super_bringup/launch/m1_super_lidar.launch.py still tells operators to use),
        # where the node keeps its default name livox_lidar_publisher. We deliberately do NOT
        # accept that name: it is what the foreign robot is called, so trusting it would
        # re-open the hole the 2026-09-11 rename closed. Unverified is the correct answer --
        # it just has to be an ACTIONABLE one.
        node.get_logger().warn(
            "  could not read user_config_path from %s -- LiDAR source UNVERIFIED. If you "
            "started the LiDAR with the upstream launch, its node is named "
            "livox_lidar_publisher and this gate CANNOT tell our LiDAR from another "
            "robot's. Relaunch via dtp_bringup sensors.launch.py (or dtp_perception "
            "calib_sensors.launch.py), which name it %s." % (driver, LIVOX_DRIVER_NODE))
        return None

    if not os.path.exists(cfg_path):
        raise CalibError(
            "LIDAR SOURCE IS NOT THIS MACHINE -- refusing to start.\n"
            "  topic            : %s\n"
            "  publishing node  : %s\n"
            "  its config path  : %s  (does NOT exist here)\n"
            "  A driver on ANOTHER HOST is publishing onto this DDS domain, so these points are\n"
            "  a different robot's. They will project plausibly and be entirely wrong.\n"
            "  Fix: start our own driver (dtp_perception calib_sensors.launch.py), or set\n"
            "  ROS_LOCALHOST_ONLY=1 / a distinct ROS_DOMAIN_ID to stop sharing discovery."
            % (topic, driver, cfg_path))

    try:
        with open(cfg_path, encoding='utf-8') as f:
            cfg = json.load(f)
        ips = []
        for entry in cfg.get('lidar_configs', []):
            if entry.get('ip'):
                ips.append(str(entry['ip']))
    except Exception as exc:
        node.get_logger().warn("  could not parse %s: %s -- LiDAR IP UNVERIFIED" % (cfg_path, exc))
        return None

    if expected_ip and expected_ip not in ips:
        raise CalibError(
            "LIDAR MISMATCH -- refusing to start.\n"
            "  calibration expects : %s\n"
            "  driver configured   : %s\n"
            "  config              : %s\n"
            "  These points are from a different LiDAR than the one calibrated against."
            % (expected_ip, ', '.join(ips) or '(none)', cfg_path))

    node.get_logger().info(
        "  lidar source verified: %s from local driver %s (config %s)"
        % (', '.join(ips) or 'unknown-ip', driver, os.path.basename(cfg_path)))
    return True


def _remote_string_param(node, target_node, name, timeout_sec=5.0):
    """Read one string parameter from another node. None if unavailable."""
    try:
        import rclpy
        from rcl_interfaces.srv import GetParameters
    except ImportError:
        return None
    cli = node.create_client(GetParameters, target_node.rstrip('/') + '/get_parameters')
    try:
        if not cli.wait_for_service(timeout_sec=timeout_sec):
            return None
        req = GetParameters.Request()
        req.names = [name]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_sec)
        res = fut.result()
        if not res or not res.values:
            return None
        v = res.values[0]
        return v.string_value.strip() if v.string_value.strip() else None
    except Exception:
        return None
    finally:
        try:
            node.destroy_client(cli)
        except Exception:
            pass


def load_for_node(node, param_value=None):
    """Resolve + load + verify + log for a rclpy node. Raises CalibError; do not continue.

    Logs the provenance note, warns when the calibration is still a motors-off bench solve --
    a condition an operator should see at every start-up, not one buried in a YAML header --
    and refuses to start if the calibration belongs to a different camera body.
    """
    path = resolve_calib_path(param_value)
    T, intr, note = load_calib(path)
    baseline = float(np.linalg.norm(T[:3, 3]))
    node.get_logger().info(
        "calibration loaded: %s (baseline %.3f m, fx=%.1f)" % (path, baseline, intr['fx']))
    if note:
        node.get_logger().info("  provenance: %s" % note)
        if 'MOTORS OFF' in note.upper() or 'BENCH' in note.upper():
            node.get_logger().warn(
                "  this is a MOTORS-OFF bench calibration; plan/dtp-imp/05 Step 0.3.1b requires "
                "motor-on re-validation before flight (vibration shifts the extrinsic)")
    else:
        node.get_logger().warn(
            "  calibration has no provenance note -- cannot tell which rig or whether it was "
            "motor-on validated")

    # Both raise CalibError on a definite mismatch and warn when undeterminable. They answer
    # the same question for the two sensors: is this the device the calibration belongs to?
    check_camera_serial(node, intr.get('camera_serial'))
    check_lidar_source(node, expected_ip=intr.get('lidar_ip'))
    return T, intr
