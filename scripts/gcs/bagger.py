#!/usr/bin/env python3
"""Where ROS bags go, what is worth recording, and what is already there.

The process half lives in supervisor.py (class Recorder), because a recording
is a process group like everything else this dashboard starts and has to be
stopped the same way -- SIGINT to a pgid we own, never a name we matched.
This file is the half with no ROS and no processes in it: the naming, the disk
arithmetic, the topic classification and the scan of bags/.  All stdlib, so

    python3 scripts/gcs/bagger.py

answers "what have I recorded, how big is it, and is it playable" from a
terminal with nothing sourced.

Two things here are not arbitrary:

  * MIN_FREE_GB.  /livox/lidar and /cloud_registered carry the MID360's whole
    ~200k points/s between them, so recording everything on this aircraft is a
    few hundred MB a minute, and bags land on the same rootfs the flight stack
    swaps against.  Filling it mid-test would not just lose the bag.

  * IMAGE_TYPES / IMAGE_EXCLUDE.  The camera sits behind a second password
    (admin.py), and /api/state omits it entirely for a client that has not
    unlocked it.  A topic picker that listed /camera/color/image_raw/compressed
    would hand back exactly what that redaction withholds, so the list is
    filtered by TYPE for a locked client and `record all` gets an --exclude.
    rosbag2 can only filter by NAME, hence the regex as well as the type set.
"""
import os
import re
import time

BAG_DIRNAME = 'bags'          # <repo>/bags, gitignored
MIN_FREE_GB = 5.0             # refuse to start, and stop a running one, below this
LOW_FREE_GB = 25.0            # say so, but record anyway

# Types worth a warning in the picker: on this aircraft these are the ones that
# turn a five-minute bag into gigabytes.
HEAVY_TYPES = {
    'sensor_msgs/msg/PointCloud2',
    'sensor_msgs/msg/Image',
    'sensor_msgs/msg/CompressedImage',
    'livox_ros_driver2/msg/CustomMsg',
}
IMAGE_TYPES = {'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'}

# What `record all` excludes for a client that has not unlocked Admin control.
# By name, because that is all rosbag2 can filter on (-x is a regex matched
# against the topic name).  Deliberately broad: over-excluding costs a topic
# nobody was recording anyway, under-excluding leaks the thing the gate exists
# to withhold.  An unlocked client gets a plain --all with no exclusion.
IMAGE_EXCLUDE = r'image_raw|image_rect|image_color|image_color_rect|/compressed'

# Topic presets for the Flight day card (flightday.py). stage_a = drone-dtp/scripts/
# stage_a_record.sh's list, verbatim; m51 = docs/m5.1-one-graph-runbook.md SS4; stage_b =
# m51 plus perception, the loss oracle and the camera frames. Image topics in a preset go
# through the same admin gate as a hand-picked one: the card says which need Admin control.
_M51 = [
    '/planning/pos_cmd', '/planning/follow_goal', '/planning/goal_status', '/planning/yaw_target',
    '/fmu/in/trajectory_setpoint', '/fmu/in/offboard_control_mode', '/fmu/in/vehicle_command',
    '/fmu/in/vehicle_visual_odometry', '/fmu/out/vehicle_command_ack', '/fmu/out/vehicle_status_v1',
    '/fmu/out/vehicle_control_mode', '/fmu/out/vehicle_local_position_v1',
    '/dtp/abort', '/mux/state', '/dtp/lock_state', '/dtp/targets', '/dtp/follow_target',
    '/dtp/sensors_ok', '/dtp/perception_ok', '/dtp/sensor_health', '/lidar_slam/odom',
    '/rog_map/target_exclusion', '/tf', '/tf_static',
    # terrain_estimator's ground (command_mux terrain_follow_enable); absent when it is off.
    '/dtp/terrain',
    # px4_offboard_bridge's camera yaw servo; published only with yaw_servo_enable, while
    # engaged (docs/plan-camera-yaw-servo.md section 2.6).
    '/bridge/yaw_servo',
]
PRESETS = {
    'stage_a': [
        '/dtp/shadow/follow_goal', '/dtp/shadow/yaw_target',
        '/dtp/targets', '/dtp/follow_target', '/dtp/follow_goal', '/dtp/lock_state',
        '/dtp/sensors_ok', '/dtp/sensor_health', '/dtp/perception_ok', '/dtp/abort', '/mux/state',
        '/dtp/terrain',
        '/planning/follow_goal', '/planning/yaw_target', '/planning/goal_status', '/planning/pos_cmd',
        '/goal_pose', '/dtp/nav_goal',
        '/fmu/out/vehicle_status_v1', '/fmu/out/vehicle_control_mode', '/fmu/out/vehicle_local_position_v1',
        '/fmu/out/vehicle_command_ack', '/fmu/in/trajectory_setpoint', '/fmu/in/vehicle_command',
        '/fmu/in/vehicle_visual_odometry',
        '/lidar_slam/odom', '/tf', '/tf_static', '/livox/imu', '/cloud_registered_body',
        '/camera/color/image_raw/compressed', '/camera/color/camera_info',
        '/camera/depth/image_raw', '/camera/depth/camera_info',
    ],
    'm51': list(_M51),
    'stage_b': _M51 + [
        '/planning/loss_event', '/dtp/shadow/follow_goal', '/dtp/shadow/yaw_target',
        '/livox/imu', '/cloud_registered_body',
        '/camera/color/image_raw/compressed', '/camera/color/camera_info',
        '/camera/depth/image_raw', '/camera/depth/camera_info',
    ],
}

# Topic names come from the browser and end up in an argv.  There is no shell
# (Popen takes a list), so this is not about quoting -- it is about a value
# starting with '-' being read by ros2 as an OPTION.  A ROS topic is a slash
# path of [A-Za-z0-9_], so anything else is simply not one.
_TOPIC = re.compile(r'^/[A-Za-z0-9_]+(/[A-Za-z0-9_]+)*$')
_UNSAFE = re.compile(r'[^A-Za-z0-9._-]+')
MAX_LABEL = 40


def valid_topic(name):
    return bool(_TOPIC.match(name or '')) and len(name) <= 256


def bag_root(repo):
    return os.path.join(repo, BAG_DIRNAME)


def new_bag_path(repo, label=''):
    """(absolute path, name) for a recording starting now.

    The timestamp is always there and always last-ish, so `ls bags/` sorts by
    time within a label; the label is what makes a bag findable a week later
    ("p3-hover" beats "rosbag2_2026_09_10-14_33_02", which is what rosbag2
    would have called it).
    """
    label = _UNSAFE.sub('-', (label or '').strip())[:MAX_LABEL].strip('-.')
    stamp = time.strftime('%Y-%m-%d_%H%M%S')
    name = f'{label}_{stamp}' if label else stamp
    return os.path.join(bag_root(repo), name), name


def record_argv(out, topics=(), all_topics=False, exclude=''):
    """The `ros2 bag record` command line.

    `out` is relative to the repo so the log line reads `-o bags/<name>`.
    rosbag2 refuses an output directory that already exists, which is exactly
    the behaviour we want -- a second recording can never overwrite the first.

    --exclude is dropped unless --all is in play.  Its own --help says it
    "works on top of --all, --regex, or topics list", and that is wrong in
    Humble: `ros2 bag record -x ... /a /b` exits 1 with

        --exclude argument cannot be used when specifying a list of topics
        explicitly

    Nothing is lost by dropping it -- an explicit list is filtered before it
    ever gets here (server._bag_start refuses a restricted topic by name), so
    the exclusion only ever had work to do on the --all path.
    """
    argv = ['ros2', 'bag', 'record', '-o', out]
    if all_topics:
        argv.append('-a')
        if exclude:
            argv += ['-x', exclude]
    argv += list(topics)
    return argv


def free_gb(path):
    try:
        st = os.statvfs(path)
    except OSError:
        return 0.0
    return round(st.f_bavail * st.f_frsize / 1073741824.0, 1)


def dir_size(path):
    """(megabytes, file count) for one bag directory."""
    total, files = 0, 0
    stack = [path]
    while stack:
        try:
            with os.scandir(stack.pop()) as it:
                for entry in it:
                    try:
                        if entry.is_dir(follow_symlinks=False):
                            stack.append(entry.path)
                        elif entry.is_file(follow_symlinks=False):
                            total += entry.stat().st_size
                            files += 1
                    except OSError:
                        continue
        except OSError:
            continue
    # 2 dp, not 1: a bag that has just started is tens of KB, and "0.0 MB"
    # on a recording that is genuinely working reads as a broken panel.
    return round(total / 1048576.0, 2), files


def read_metadata(path):
    """What rosbag2 wrote about a finished bag, or Nones.

    metadata.yaml is written when the recorder SHUTS DOWN, so its presence is
    the one reliable answer to "is this bag playable" -- a recording that was
    SIGKILLed has all its messages on disk and no way to read them back.  That
    matters enough to be the headline in the UI.

    Parsed with regexes rather than PyYAML on purpose: this module is stdlib
    only so it still answers from a terminal with no ROS sourced.  A format
    change costs the two numbers, never the finalised flag or the size.
    """
    meta = os.path.join(path, 'metadata.yaml')
    out = {'finalized': os.path.isfile(meta), 'messages': None,
           'duration_s': None, 'topics': None}
    if not out['finalized']:
        return out
    try:
        with open(meta, errors='replace') as f:
            text = f.read(2000000)
    except OSError:
        return out
    # Anchored at two spaces: the per-topic entries carry a message_count too,
    # indented deeper, and matching one of those would report a single topic's
    # count as the whole bag's.
    m = re.search(r'^  message_count:\s*(\d+)\s*$', text, re.M)
    if m:
        out['messages'] = int(m.group(1))
    m = re.search(r'^  duration:\s*\n\s+nanoseconds:\s*(\d+)', text, re.M)
    if m:
        out['duration_s'] = round(int(m.group(1)) / 1e9, 1)
    n = len(re.findall(r'^\s*-\s*topic_metadata:', text, re.M))
    if n:
        out['topics'] = n
    return out


def is_bag(path):
    """A rosbag2 output directory, finished or not.

    Checked by content rather than by name so a half-written bag -- the one you
    most want to see listed -- still shows up: metadata.yaml appears only at
    shutdown, but the storage file exists from the first message.
    """
    if not os.path.isdir(path):
        return False
    if os.path.isfile(os.path.join(path, 'metadata.yaml')):
        return True
    try:
        return any(e.name.endswith(('.db3', '.mcap'))
                   for e in os.scandir(path))
    except OSError:
        return False


def list_bags(repo, limit=40, active=None):
    """Recordings in bags/, newest first.

    `active` is the path of a recording still in progress, which is reported as
    such rather than as an unfinalised bag -- they look identical on disk and
    only one of them is a problem.
    """
    root = bag_root(repo)
    rows = []
    try:
        entries = sorted(os.scandir(root), key=lambda e: e.name, reverse=True)
    except OSError:
        return rows
    for entry in entries:
        if len(rows) >= limit:
            break
        if not is_bag(entry.path):
            continue
        mb, files = dir_size(entry.path)
        meta = read_metadata(entry.path)
        rows.append({'name': entry.name, 'dir': f'{BAG_DIRNAME}/{entry.name}',
                     'size_mb': mb, 'files': files,
                     'mtime': int(entry.stat().st_mtime),
                     'recording': entry.path == active, **meta})
    rows.sort(key=lambda r: r['mtime'], reverse=True)
    return rows


def classify(type_name):
    """'image', 'heavy' or '' -- what the picker needs to warn about."""
    if type_name in IMAGE_TYPES:
        return 'image'
    return 'heavy' if type_name in HEAVY_TYPES else ''


def main():
    """`python3 scripts/gcs/bagger.py` -- what is in bags/, from a terminal."""
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(os.path.dirname(here))
    print(f'{bag_root(repo)}   ({free_gb(repo)} GB free on this filesystem)')
    rows = list_bags(repo)
    if not rows:
        print('  no recordings yet')
        return
    total = 0.0
    for r in rows:
        total += r['size_mb']
        when = time.strftime('%Y-%m-%d %H:%M', time.localtime(r['mtime']))
        state = ('RECORDING' if r['recording']
                 else 'ok' if r['finalized'] else 'UNFINALISED')
        extra = []
        if r['topics']:
            extra.append(f'{r["topics"]} topics')
        if r['messages']:
            extra.append(f'{r["messages"]} msgs')
        if r['duration_s']:
            extra.append(f'{r["duration_s"]}s')
        print(f'  {when}  {r["size_mb"]:>9.1f} MB  {state:<11} {r["name"]}'
              + (f'   [{", ".join(extra)}]' if extra else ''))
    print(f'  {"":>18}{total:>9.1f} MB total')
    if any(not r['finalized'] and not r['recording'] for r in rows):
        print('\nUNFINALISED means metadata.yaml is missing: the recorder was '
              'killed before\nit could write one, and `ros2 bag play` cannot '
              'read the bag without it.')


if __name__ == '__main__':
    main()
