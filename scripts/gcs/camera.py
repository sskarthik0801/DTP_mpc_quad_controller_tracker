#!/usr/bin/env python3
"""Live camera on the dashboard: Gemini 336L JPEG frames, ROS 2 -> browser.

The Orbbec driver already publishes the camera's own MJPEG straight through as
sensor_msgs/CompressedImage (docs/geminitest.md), so a frame can go from DDS to
an <img> tag with **no decode and no re-encode** -- the bytes are copied, never
touched.  That is the whole reason this is affordable on a Jetson that has no
web framework, no npm and no spare core: `cv2.imdecode` + re-encode per frame
would cost more than the flight stack.

Two rules carried over from telemetry.py, for the same reason (rclpy costs
~1.6 ms of Python per message here):

  1. The image subscription exists ONLY while a browser is actually looking at
     the feed.  A 30 Hz 720p stream is ~5% of a core and ~40 Mbit/s of Wi-Fi;
     paying that for a tab nobody has open is how the GCS got to 28% of a core
     the first time round.
  2. Nothing is deserialised anywhere else.  Discovery, publisher counts and
     the USB probe answer "is the camera free?" without a subscription at all.

The USB half is stdlib-only sysfs/procfs, so it still answers with --no-ros:
it is what lets the dashboard say *why* Start is unavailable instead of just
greying a button out.

  IMPORTANT, and the reason this module exists rather than just a Start button:
  a Gemini can only be opened by ONE process.  A second reader fails with
  `uvc_open ... Return Code: -6` (busy).  On this Jetson the camera is
  routinely already owned by another launch (the /camera namespace), so the
  dashboard must detect that and offer to VIEW the existing stream rather than
  launch a second driver on top of it and kill someone else's session.
"""
import os
import time
import threading
from collections import deque

# rclpy is absent in --no-ros mode and on a machine with no ROS sourced.  The
# sysfs/procfs half below must still work there -- it is what renders the
# "no camera on USB" / "held by pid N" reasons.  CameraFeed is only ever
# constructed from telemetry.py, which has already imported rclpy.
try:
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from sensor_msgs.msg import CompressedImage
except ImportError:                                   # pragma: no cover
    Node = object
    CompressedImage = None

ORBBEC_VENDOR = '2bc5'          # Orbbec 3D Technology International
COMPRESSED = 'sensor_msgs/msg/CompressedImage'
OUR_NS = '/gemini/'             # what super_bringup/launch/gemini_camera.launch.py uses
COLOR_SUFFIX = '/color/image_raw/compressed'

DISCOVERY_TTL = 2.0             # graph queries are cheap but not free
HOLDER_TTL = 3.0                # a /proc/*/fd walk is ~8000 readlinks
VIEW_TTL = 6.0                  # no viewer for this long -> drop the subscription
RATE_WINDOW = 4.0

# Image QoS is BEST_EFFORT/VOLATILE (docs/geminitest.md).  A default (RELIABLE)
# subscription would silently never match and never error -- the same trap as
# /planning/pos_cmd in docs/fact-super-cmd-stream.md.  Depth 1 on purpose: a
# queue here only ever buys latency, and a frame we are too slow for is a frame
# we should drop, not show late.
IMAGE_QOS = None
if CompressedImage is not None:
    IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE,
                           history=HistoryPolicy.KEEP_LAST, depth=1)


# --------------------------------------------------------------------------
# JPEG header, for reporting the resolution the camera is actually sending.
# --------------------------------------------------------------------------

def jpeg_size(buf):
    """(width, height) from a JPEG's SOF marker, or None.

    Worth the 20 lines: the profile the driver negotiated is not always the one
    it was asked for, and "640x480" on the panel is the only place that shows
    up without opening the launch log.  It also proves the payload really is a
    JPEG rather than something we are about to hand a browser and blame on the
    network.
    """
    if not buf or len(buf) < 4 or buf[0] != 0xFF or buf[1] != 0xD8:
        return None
    i, n = 2, len(buf)
    while i + 9 < n:
        if buf[i] != 0xFF:
            i += 1
            continue
        marker = buf[i + 1]
        if marker == 0xFF:                       # fill byte
            i += 1
            continue
        if marker == 0xD8 or marker == 0x01 or 0xD0 <= marker <= 0xD7:
            i += 2
            continue
        if marker == 0xDA:                       # start of scan: no header left
            return None
        seg = int.from_bytes(buf[i + 2:i + 4], 'big')
        # SOF0..SOF15 carry the frame size; DHT/JPG/DAC share the range.
        if 0xC0 <= marker <= 0xCF and marker not in (0xC4, 0xC8, 0xCC):
            return (int.from_bytes(buf[i + 7:i + 9], 'big'),
                    int.from_bytes(buf[i + 5:i + 7], 'big'))
        if seg < 2:
            return None
        i += 2 + seg
    return None


# --------------------------------------------------------------------------
# USB: is a Gemini plugged in, and is anything already holding it?
# --------------------------------------------------------------------------

def _sysfs(path, name):
    try:
        with open(os.path.join(path, name)) as f:
            return f.read().strip()
    except OSError:
        return None


def usb_devices(vendor=ORBBEC_VENDOR):
    """Every Orbbec camera on USB, from sysfs.  No lsusb, no root."""
    out = []
    root = '/sys/bus/usb/devices'
    try:
        entries = sorted(os.listdir(root))
    except OSError:
        return out
    for name in entries:
        path = os.path.join(root, name)
        if _sysfs(path, 'idVendor') != vendor:
            continue
        bus, dev = _sysfs(path, 'busnum'), _sysfs(path, 'devnum')
        try:
            node = f'/dev/bus/usb/{int(bus):03d}/{int(dev):03d}'
        except (TypeError, ValueError):
            node = None
        out.append({
            'name': _sysfs(path, 'product') or 'Orbbec camera',
            'serial': _sysfs(path, 'serial'),
            'id': f'{vendor}:{_sysfs(path, "idProduct")}',
            'speed': _sysfs(path, 'speed'),        # 5000 = USB 3; 480 = USB 2
            'node': node,
        })
    return out


_holders = {'t': 0.0, 'rows': []}


def usb_holders(nodes, ttl=HOLDER_TTL):
    """pids with one of these /dev/bus/usb nodes open.

    This is `fuser /dev/bus/usb/<bus>/<dev>` without shelling out -- the check
    docs/geminitest.md tells you to run by hand when a launch dies with
    `uvc_open ... Return Code: -6`.

    CAVEAT, and it is in CLAUDE.md for a reason: as an unprivileged user a
    /proc/*/fd walk cannot see another user's fds, so an empty list means "no
    process OF OURS holds it", not "nothing holds it".  Callers must not
    promise the camera is free on the strength of this alone -- which is why
    the publisher count is checked first and this is only the tiebreak.
    """
    nodes = {n for n in nodes if n}
    if not nodes:
        return []
    now = time.monotonic()
    if now - _holders['t'] < ttl:
        return _holders['rows']
    rows = []
    me = os.getpid()
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) == me:
            continue
        fddir = f'/proc/{pid}/fd'
        try:
            fds = os.listdir(fddir)
        except OSError:
            continue                    # gone, or another user's -- see above
        for fd in fds:
            try:
                target = os.readlink(os.path.join(fddir, fd))
            except OSError:
                continue
            if target in nodes:
                try:
                    with open(f'/proc/{pid}/cmdline', 'rb') as f:
                        argv = f.read().decode('utf-8', 'replace').split('\0')
                except OSError:
                    argv = []
                rows.append({'pid': int(pid),
                             'cmd': ' '.join(t for t in argv if t)[:160]})
                break
    _holders.update(t=now, rows=rows)
    return rows


# --------------------------------------------------------------------------
# The ROS side
# --------------------------------------------------------------------------

def pick_source(rows, pinned=None):
    """Which topic to show, given what is publishing and what the user pinned.

    Two separate paths meet here and they must not be confused:

      * START THE DRIVER -- we launch the camera and it publishes under
        /gemini.  That is the 'ours' row, and it wins by default.
      * WATCH WHAT IS ALREADY THERE -- somebody else's driver is publishing
        under /camera (or any other namespace).  Nothing of ours is running;
        we are a subscriber like any other.

    A pinned topic wins over both, because "show me that one" is an explicit
    instruction -- but only WHILE IT IS PUBLISHING.  When it stops, this falls
    back to the automatic choice rather than showing a dead panel, and
    status() reports the pin as no longer live so the page can say so.
    """
    live = [r['topic'] for r in rows]
    if pinned and pinned in live:
        return pinned
    return rows[0]['topic'] if rows else None


class CameraFeed(Node):
    """Latest JPEG frame from whichever camera topic is live, on demand."""

    def __init__(self, name='gcs_camera'):
        super().__init__(name)
        self._cond = threading.Condition()
        self._frame = None          # (seq, bytes, monotonic, content_type)
        self._seq = 0
        self._viewers = 0
        self._last_view = 0.0
        self._sub = None
        self._sub_topic = None
        self._arrivals = deque(maxlen=256)
        self._bytes = deque(maxlen=256)
        self._size_cache = (None, None)          # (seq, (w, h))
        self._disc = {'t': 0.0, 'rows': []}
        self._pinned = None         # topic the user chose, if ever
        # Same reason as telemetry.py's _retune: subscriptions may only be
        # created and destroyed on the executor thread, never from an HTTP one.
        self.create_timer(0.5, self._retune)

    # -- discovery ---------------------------------------------------------
    def sources(self, ttl=DISCOVERY_TTL):
        """Live CompressedImage topics, best candidate first.

        Filtered by count_publishers(), not just by the topic list: a topic
        listed with zero publishers is another machine's stale discovery cache,
        which CLAUDE.md warns about for exactly this kind of check.
        """
        now = time.monotonic()
        if now - self._disc['t'] < ttl:
            return self._disc['rows']
        rows = []
        try:
            for name, types in self.get_topic_names_and_types():
                if COMPRESSED not in types:
                    continue
                if not (name.endswith(COLOR_SUFFIX) or name.startswith(OUR_NS)):
                    continue
                n = self.count_publishers(name)
                if n:
                    rows.append({'topic': name, 'publishers': n,
                                 'ours': name.startswith(OUR_NS)})
        except Exception:
            rows = self._disc['rows']
        rows.sort(key=lambda r: (0 if r['ours'] else 1, r['topic']))
        self._disc.update(t=now, rows=rows)
        return rows

    def source(self):
        return pick_source(self.sources(), self._pinned)

    def pin(self, topic):
        """Watch this topic specifically, or None to go back to automatic.

        Not validated against what is publishing right now: pinning a topic
        before its driver is up is a reasonable thing to do, and pick_source()
        falls back on its own until it appears.
        """
        with self._cond:
            self._pinned = topic or None
        return self._pinned

    # -- demand-driven subscription ---------------------------------------
    def _retune(self):
        with self._cond:
            wanted = (self._viewers > 0
                      or (time.monotonic() - self._last_view) < VIEW_TTL)
        topic = self.source() if wanted else None
        if self._sub is not None and topic != self._sub_topic:
            self.destroy_subscription(self._sub)
            self._sub, self._sub_topic = None, None
            with self._cond:
                self._frame = None       # never render a stale frame as live
                self._arrivals.clear()
                self._bytes.clear()
                self._cond.notify_all()  # unblock streams so they can retry
        if topic and self._sub is None:
            self._sub = self.create_subscription(
                CompressedImage, topic, self._on_frame, IMAGE_QOS)
            self._sub_topic = topic

    def _on_frame(self, msg):
        fmt = (msg.format or '').lower()
        ctype = 'image/png' if 'png' in fmt else 'image/jpeg'
        now = time.monotonic()
        with self._cond:
            self._seq += 1
            self._frame = (self._seq, bytes(msg.data), now, ctype)
            self._arrivals.append(now)
            self._bytes.append(len(msg.data))
            self._cond.notify_all()

    # -- viewers -----------------------------------------------------------
    class _Viewer:
        def __init__(self, feed):
            self.feed = feed

        def __enter__(self):
            with self.feed._cond:
                self.feed._viewers += 1
                self.feed._last_view = time.monotonic()
            return self.feed

        def __exit__(self, *_exc):
            with self.feed._cond:
                self.feed._viewers -= 1
                self.feed._last_view = time.monotonic()
            return False

    def viewer(self):
        """`with feed.viewer():` keeps the subscription alive for one client."""
        return CameraFeed._Viewer(self)

    def touch(self):
        """Ask for the subscription without holding it open (single snapshot)."""
        with self._cond:
            self._last_view = time.monotonic()

    @property
    def viewers(self):
        with self._cond:
            return self._viewers

    def wait(self, since_seq, timeout):
        """Block for the next frame after since_seq.  None on timeout."""
        deadline = time.monotonic() + timeout
        with self._cond:
            while True:
                f = self._frame
                if f is not None and f[0] != since_seq:
                    return f
                left = deadline - time.monotonic()
                if left <= 0:
                    return None
                self._cond.wait(left)

    # -- status ------------------------------------------------------------
    def status(self):
        sources = self.sources()                 # outside the lock: takes the
        topic = self.source()                    # node's own lock internally
        now = time.monotonic()
        with self._cond:
            frame = self._frame
            while self._arrivals and now - self._arrivals[0] > RATE_WINDOW:
                self._arrivals.popleft()
                if self._bytes:
                    self._bytes.popleft()
            stamps = list(self._arrivals)
            sizes = list(self._bytes)
            viewers = self._viewers
            subscribed = self._sub_topic
        span = (stamps[-1] - stamps[0]) if len(stamps) > 1 else 0.0
        size = None
        if frame is not None:
            if self._size_cache[0] == frame[0]:
                size = self._size_cache[1]
            else:
                size = jpeg_size(frame[1])
                self._size_cache = (frame[0], size)
        return {
            'available': True,
            'source': topic,
            'sources': sources,
            'pinned': self._pinned,
            # A pin that is no longer publishing is not an error -- we fell
            # back -- but the page has to say so, or the panel silently shows
            # a different camera than the one you asked for.
            'pinned_live': bool(self._pinned and self._pinned == topic),
            'subscribed': subscribed,
            'viewers': viewers,
            'fps': round((len(stamps) - 1) / span, 1) if span > 0.05 else 0.0,
            'kb': round(sum(sizes) / len(sizes) / 1024.0, 1) if sizes else None,
            'width': size[0] if size else None,
            'height': size[1] if size else None,
            'frame_age': round(now - frame[2], 2) if frame else None,
        }


def main():
    """`python3 scripts/gcs/camera.py` -- who has the camera, and on what topic.

    The one question worth asking before starting a camera launch, and the
    reason `uvc_open ... Return Code: -6` used to cost twenty minutes.  The USB
    half needs no ROS; the topic half is skipped if rclpy is not importable.
    """
    devices = usb_devices()
    if not devices:
        print('no Orbbec camera on USB (looked for vendor '
              f'{ORBBEC_VENDOR} in /sys/bus/usb/devices)')
    for d in devices:
        speed = f'{d["speed"]} Mbit/s' if d['speed'] else 'unknown speed'
        print(f'{d["name"]}  {d["id"]}  serial={d["serial"]}  {speed}  '
              f'{d["node"]}')
        if d['speed'] and int(d['speed']) < 5000:
            print('  WARNING: not on a USB 3 port -- expect profile failures')

    holders = usb_holders([d['node'] for d in devices], ttl=0.0)
    for h in holders:
        print(f'  held by pid {h["pid"]}: {h["cmd"]}')
    if devices and not holders:
        print('  no process of ours holds the USB device '
              '(another user\'s would not be visible here)')

    if CompressedImage is None:
        print('rclpy not importable -- source the workspace to list topics')
        return 0
    import rclpy
    rclpy.init(args=None)
    node = CameraFeed('gcs_camera_probe')
    try:
        # Discovery is not instant -- 2 s was not always enough on this box,
        # and reporting an empty graph as "nothing is publishing" is the wrong
        # answer twice over: it says the camera is free when it is not.
        rows = []
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            rows = node.sources(ttl=0.0)
            if rows:
                break
        if not rows:
            print('no camera topic is publishing -- the camera is free')
        for r in rows:
            print(f'  publishing: {r["topic"]}  ({r["publishers"]} publisher'
                  f'{"s" if r["publishers"] > 1 else ""}'
                  f'{", ours" if r["ours"] else ""})')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
