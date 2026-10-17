"""dtp_image_overlay -- the camera picture with perception's person boxes on it, for RViz.

WHY THIS NODE HAS TO EXIST
--------------------------
dtp_viz draws what the stack believes in 3-D, but nothing showed the one thing an operator checks
first: is the box actually on the person? perception_node publishes its boxes on
/dtp/detections_2d (JSON, colour-image pixels, the colour frame's own stamp) and this node draws
them onto that frame and publishes a JPEG:

    /dtp/image_annotated/compressed   sensor_msgs/CompressedImage, BEST_EFFORT, depth 1

A JPEG, not a raw Image, because the viewer is a laptop on field Wi-Fi: a raw 720p frame is
2.7 MB, while this is ~50-80 kB at 8 Hz. RViz on Humble cannot display a CompressedImage, so
scripts/laptop_image_decode.py turns it back into a raw Image ON THE LAPTOP, where it costs no
network at all. `DTP=1 bash scripts/laptop_viewer.sh` starts both it and RViz.

EVERY BOX IS DRAWN ON THE PICTURE IT WAS COMPUTED FROM
-----------------------------------------------------
Boxes arrive 100-300 ms after their frame (TF wait + cloud + inference). Drawing the newest boxes
on the newest frame would put every box that far behind a walking person, which looks exactly
like a tracking fault. So the last few seconds of JPEGs are kept keyed by header stamp, and each
detection message is drawn on the frame with the SAME stamp. The picture therefore runs one
perception latency behind real time, at the detection rate -- which is the honest picture of what
the follower knows. While perception is silent, the live camera is published at live_fps under an
amber banner saying there are no boxes, so an empty picture never reads as "nobody is here".

IT COSTS NOTHING UNLESS SOMEONE IS WATCHING
-------------------------------------------
Passive, like dtp_viz: nothing in the flight path subscribes to its output. And it subscribes to
nothing -- not even the camera -- until its output topic has a subscriber, and drops everything
IDLE_S after the last one leaves. While watched: the camera subscription is raw (the stamp and
JPEG are read straight out of the serialised message, no rclpy message is built for a 170 kB
frame), and only frames that are actually shown are decoded, drawn and re-encoded -- one per
detection, i.e. 8 Hz.
"""
import array
import json
import struct
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from dtp_msgs.msg import Target

FRAME_KEEP_S = 3.0          # boxes arrive <= ~0.5 s after their frame; 3 s is ample
FRAME_KEEP_MAX = 120        # 4 s at 30 Hz: binds only if stamps stop advancing
MATCH_TOL_S = 0.040         # past about one 30 Hz frame period it is a different picture
MATCH_WAIT_S = 0.30         # boxes whose frame has not arrived yet wait this long, then drop
DET_LIVE_S = 1.0            # no boxes for this long -> publish the live camera instead
IDLE_S = 5.0                # no subscriber for this long -> drop every subscription
RATE_WINDOW_S = 4.0
FOLLOW_TARGET_TTL_S = 0.5   # dtp_viz uses 0.35; the lock is re-published every detection
LOCK_STATE_TTL_S = 3.0      # target_manager heartbeats /dtp/lock_state at 1 Hz

# BEST_EFFORT everywhere. A best-effort reader matches reliable and best-effort writers alike,
# while a RELIABLE reader silently never matches a best-effort writer
# (docs/fact-super-cmd-stream.md).
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=5)
OUT_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)

# BGR. Same meanings as dtp_viz: locked green, others blue/grey, anything the range gate refused
# amber, a blind pipeline red.
C_LOCKED = (94, 204, 46)
C_TRACKED = (255, 180, 90)
C_TENTATIVE = (200, 200, 200)
C_HELD = (40, 170, 240)
C_PATCH = (220, 120, 255)
C_BAD = (80, 80, 235)
C_INK = (20, 20, 20)
C_TEXT = (240, 240, 240)
FONT = cv2.FONT_HERSHEY_SIMPLEX


def split_compressed_image(raw):
    """(stamp_ns, frame_id, data_offset, data_len) from a SERIALISED sensor_msgs/CompressedImage.

    CDR: a 4-byte encapsulation header (byte 1 odd = little-endian), header.stamp (int32 sec,
    uint32 nanosec), header.frame_id (string), format (string), data (uint8[]). A string or
    sequence is a uint32 length -- a string's counts its NUL -- aligned to 4 relative to the end
    of the encapsulation header, then that many bytes. test_dtp_image_overlay.py checks this
    against rclpy's own serialiser.
    """
    if len(raw) < 16 or raw[0] != 0:
        raise ValueError('not a CDR payload')
    end = '<' if raw[1] & 1 else '>'
    base = 4
    sec, nsec = struct.unpack_from(end + 'iI', raw, base)

    def sequence(off):
        off = base + ((off - base + 3) & ~3)
        (n,) = struct.unpack_from(end + 'I', raw, off)
        off += 4
        if off + n > len(raw):
            raise ValueError('truncated CompressedImage')
        return off, n

    off, n = sequence(base + 8)                              # header.frame_id
    frame_id = bytes(raw[off:off + max(0, n - 1)]).decode('utf-8', 'replace')
    off, n = sequence(off + n)                               # format
    off, n = sequence(off + n)                               # data
    return sec * 1_000_000_000 + nsec, frame_id, off, n


def _dashed_rect(img, p1, p2, color, thickness, dash=10, gap=7):
    (x1, y1), (x2, y2) = p1, p2
    for a, b, horizontal in (((x1, y1), (x2, y1), True), ((x1, y2), (x2, y2), True),
                             ((x1, y1), (x1, y2), False), ((x2, y1), (x2, y2), False)):
        length = abs((b[0] - a[0]) if horizontal else (b[1] - a[1]))
        pos = 0
        while pos < length:
            q = min(pos + dash, length)
            if horizontal:
                cv2.line(img, (a[0] + pos, a[1]), (a[0] + q, a[1]), color, thickness)
            else:
                cv2.line(img, (a[0], a[1] + pos), (a[0], a[1] + q), color, thickness)
            pos = q + gap


def _label(img, text, x, y, bg, scale, above=True):
    """Filled tag with dark text; placed above (x, y), or below it if that runs off the top."""
    (tw, th), base = cv2.getTextSize(text, FONT, scale, 1)
    pad = max(3, int(round(5 * scale)))
    h = th + base + 2 * pad
    top = y - h if above and y - h >= 0 else y
    x = int(max(0, min(x, img.shape[1] - tw - 2 * pad)))
    cv2.rectangle(img, (x, top), (x + tw + 2 * pad, top + h), bg, -1)
    cv2.putText(img, text, (x + pad, top + pad + th), FONT, scale, C_INK, 1, cv2.LINE_AA)


def _banner(img, lines, color, scale):
    """A dark translucent band across the top, one line of text per entry, coloured edge."""
    (_, th), base = cv2.getTextSize('Ag', FONT, scale, 1)
    lh = th + base + int(round(8 * scale))
    h = lh * len(lines) + int(round(8 * scale))
    band = img[0:h].copy()
    cv2.rectangle(band, (0, 0), (img.shape[1], h), (12, 12, 12), -1)
    img[0:h] = cv2.addWeighted(band, 0.72, img[0:h], 0.28, 0)
    cv2.rectangle(img, (0, h - 3), (img.shape[1], h), color, -1)
    for i, text in enumerate(lines):
        cv2.putText(img, text, (int(10 * scale), int(4 * scale) + lh * (i + 1) - base),
                    FONT, scale, C_TEXT, 1, cv2.LINE_AA)


def render(jpeg, det, info, out_width, quality):
    """Decode, draw `det` (a /dtp/detections_2d payload, or None for a live frame), re-encode.

    `info` carries what is drawn around the boxes: 'locked' (track id or None), 'lock' and 'mux'
    strings, 'rate' (Hz), 'skew_ms', 'coasting' (id or None). Returns JPEG bytes, or None if the
    input would not decode.
    """
    # Asking the JPEG decoder for half size is ~2x cheaper than decoding full and shrinking, and
    # exact for any output at or below half the camera's width (output_width:=640 on 1280).
    src_w = float((det or {}).get('w') or 1280)
    half = bool(out_width) and out_width * 2 <= src_w
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8),
                       cv2.IMREAD_REDUCED_COLOR_2 if half else cv2.IMREAD_COLOR)
    if img is None:
        return None
    h0, w0 = img.shape[:2]
    if half:
        h0, w0 = h0 * 2, w0 * 2                  # boxes are in full-size pixels
    if out_width and img.shape[1] > out_width:
        img = cv2.resize(img, (int(out_width), int(round(h0 * out_width / w0))),
                         interpolation=cv2.INTER_LINEAR)
    H, W = img.shape[:2]
    scale = W / 1280.0 * 0.9 + 0.15              # text size that reads at 640..1280 px wide
    thick = max(2, int(round(W / 400)))

    if det is None:
        _banner(img, ['LIVE CAMERA - perception is not publishing boxes',
                      '  (no /dtp/detections_2d: perception_node not running, or warming up)'],
                C_HELD, scale)
        return _encode(img, quality)

    sx = W / float(det.get('w') or w0)
    sy = H / float(det.get('h') or h0)
    locked = info.get('locked')
    dets = sorted(det.get('dets') or [], key=lambda d: d.get('id') == locked)  # locked on top

    for d in dets:
        box = d.get('box')
        if not box or any(v is None for v in box):
            continue
        x1, y1 = int(round(box[0] * sx)), int(round(box[1] * sy))
        x2, y2 = int(round(box[2] * sx)), int(round(box[3] * sy))
        is_locked = locked is not None and d.get('id') == locked
        color = C_LOCKED if is_locked else (C_TRACKED if d.get('confirmed') else C_TENTATIVE)
        if is_locked:
            roi = img[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
            if roi.size:
                tint = np.full_like(roi, C_LOCKED)
                img[max(0, y1):max(0, y2), max(0, x1):max(0, x2)] = cv2.addWeighted(
                    roi, 0.85, tint, 0.15, 0)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, thick + (1 if is_locked else 0))
        if d.get('held'):
            inset = thick + 3
            _dashed_rect(img, (x1 + inset, y1 + inset), (x2 - inset, y2 - inset), C_HELD,
                         max(1, thick - 1))
        patch = d.get('patch')
        if patch and patch[0] is not None and patch[1] is not None:
            px, py = int(round(patch[0] * sx)), int(round(patch[1] * sy))
            r = max(4, thick * 3)
            cv2.circle(img, (px, py), r, C_PATCH, max(1, thick - 1), cv2.LINE_AA)
            cv2.line(img, (px - 2 * r, py), (px - r, py), C_PATCH, max(1, thick - 1))
            cv2.line(img, (px + r, py), (px + 2 * r, py), C_PATCH, max(1, thick - 1))

        parts = ['#%d' % d.get('id', -1)]
        if is_locked:
            parts.append('LOCKED')
        if d.get('dist') is not None:
            parts.append('%.1f m' % d['dist'])
        if d.get('conf') is not None:
            parts.append('%d%%' % round(d['conf'] * 100))
        if d.get('held'):
            parts.append('range held')
        _label(img, '  '.join(parts), x1 - thick // 2, y1 - thick, color, scale * 0.85)

    n = len(dets)
    top = ['LOCK %s   MUX %s' % (info.get('lock') or '-', info.get('mux') or '-')]
    second = ['%d %s' % (n, 'person' if n == 1 else 'people')]
    if info.get('rate'):
        second.append('detect %.1f Hz' % info['rate'])
    if det.get('age_s') is not None:
        second.append('%d ms behind' % round(det['age_s'] * 1000))
    if det.get('infer_ms') is not None:
        second.append('YOLO %d ms' % round(det['infer_ms']))
    if info.get('skew_ms'):
        second.append('nearest frame, %d ms off' % round(info['skew_ms']))
    if info.get('coasting') is not None:
        second.append('COASTING #%d - not in view' % info['coasting'])
    lines = top + ['   '.join(second)]
    edge = C_LOCKED if locked is not None else (60, 60, 60)
    if det.get('ok') is False:
        lines = ['PERCEPTION BLIND this frame', '  ' + str(det.get('reason', ''))[:110]] + top
        edge = C_BAD
    elif info.get('coasting') is not None:
        edge = C_HELD
    _banner(img, lines, edge, scale)
    return _encode(img, quality)


def _encode(img, quality):
    ok, enc = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    return enc.tobytes() if ok else None


def _short_state(text, keep_track):
    """'LOCKED|3|detail' -> 'LOCKED #3'; 'FOLLOW|OK|detail' -> 'FOLLOW OK'."""
    parts = (text or '').split('|')
    if not parts or not parts[0]:
        return None
    if len(parts) > 1 and parts[1] and parts[1] != '-':
        return '%s %s%s' % (parts[0], '#' if keep_track else '', parts[1])
    return parts[0]


class DtpImageOverlay(Node):

    def __init__(self):
        super().__init__('dtp_image_overlay')
        self.declare_parameter('image_topic', '/camera/color/image_raw/compressed')
        self.declare_parameter('detections_topic', '/dtp/detections_2d')
        self.declare_parameter('output_topic', '/dtp/image_annotated/compressed')
        # 960 px keeps labels readable in an RViz panel at ~50-80 kB a frame; 0 = camera size.
        self.declare_parameter('output_width', 960)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('live_fps', 4.0)
        # Subscribe and draw even with no viewer -- for a bag recording of the annotated picture.
        self.declare_parameter('always_on', False)

        def g(name):
            return self.get_parameter(name).value

        self._image_topic = str(g('image_topic'))
        self._dets_topic = str(g('detections_topic'))
        self._out_width = int(g('output_width'))
        self._quality = int(g('jpeg_quality'))
        live_fps = float(g('live_fps'))
        self._live_period = 1.0 / live_fps if live_fps > 0 else float('inf')
        self._always_on = bool(g('always_on'))

        self._pub = self.create_publisher(CompressedImage, str(g('output_topic')), OUT_QOS)
        self._subs = []
        self._last_wanted = 0.0
        self._reset()
        self.create_timer(1.0, self._retune)
        self.get_logger().info(
            'dtp_image_overlay up: %s + %s -> %s (%s px, q%d). Idle until something subscribes.'
            % (self._image_topic, self._dets_topic, g('output_topic'),
               self._out_width or 'camera', self._quality))

    def _reset(self):
        self._frames = deque(maxlen=FRAME_KEEP_MAX)   # (stamp_ns, arrival, raw, off, n, frame_id)
        self._pending = None                          # (arrival, payload, stamp_ns)
        self._last_det = 0.0
        self._det_arrivals = deque(maxlen=64)
        self._last_live = 0.0
        self._lock = None                             # (arrival, text)
        self._mux = None
        self._follow = None                           # (arrival, track_id)

    # -- demand ------------------------------------------------------------------------------
    def _retune(self):
        now = time.monotonic()
        if self._always_on or self._pub.get_subscription_count() > 0:
            self._last_wanted = now
        wanted = now - self._last_wanted < IDLE_S
        if wanted and not self._subs:
            self._subs = [
                self.create_subscription(CompressedImage, self._image_topic, self._on_image,
                                         SENSOR_QOS, raw=True),
                self.create_subscription(String, self._dets_topic, self._on_dets, SENSOR_QOS),
                self.create_subscription(String, '/dtp/lock_state', self._on_lock, SENSOR_QOS),
                self.create_subscription(String, '/mux/state', self._on_mux, SENSOR_QOS),
                self.create_subscription(Target, '/dtp/follow_target', self._on_follow,
                                         SENSOR_QOS, raw=True),
            ]
            self.get_logger().info('a viewer subscribed -- drawing')
        elif not wanted and self._subs:
            for sub in self._subs:
                self.destroy_subscription(sub)
            self._subs = []
            self._reset()             # never show the next viewer a picture from minutes ago
            self.get_logger().info('no viewer for %.0f s -- idle' % IDLE_S)

    # -- inputs ------------------------------------------------------------------------------
    def _on_lock(self, msg):
        self._lock = (time.monotonic(), msg.data)

    def _on_mux(self, msg):
        self._mux = (time.monotonic(), msg.data)

    def _on_follow(self, raw):
        try:
            self._follow = (time.monotonic(), int(deserialize_message(raw, Target).track_id))
        except Exception:                                               # noqa: BLE001
            pass

    def _on_image(self, raw):
        now = time.monotonic()
        try:
            stamp_ns, frame_id, off, n = split_compressed_image(raw)
        except (ValueError, struct.error):
            return
        self._frames.append((stamp_ns, now, raw, off, n, frame_id))
        while self._frames and now - self._frames[0][1] > FRAME_KEEP_S:
            self._frames.popleft()

        if self._pending is not None:
            self._try_pending(now)
        if now - self._last_det > DET_LIVE_S and now - self._last_live >= self._live_period:
            self._last_live = now
            self._publish(raw[off:off + n], None, stamp_ns, frame_id, {})

    def _on_dets(self, msg):
        now = time.monotonic()
        try:
            payload = json.loads(msg.data)
            sec, nsec = payload['stamp']
            stamp_ns = int(sec) * 1_000_000_000 + int(nsec)
        except Exception:                                               # noqa: BLE001
            self.get_logger().warn('unreadable %s message' % self._dets_topic,
                                   throttle_duration_sec=5.0)
            return
        self._last_det = now
        self._det_arrivals.append(now)
        # A newer detection replaces an older one still waiting for its frame: showing the latest
        # boxes matters more than showing every one.
        self._pending = (now, payload, stamp_ns)
        self._try_pending(now)

    # -- drawing -----------------------------------------------------------------------------
    def _match(self, stamp_ns):
        best, best_skew = None, None
        for fr in reversed(self._frames):
            skew = abs(fr[0] - stamp_ns) * 1e-9
            if best_skew is None or skew < best_skew:
                best, best_skew = fr, skew
                if skew == 0.0:
                    break
        if best is None or best_skew > MATCH_TOL_S:
            return None, None
        return best, best_skew

    def _try_pending(self, now):
        arrival, payload, stamp_ns = self._pending
        fr, skew = self._match(stamp_ns)
        if fr is None:
            if now - arrival > MATCH_WAIT_S:
                self._pending = None          # its frame never reached us; never draw it on
            return                            # a different picture
        self._pending = None
        stamp, _arr, raw, off, n, frame_id = fr
        self._publish(raw[off:off + n], payload, stamp, frame_id,
                      {'skew_ms': skew * 1e3 if skew else 0.0})

    def _info(self, now):
        info = {}
        locked = None
        if self._follow and now - self._follow[0] < FOLLOW_TARGET_TTL_S:
            locked = self._follow[1]
        lock = self._lock if self._lock and now - self._lock[0] < LOCK_STATE_TTL_S else None
        if lock:
            parts = lock[1].split('|')
            if locked is None and parts[0] in ('LOCKED', 'COASTING') and len(parts) > 1 \
                    and parts[1].isdigit():
                locked = int(parts[1])
            info['lock'] = _short_state(lock[1], keep_track=True)
            if parts[0] == 'COASTING' and len(parts) > 1 and parts[1].isdigit():
                info['coasting'] = int(parts[1])
        mux = self._mux if self._mux and now - self._mux[0] < LOCK_STATE_TTL_S else None
        if mux:
            info['mux'] = _short_state(mux[1], keep_track=False)
        info['locked'] = locked
        while self._det_arrivals and now - self._det_arrivals[0] > RATE_WINDOW_S:
            self._det_arrivals.popleft()
        span = (self._det_arrivals[-1] - self._det_arrivals[0]) if len(self._det_arrivals) > 1 \
            else 0.0
        info['rate'] = (len(self._det_arrivals) - 1) / span if span > 0.05 else 0.0
        return info

    def _publish(self, jpeg, det, stamp_ns, frame_id, extra):
        info = self._info(time.monotonic())
        info.update(extra)
        if det is not None and info.get('coasting') is not None and any(
                d.get('id') == info['coasting'] for d in det.get('dets') or []):
            info['coasting'] = None           # seen in this very frame after all
        try:
            data = render(jpeg, det, info, self._out_width, self._quality)
        except Exception as exc:                                        # noqa: BLE001
            self.get_logger().warn('render failed: %s: %s' % (type(exc).__name__, exc),
                                   throttle_duration_sec=5.0)
            return
        if data is None:
            return
        out = CompressedImage()
        out.header.stamp.sec = int(stamp_ns // 1_000_000_000)
        out.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        out.header.frame_id = frame_id
        out.format = 'jpeg'
        # array.array, NOT bytes: rclpy's uint8[] setter validates a bytes value one element at a
        # time in Python -- two loops over ~80k items, which measured 40 % of a core here.
        out.data = array.array('B', data)
        self._pub.publish(out)


def main(args=None):
    # One OpenCV thread: on these ~1 MP images the pool's hand-off costs more than it saves --
    # 22.6 ms of CPU per frame with 8 threads against 17.6 ms with one, measured on this NX.
    cv2.setNumThreads(1)
    rclpy.init(args=args)
    node = None
    try:
        node = DtpImageOverlay()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
