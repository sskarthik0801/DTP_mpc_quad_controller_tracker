"""Tests for dtp_image_overlay -- the boxes-on-the-picture view for a laptop RViz.

Two things in it fail silently rather than loudly, and those are what is tested:

  * split_compressed_image reads the stamp and JPEG out of SERIALISED CDR by hand. An alignment
    slip would hand the decoder bytes that are nearly a JPEG. Checked against rclpy's serialiser.
  * The pairing. Boxes must be drawn on the frame with THEIR stamp, not on the newest frame --
    which is what a naive overlay does, and which puts every box a perception latency behind a
    walking person. Boxes whose frame never arrived must be dropped, not drawn on another one.

The node is driven as a bare object (no ROS graph, no executor), like test_target_manager_coast.
"""
import json
import types

import cv2
import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from dtp_perception import dtp_image_overlay as ov

W, H = 1280, 720


def _jpeg(shade):
    img = np.full((H, W, 3), shade, np.uint8)
    ok, enc = cv2.imencode('.jpg', img)
    assert ok
    return enc.tobytes()


def _raw_image(sec, nsec, data, frame_id='camera_color_optical_frame', fmt='jpeg'):
    m = CompressedImage()
    m.header.stamp.sec, m.header.stamp.nanosec = sec, nsec
    m.header.frame_id = frame_id
    m.format = fmt
    m.data = data
    return serialize_message(m)


def _dets(sec, nsec, boxes=((100, 100, 300, 600),), ok=True):
    return String(data=json.dumps({
        'stamp': [sec, nsec], 'ok': ok, 'w': W, 'h': H,
        'dets': [{'id': i + 1, 'box': list(b),
                  'patch': [(b[0] + b[2]) / 2, b[1] + 0.65 * (b[3] - b[1])],
                  'conf': 0.9, 'confirmed': True, 'held': False, 'dist': 3.0}
                 for i, b in enumerate(boxes)],
        'age_s': 0.12, 'infer_ms': 60.0}))


def _bare():
    node = object.__new__(ov.DtpImageOverlay)
    node._out_width = 960
    node._quality = 80
    node._live_period = 0.25
    node._dets_topic = '/dtp/detections_2d'
    node._reset()
    node.sent = []
    node._pub = types.SimpleNamespace(publish=node.sent.append)
    quiet = types.SimpleNamespace(warn=lambda *a, **k: None, info=lambda *a, **k: None)
    node.get_logger = lambda: quiet
    return node


def _decode(msg):
    return cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)


def test_cdr_split_matches_rclpy_for_every_padding():
    payload = b'\xff\xd8' + bytes(range(256)) * 2 + b'\xff\xd9'
    for fid_len in range(0, 9):
        for fmt in ('jpeg', 'png', 'rgb8; jpeg compressed bgr8', 'x'):
            raw = _raw_image(1789279866, 412672846, payload, 'f' * fid_len, fmt)
            stamp, frame_id, off, n = ov.split_compressed_image(raw)
            assert stamp == 1789279866 * 1_000_000_000 + 412672846
            assert frame_id == 'f' * fid_len
            assert raw[off:off + n] == payload


def test_cdr_split_refuses_a_truncated_message():
    raw = _raw_image(1, 2, b'\xff\xd8' + b'x' * 100)
    try:
        ov.split_compressed_image(raw[:-20])
    except ValueError:
        return
    raise AssertionError('a truncated message was accepted')


def test_boxes_are_drawn_on_their_own_frame_not_the_newest():
    node = _bare()
    # Frames at t, t+33 ms, t+66 ms, each a different flat grey so the output says which it was.
    shades = {0: 40, 1: 120, 2: 200}
    for i, shade in shades.items():
        node._on_image(_raw_image(500, i * 33_000_000, _jpeg(shade)))
    node._last_det = node._last_live = 1e18      # suppress the live-camera path for this test
    node.sent.clear()
    node._on_dets(_dets(500, 33_000_000))        # boxes for the MIDDLE frame
    assert len(node.sent) == 1
    out = node.sent[0]
    assert (out.header.stamp.sec, out.header.stamp.nanosec) == (500, 33_000_000)
    img = _decode(out)
    assert img.shape[:2] == (540, 960)           # output_width respected, aspect kept
    # A pixel well away from the box and the banner still carries that frame's shade.
    assert abs(int(img[400, 800, 0]) - shades[1]) < 12


def test_box_is_drawn_where_perception_put_it():
    node = _bare()
    node._on_image(_raw_image(7, 0, _jpeg(0)))
    node._last_live = 1e18
    node._on_dets(_dets(7, 0, boxes=((640, 300, 900, 700),)))
    img = _decode(node.sent[-1])
    # 1280 -> 960 is x0.75: the left edge lands at x=480, between y=225 and y=525.
    rows_with_edge = img[260:500, 478:483].max(axis=(1, 2)) > 100    # edge within +-2 px
    assert rows_with_edge.mean() > 0.9, 'no box edge at the scaled position'
    assert img[260:500, 440:460].max() < 90, 'nothing drawn left of the box edge'
    # Above the range-patch marker (drawn at 0.65 of the box height, y~420 here).
    inside = img[240:380, 500:660]
    assert inside.max() < 90, 'the box interior should stay dark (it is not the locked target)'
    px, py = int(770 * 0.75), int((300 + 0.65 * 400) * 0.75)
    assert img[py - 12:py + 13, px - 12:px + 13].max() > 100, 'no range-patch marker'


def test_boxes_whose_frame_never_arrived_are_dropped():
    node = _bare()
    node._on_image(_raw_image(10, 0, _jpeg(80)))
    node._last_live = 1e18
    node.sent.clear()
    node._on_dets(_dets(99, 0))                   # nowhere near any buffered frame
    assert node.sent == []
    assert node._pending is not None              # waiting, in case the frame is late
    node._pending = (node._pending[0] - 1.0,) + node._pending[1:]   # ...and now too late
    node._on_image(_raw_image(10, 33_000_000, _jpeg(80)))
    assert node.sent == [] and node._pending is None


def test_late_frame_is_still_paired_when_it_arrives():
    node = _bare()
    node._last_live = 1e18
    node._on_dets(_dets(20, 0))                   # boxes first, frame not here yet
    assert node.sent == []
    node._on_image(_raw_image(20, 0, _jpeg(60)))
    assert len(node.sent) == 1
    assert node.sent[0].header.stamp.sec == 20


def test_live_camera_when_perception_is_silent():
    node = _bare()
    node._on_image(_raw_image(30, 0, _jpeg(100)))
    assert len(node.sent) == 1, 'no boxes ever -> the live picture must still be shown'
    node._on_image(_raw_image(30, 33_000_000, _jpeg(100)))
    assert len(node.sent) == 1, 'live frames are rate-limited to live_fps'


def test_blind_frame_is_rendered_not_skipped():
    node = _bare()
    node._on_image(_raw_image(40, 0, _jpeg(100)))
    node.sent.clear()
    node._on_dets(String(data=json.dumps({'stamp': [40, 0], 'ok': False, 'reason': 'TF missing',
                                          'dets': []})))
    assert len(node.sent) == 1


def test_locked_target_comes_from_lock_state():
    node = _bare()
    node._on_lock(String(data='LOCKED|2|tracking'))
    info = node._info(__import__('time').monotonic())
    assert info['locked'] == 2 and info['lock'] == 'LOCKED #2'
    node._on_lock(String(data='COASTING|5|occluded'))
    info = node._info(__import__('time').monotonic())
    assert info['locked'] == 5 and info['coasting'] == 5


def test_output_is_a_valid_compressed_image_message():
    node = _bare()
    node._on_image(_raw_image(50, 0, _jpeg(100)))
    msg = node.sent[-1]
    back = deserialize_message(serialize_message(msg), CompressedImage)
    assert back.format == 'jpeg' and back.header.frame_id == 'camera_color_optical_frame'
    assert bytes(back.data[:2]) == b'\xff\xd8'
