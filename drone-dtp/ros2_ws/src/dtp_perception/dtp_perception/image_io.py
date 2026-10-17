"""Image ingest for the Python perception nodes -- compressed by default, and why.

THE MEASUREMENT THAT DECIDED THIS (drone NX, 2026-09-07)
--------------------------------------------------------
A raw `sensor_msgs/Image` at 1280x720x3 is **2.76 MB**, so 30 Hz is ~83 MB/s. rclpy has to
deserialise every one of those into a Python object before your callback runs, and with
BEST_EFFORT sensor QoS the middleware simply drops whatever the consumer cannot drain.

Measured, in one process, with counter-only callbacks:

    /camera/color/metadata   (tiny)      29.97 Hz    <- control topic, same driver
    /camera/color/image_raw  (2.76 MB)   27.41 Hz    <- 91.3 % of the control

A **C++** subscriber (`ros2 bag record`) took 686 of 686 images over the same period -- zero
loss. So the camera is fine and the transport is fine: the ~8 % loss is the Python consumer.

The compressed topic is **JPEG at 0.17 MB -- 16x smaller, ~5 MB/s** -- and reaches 29.97 Hz in
Python with no loss.

TRADE-OFF, STATED PLAINLY
-------------------------
JPEG is lossy. We accept mild compression artefacts to stop losing ~8 % of frames outright,
because a dropped frame is a total loss of information while a JPEG frame is a slightly degraded
one. Decode costs ~2-4 ms against 13.6 ms inference with 2.7x headroom.

If detection quality is ever suspected, flip `use_compressed` to false and compare -- that is
exactly why this is a parameter and not a hard-coded topic.

WHAT THIS DOES *NOT* FIX
------------------------
`/dtp/points_fused` is a `PointCloud2` in the same size class, and there is **no compressed
transport for point clouds**. A Python consumer of the fused cloud will lose frames the same
way. That path needs a C++ node, a smaller representation, or a deliberate decision to accept
the loss -- see plan/dtp-imp/21.
"""
import numpy as np
from sensor_msgs.msg import CompressedImage, Image

# Appended by image_transport's republisher to the base topic.
COMPRESSED_SUFFIX = '/compressed'


def image_topic_and_type(base_topic, use_compressed=True):
    """Return (topic, msg_type) for the chosen transport.

    Returned as a pair rather than hidden behind a subscribe() helper because
    `perception_node` feeds them to `message_filters.Subscriber` for time synchronisation,
    which needs the type explicitly.
    """
    if use_compressed:
        return base_topic + COMPRESSED_SUFFIX, CompressedImage
    return base_topic, Image


def decode(msg):
    """Decode either message type to a **BGR** uint8 HxWx3 array.

    Always BGR, whichever transport is in use, so a node can switch `use_compressed` without
    a channel-order bug quietly inverting red and blue -- which would not crash, would not look
    obviously wrong on a person, and would degrade detection in a way that is easy to blame on
    the model.
    """
    if isinstance(msg, CompressedImage):
        import cv2
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)   # cv2 returns BGR
        if img is None:
            raise ValueError("cv2.imdecode failed (format=%r, %d bytes)"
                             % (msg.format, len(msg.data)))
        return img

    # Raw Image: reshape in place, then normalise channel order to BGR.
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    try:
        img = buf.reshape(msg.height, msg.width, -1)
    except ValueError:
        raise ValueError("image data %d bytes does not match %dx%d"
                         % (len(msg.data), msg.width, msg.height))
    # ascontiguousarray is NOT decoration. `img[:, :, ::-1]` is a VIEW with a negative stride,
    # and every downstream consumer (ultralytics/torch) has to copy it to a contiguous buffer
    # before it can run. Measured on the drone NX 2026-09-07: returning the bare view made
    # "inference" read 20.5 ms instead of 13.1 ms -- the copy was real, but it was billed to the
    # model instead of to this function, which made a transport A/B look ~7 ms better than it
    # was. Pay it here, once, where it is visible in the decode timing.
    enc = (msg.encoding or '').lower()
    if enc == 'bgr8':
        return np.ascontiguousarray(img[:, :, :3])
    if enc == 'rgb8':
        return np.ascontiguousarray(img[:, :, ::-1])
    if enc == 'bgra8':
        return np.ascontiguousarray(img[:, :, :3])
    if enc == 'rgba8':
        return np.ascontiguousarray(img[:, :, 2::-1])
    raise ValueError("unsupported encoding %r -- expected rgb8/bgr8/rgba8/bgra8" % msg.encoding)


def describe(use_compressed):
    """One-line summary for a node's start-up log, so the transport is never a mystery."""
    if use_compressed:
        return ("compressed (JPEG ~0.17 MB/frame). Chosen because rclpy drops ~8 %% of raw "
                "2.76 MB frames at 30 Hz under BEST_EFFORT; see dtp_perception/image_io.py")
    return ("RAW (2.76 MB/frame). NOTE: measured ~8 %% frame loss in Python at 30 Hz -- the "
            "middleware drops what rclpy cannot deserialise in time")
