#!/usr/bin/env python3
"""Test the camera interlock and the JPEG header parser.

    python3 scripts/gcs/test_camera.py

Needs no ROS, no camera and no running GCS -- camera_blocked() is a pure
function of a state dict, which is exactly why it is one.

What it protects.  A Gemini can be opened by exactly ONE process: a second
reader dies with `uvc_open ... Return Code: -6`.  On this Jetson the camera is
routinely already owned by another launch under /camera, so "Start" has to
mean "start it if it is free", and getting that wrong takes somebody else's
perception stack down in the middle of a run.  The same decision is rendered
twice -- as the panel's Start button and as the Stack row's -- so it lives in
one function and is tested here rather than trusted twice.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import camera                  # noqa: E402
import server                  # noqa: E402

fails = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}' + (f'  -- {detail}' if detail else ''))
    if not cond:
        fails.append(name)


DEV = [{'name': 'Orbbec Gemini 336L', 'serial': 'CPC64630007M',
        'id': '2bc5:0807', 'speed': '5000', 'node': '/dev/bus/usb/002/021'}]
STOPPED = {'key': 'camera', 'state': 'stopped'}
RUNNING = {'key': 'camera', 'state': 'running'}


def state(**kw):
    base = {'devices': DEV, 'source': None, 'holders': []}
    base.update(kw)
    return base


print('camera_blocked')
check('a free camera may be started',
      server.camera_blocked(state(), STOPPED) is None)

why = server.camera_blocked(state(devices=[]), STOPPED)
check('no camera on USB is refused, and says how to check',
      why is not None and 'lsusb' in why, why)

why = server.camera_blocked(state(source='/camera/color/image_raw/compressed'),
                            STOPPED)
check('a camera already publishing elsewhere is refused', why is not None, why)
check('...naming the topic, so you know what to watch instead',
      why and '/camera/color/image_raw/compressed' in why, why)

why = server.camera_blocked(
    state(holders=[{'pid': 4242, 'cmd': '/usr/bin/some_sdk_tool --foo'}]),
    STOPPED)
check('a USB device held with nothing on ROS is refused', why is not None, why)
check('...naming the pid, because that is what you have to stop',
      why and '4242' in why, why)

check('a camera we already started is refused',
      server.camera_blocked(state(source='/gemini/color/image_raw/compressed'),
                            RUNNING) is not None)
# The order matters: with our own camera up, BOTH rules fire, and "already
# running here" is the useful sentence -- "another process owns it" would send
# you hunting for a process that is our own.
check('...as "already running here", not as a foreign owner',
      'already running here' in server.camera_blocked(
          state(source='/gemini/color/image_raw/compressed'), RUNNING))

# The DTP follower opens the Gemini itself. While it is still starting there is
# no publisher to find yet, so the row -- not the topic -- has to refuse.
why = server.camera_blocked(state(), STOPPED, {'key': 'dtp', 'state': 'running'})
check('a running DTP follower refuses the camera even before it publishes',
      why is not None and 'follower' in why, why)
check('a stopped follower does not',
      server.camera_blocked(state(), STOPPED, {'key': 'dtp', 'state': 'stopped'})
      is None)

print('\npick_source  (start-the-driver vs watch-what-is-there)')
OURS = {'topic': '/gemini/color/image_raw/compressed', 'publishers': 1, 'ours': True}
THEIRS = {'topic': '/camera/color/image_raw/compressed', 'publishers': 1, 'ours': False}

check('nothing publishing -> no source', camera.pick_source([]) is None)
check('the only publisher is the source',
      camera.pick_source([THEIRS]) == THEIRS['topic'])
check('ours wins when both are up',
      camera.pick_source([OURS, THEIRS]) == OURS['topic'])
check('a pin beats the automatic choice',
      camera.pick_source([OURS, THEIRS], THEIRS['topic']) == THEIRS['topic'])
# The panel would otherwise sit on a dead topic for ever after the driver it
# was pinned to went away.
check('a pin whose publisher is gone falls back rather than showing nothing',
      camera.pick_source([THEIRS], OURS['topic']) == THEIRS['topic'])
check('a pin with nothing publishing at all is still None',
      camera.pick_source([], OURS['topic']) is None)

print('\njpeg_size')
# A 1x1 JPEG, baseline: SOI, APP0, SOF0 (h=1 w=1), and enough to stop scanning.
TINY = bytes.fromhex(
    'ffd8ffe000104a46494600010100000100010000'
    'ffdb004300ffffffffffffffffffffffffffffffffffffffffffffffffffffffff'
    'ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff'
    'ffffffffffffff'
    'ffc0000b08000100010101110000'
    'ffda0008010100003f00d2cf20')
check('reads the SOF frame size', camera.jpeg_size(TINY) == (1, 1),
      str(camera.jpeg_size(TINY)))
check('a non-JPEG is None, not a guess', camera.jpeg_size(b'\x89PNG\r\n') is None)
check('an empty payload is None', camera.jpeg_size(b'') is None)
check('a truncated header is None, not an exception',
      camera.jpeg_size(TINY[:14]) is None)
# A stream that reached the scan without a SOF is corrupt; returning a size
# from whatever bytes followed would put a made-up resolution on the panel.
check('no SOF before the scan is None',
      camera.jpeg_size(b'\xff\xd8' + b'\xff\xda\x00\x08' + b'\x00' * 20) is None)

print('\nUSB probe (live, on this machine)')
devs = camera.usb_devices()
print(f'  found {len(devs)} Orbbec device(s): '
      + ', '.join(f'{d["name"]} {d["id"]} at {d["node"]}' for d in devs))
check('the probe returns well-formed rows, or none at all',
      all(d.get('id', '').startswith(camera.ORBBEC_VENDOR + ':') for d in devs))

print('\n' + (f'{len(fails)} FAILED: ' + ', '.join(fails) if fails else 'all passed'))
sys.exit(1 if fails else 0)
