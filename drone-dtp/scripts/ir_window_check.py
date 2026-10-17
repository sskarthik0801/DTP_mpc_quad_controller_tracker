#!/usr/bin/env python3
"""Live brightness meter for the 336L's two IR cameras -- find which physical window is blocked.

WHY THIS EXISTS
---------------
On 2026-09-08 the left IR camera was obscured and camera depth was 99.7 % empty, while RGB looked
perfect and the driver reported no error. Re-taping "clear of the IR windows" did not fix it,
because knowing that `left_ir` is blocked does not tell you WHICH opening on the camera's face
`left_ir` is -- and guessing wrong means re-taping the wrong one.

This closes that loop physically: run it, then pass a finger over each opening in turn and watch
which number drops. That is the left IR window. No datasheet, no guessing at the layout.

WHAT THE NUMBERS MEAN
---------------------
    mean   average brightness. A clear camera in a lit room sits well above 30 and RESPONDS
           strongly when you cover it. A blocked one barely moves -- it is already seeing only
           diffuse light.
    std    contrast, and the better indicator of the two. A working IR camera under the dot
           projector shows std ~35; a lens behind tape or film shows ~13 no matter how bright
           the room is, because a diffuser destroys structure while still passing light.
    depth  fraction of depth pixels with a value. This is the payoff -- it should climb from
           under 1 % to tens of percent the moment both IR cameras see the scene.

READ std, NOT mean. A translucent obstruction passes plenty of light, so mean stays plausible
while std collapses. That is precisely why this failure hides: every brightness-based check says
the camera is fine.
"""
import argparse
import sys
import time

import numpy as np


def bar(value, lo, hi, width=28):
    """Simple text bar so a change is obvious at a glance while your hands are on the hardware."""
    f = max(0.0, min(1.0, (value - lo) / float(hi - lo)))
    n = int(round(f * width))
    return '[' + '#' * n + '.' * (width - n) + ']'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--seconds', type=float, default=90.0)
    ap.add_argument('--hz', type=float, default=2.0)
    args = ap.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node('ir_window_check')
    S = {}

    def mk(key):
        def cb(msg):
            S[key] = msg
        return cb

    for key, topic in (('L', '/camera/left_ir/image_raw'),
                       ('R', '/camera/right_ir/image_raw'),
                       ('D', '/camera/depth/image_raw')):
        node.create_subscription(Image, topic, mk(key), qos_profile_sensor_data)

    print('Waiting for IR streams (needs enable_left_ir:=true enable_right_ir:=true) ...')
    t0 = time.time()
    while time.time() - t0 < 15 and len(S) < 3:
        rclpy.spin_once(node, timeout_sec=0.2)
    if 'L' not in S or 'R' not in S:
        print('ERROR: IR streams not publishing. Relaunch the camera with both IR streams on.',
              file=sys.stderr)
        return 1

    # Establish what each stream looks like RIGHT NOW, so any change has something to be a
    # change FROM. Without a baseline printed on screen you are comparing against memory.
    base = {}
    for key in ('L', 'R'):
        g = np.frombuffer(S[key].data, np.uint8)
        base[key] = (g.mean(), g.std())
    print()
    print('  BASELINE   L mean %5.1f std %5.1f     R mean %5.1f std %5.1f'
          % (base['L'][0], base['L'][1], base['R'][0], base['R'][1]))
    blocked = [k for k in ('L', 'R') if base[k][1] < 20]
    if blocked:
        print('  -> %s looks OBSTRUCTED already (std < 20 = light passes, no image forms).'
              % ', '.join(blocked))
        print()
        print('  USE A TORCH, NOT A FINGER. Covering a window that is already covered barely')
        print('  changes anything, so the finger test cannot find it. Shine a phone torch into')
        print('  each opening in turn instead: the one that makes %s mean JUMP is that camera.'
              % '/'.join(blocked))
    else:
        print('  -> both IR cameras look clear. Pass a finger over each opening; the stream')
        print('     whose std collapses is that window.')
    print()
    print('  Watch std, not mean: a translucent film passes light (mean stays plausible) while')
    print('  destroying structure (std collapses). That is why this fault hides.')
    print()
    period = 1.0 / args.hz
    end = time.time() + args.seconds
    nxt = 0.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() < nxt:
            continue
        nxt = time.time() + period
        out = []
        for key in ('L', 'R'):
            m = S.get(key)
            if m is None:
                out.append('%s --' % key)
                continue
            g = np.frombuffer(m.data, np.uint8)
            out.append('%s mean %5.1f  std %5.1f %s' % (key, g.mean(), g.std(),
                                                        bar(g.std(), 0, 45)))
        d = S.get('D')
        if d is not None:
            a = np.frombuffer(d.data, np.uint16)
            valid = 100.0 * ((a > 0) & (a < 65535)).mean()
            out.append('depth %5.2f%% %s' % (valid, bar(valid, 0, 40)))
        sys.stdout.write('  ' + '   '.join(out) + '        \r')
        sys.stdout.flush()
    print()

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
