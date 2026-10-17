#!/usr/bin/env python3
"""verify_dtp_viz -- prove the DTP markers actually render, with a person in frame.

*** WHY THIS EXISTS ***

On 2026-09-11 dtp_viz was verified live and reported healthy. What it actually exercised was the
`status` block and a stale `standoff_goal`: the sensors were down, so NOT ONE of the namespaces
that matter with a human present -- people, labels, veto -- had ever been published. The node was
declared working on the strength of the one marker that renders even when nothing works.

So this is the gate that was missing. It subscribes to /dtp/viz exactly as RViz does and refuses to
pass until it has SEEN each expected namespace, CONTINUOUSLY. It renders nothing: if RViz on the
laptop shows an empty scene while this prints PASS, the fault is in RViz's config, its Fixed Frame
or the DDS path to the laptop -- not in the stack. Splitting those two is the whole point.

*** WHY CONTINUITY AND NOT EXISTENCE ***  An adversarial review on 2026-09-12 found that the first
version passed on a SINGLE marker anywhere in the window, and that dtp_viz never expires its target
list (it sets a 0.6 s marker lifetime for RViz but keeps `self._targets` forever). So one frame of
perception at the start, followed by a dead detector for the rest of the window, passed -- with the
status block reading `perception_ok FALSE` right there in the output, captured and never consulted.
Now a namespace must appear in at least half the messages, the publisher itself must be producing
at a plausible rate, and a FALSE health flag is a hard failure.

    # empty room -- what the stack looks like with nobody in it
    python3 scripts/verify_dtp_viz.py --seconds 10

    # the real check: stand 3-4 m in front of the camera, then
    python3 scripts/verify_dtp_viz.py --seconds 20 --expect-person

    # the LOCK path, which needs more than standing there -- see --expect-lock below
    ros2 param set /target_manager follow_enabled true
    python3 scripts/verify_dtp_viz.py --seconds 25 --expect-lock    # and WALK

Exit 0 = every expected namespace rendered continuously and the stack reported healthy.
Exit 1 = something was missing or unhealthy, and the report says which, with the cause.
"""
import argparse
import collections
import math
import re
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray

# Must match dtp_perception/dtp_viz.py. Deliberately duplicated rather than imported: this script
# runs from the repo on a laptop that may not have dtp_perception installed (it is NOT installed on
# dtp-lab-ws), and a verifier that cannot start is worth less than one carrying two constants.
NS_PERSON, NS_LABEL, NS_VETO = 'people', 'labels', 'veto'
NS_PATH, NS_GOAL, NS_STATUS = 'forecast', 'standoff_goal', 'status'

# dtp_viz.py:131 -- create_timer(0.2, self._publish). Used only to sanity-check that the publisher
# is actually running at roughly its design rate, not to time anything.
VIZ_RATE_HZ = 5.0

# A namespace must appear in at least this fraction of received messages. Not 1.0: the status block
# is published every tick but people/veto legitimately blink out for a frame when a detection is
# dropped, and demanding perfection would fail a healthy stack.
CONTINUITY = 0.5

# The six real states in dtp_perception/target_manager.py:196-203. An earlier version of this file
# named a seventh, "NO_TARGET", which does not exist anywhere in the
# stack -- so the hint it printed
# sent the operator looking for a state the code can never report.
LOCK_ENGAGED = ('LOCKED', 'COASTING')
LOCK_NOT_ENGAGED = {
    'IDLE': 'FOLLOW is not engaged at all. Run:'
            ' ros2 param set /target_manager follow_enabled true',
    'ACQUIRING': 'engaged, but no candidate has qualified yet. The subject must be a confirmed '
                 'track in the standoff band, clear of the ignore radius around the launch point, '
                 'and MOVING -- acquisition requires real displacement, so walk, not stand.',
    'ABSTAIN': 'two or more candidates in the re-acquisition band; it is refusing to guess. Have '
               'everyone but the subject step well out of frame.',
    'LOST': 'the coast/ambiguity timeout expired. Re-designate by toggling follow_enabled false '
            'then true.',
}

WHY_MISSING = {
    NS_PERSON: 'perception published no targets. Check /dtp/targets is non-empty '
               '(ros2 topic echo --once /dtp/targets) and that the detector sees you at all '
               '(scripts/live_overlay.py). In an empty room this SHOULD be absent.',
    NS_LABEL: 'labels are drawn per person, so this is absent for the same reason people is.',
    NS_VETO: 'the keep-out is drawn around EVERY person, so this too follows people. If people '
             'appeared and this did not, dtp_viz is at fault rather than perception.',
    NS_PATH: 'the CTRV forecast is drawn ONLY for a LOCKED subject, which needs more than being '
             'visible -- see the lock line below and --expect-lock in this file\'s header.',
    NS_GOAL: 'no goal is being computed. In observe-only the goal goes to the SHADOW topic, so '
             'check dtp_viz was launched with the topic it is actually publishing on '
             '(viz_goal_topic:=/dtp/shadow/follow_goal, which is the default).',
    NS_STATUS: 'dtp_viz is not publishing AT ALL -- this block renders even when every sensor is '
               'down. Check the node is alive (ros2 node list | grep dtp_viz) and that you are on '
               'the same ROS_DOMAIN_ID.',
}


class Verifier(Node):
    def __init__(self):
        super().__init__('verify_dtp_viz')
        # Reliable, matching the RViz display config in dtp_perception/rviz/dtp_follow.rviz. A
        # mismatched QoS here would show zero markers against a perfectly healthy publisher, which
        # is precisely the false negative this script must not produce.
        self.msgs = 0
        self.ns_msgs = collections.Counter()   # messages in which a namespace appeared
        self.ns_markers = collections.Counter()
        self.status_text = None
        self.frames = set()
        self.create_subscription(
            MarkerArray, '/dtp/viz', self._on_markers,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def _on_markers(self, msg):
        self.msgs += 1
        seen = set()
        for m in msg.markers:
            # DELETE/DELETEALL markers carry a namespace but draw nothing. Counting them would let
            # a publisher that only ever clears markers pass as if it drew them.
            if m.action != 0:
                continue
            seen.add(m.ns)
            self.ns_markers[m.ns] += 1
            if m.header.frame_id:
                self.frames.add(m.header.frame_id)
            if m.ns == NS_STATUS and m.text:
                self.status_text = m.text
        for ns in seen:
            self.ns_msgs[ns] += 1

    def health(self):
        """(sensors_ok, perception_ok, lock_state) as strings, from the status block text.

        Parsed rather than subscribed because the block is what an operator actually reads in RViz;
        if this script and RViz disagree about the health, the block itself is the bug.
        """
        if not self.status_text:
            return None, None, None
        flags = re.search(r'sensors_ok\s+(\S+)\s+perception_ok\s+(\S+)', self.status_text)
        lock = re.search(r'lock\s*:\s*([A-Z_]+)', self.status_text)
        return (flags.group(1) if flags else None,
                flags.group(2) if flags else None,
                lock.group(1) if lock else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=15.0, help='how long to watch')
    ap.add_argument('--expect-person', action='store_true',
                    help='require the namespaces that appear whenever a human is VISIBLE: '
                         'people, labels, veto. Does NOT require forecast -- see --expect-lock.')
    ap.add_argument('--expect-lock', action='store_true',
                    help='additionally require forecast, which needs an actual LOCK: '
                         'follow_enabled true, a qualifying candidate, and a MOVING subject. '
                         'Implies --expect-person.')
    a = ap.parse_args()

    rclpy.init()
    node = Verifier()
    print('watching /dtp/viz for %.0f s ...' % a.seconds)
    if a.expect_person or a.expect_lock:
        print('STAND 3-4 m IN FRONT OF THE CAMERA and stay in view for the whole window.')
    if a.expect_lock:
        print('AND KEEP MOVING -- take a step or two every few seconds. Acquisition requires')
        print('real displacement, so standing still is what prevents a lock.')
    deadline = time.time() + a.seconds
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    expected = [NS_STATUS]
    if a.expect_person or a.expect_lock:
        expected += [NS_PERSON, NS_LABEL, NS_VETO]
    if a.expect_lock:
        expected.append(NS_PATH)

    need_msgs = max(1, int(CONTINUITY * a.seconds * VIZ_RATE_HZ))
    need_each = max(1, int(math.ceil(CONTINUITY * node.msgs)))
    sensors_ok, perception_ok, lock_state = node.health()

    print('')
    print('MarkerArray messages : %d   (expected >= %d at %.0f Hz)'
          % (node.msgs, need_msgs, VIZ_RATE_HZ))
    print('frame_id(s) seen     : %s' % (', '.join(sorted(node.frames)) or '(none)'))
    print('')
    print('%-16s %9s %9s  %s' % ('namespace', 'markers', 'in msgs', 'verdict'))
    for ns in (NS_STATUS, NS_PERSON, NS_LABEL, NS_VETO, NS_PATH, NS_GOAL):
        appeared, marks = node.ns_msgs[ns], node.ns_markers[ns]
        if ns not in expected:
            verdict = 'seen' if appeared else 'absent (not required here)'
        elif not appeared:
            verdict = 'MISSING -- required'
        elif appeared < need_each:
            verdict = 'INTERMITTENT -- required in >= %d msgs' % need_each
        else:
            verdict = 'continuous'
        print('%-16s %9d %9d  %s' % (ns, marks, appeared, verdict))

    if node.status_text:
        print('')
        print('status block as RViz would draw it:')
        for line in node.status_text.split(chr(10)):
            print('   ' + line)

    problems = []
    if node.msgs < need_msgs:
        problems.append(
            'dtp_viz produced %d messages in %.0f s; at its %.0f Hz timer that should be >= %d. '
            'The node is stalled or starved, not merely quiet.'
            % (node.msgs, a.seconds, VIZ_RATE_HZ, need_msgs))
    for ns in expected:
        if not node.ns_msgs[ns]:
            problems.append('%s never rendered: %s' % (ns, WHY_MISSING[ns]))
        elif node.ns_msgs[ns] < need_each:
            problems.append(
                '%s rendered in only %d of %d messages (need >= %d). Something is dropping in and '
                'out -- a marker that appears once is not a working display. %s'
                % (ns, node.ns_msgs[ns], node.msgs, need_each, WHY_MISSING[ns]))

    # The health flags were being captured and printed and never consulted, which is how a pass
    # could coexist with perception_ok FALSE in the same output.
    if sensors_ok == 'FALSE':
        problems.append('the stack reports sensors_ok FALSE. Markers may be drawn from stale '
                        'state; fix the sensor before trusting anything above.')
    if perception_ok == 'FALSE':
        problems.append('the stack reports perception_ok FALSE. dtp_viz does not expire its '
                        'target list, so people markers can persist after the detector has '
                        'stopped -- treat any people/veto above as unproven.')
    if a.expect_lock and lock_state in LOCK_NOT_ENGAGED:
        problems.append('lock state is %s, so no forecast can exist: %s'
                        % (lock_state, LOCK_NOT_ENGAGED[lock_state]))

    print('')
    if not problems:
        print('PASS -- every required namespace rendered continuously.')
        if not (a.expect_person or a.expect_lock):
            print('NOTE: run again with --expect-person to exercise people/labels/veto. Those are')
            print('      the ones a status-only check silently skips.')
        elif not a.expect_lock:
            print('NOTE: this did NOT exercise the lock path. forecast markers need'
                  ' --expect-lock,')
            print('      follow_enabled true, and a walking subject.')
        # Say what this did NOT establish. A pass that implies more than it tested is how the
        # 2026-09-11 "verified" result happened in the first place.
        print('')
        print('NOT established by this script: that RViz on the laptop draws them'
              ' (different')
        print('host, different DDS path, its own Fixed Frame), or that the positions are CORRECT')
        print('-- only that markers exist continuously. For position, use a tape measure.')
        rclpy.shutdown()
        return 0

    print('FAIL -- %d problem(s):' % len(problems))
    for p in problems:
        print('')
        print('  * %s' % p)
    if node.msgs == 0:
        print('')
        print('  NOTHING arrived on /dtp/viz at all. Before anything else: is dtp_viz running')
        print('  (ros2 node list | grep dtp_viz), and is ROS_DOMAIN_ID the same here as')
        print('  on the NX?')
    rclpy.shutdown()
    return 1


if __name__ == '__main__':
    sys.exit(main())
