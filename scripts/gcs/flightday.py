"""The Flight day card: the standard follower bring-up, in order, for the mode you pick.

Until 2026-09-20 this was docs/flight-day-runbook.md step by step: three sorties, 37 rows, most
of them a pilot's or a terminal's and so without a button.  It is now only what is pressed on
every bring-up -- stack, bridges, gates, follower, recording, preflight, engage, and the stops
after landing -- with ONE choice at the top of the card: Observe only, or Live.  A step with
`modes` is shown in those modes only; one without is shown in both.  The runbook is still the
reference for the sorties; the checks this card dropped (desk, RC switches, the mannequin
preflight) are still chips on the Checks card.

This module holds DATA only.  Every step maps onto an endpoint that already exists and already
carries its own interlock -- a process row's Start (with its config or mode), a check chip, the
recorder, and the DTP row's Engage FOLLOW (the one button that designates a target; behind
Admin control, see dtp.follow_blocked()).  What the card adds is the ORDER and the expected
result next to each button.

kind      what the page does
  check   a button that runs CHECKS[key]; the result pill comes from the last run
  proc    a button that starts (or, action='stop', stops) PROCS[key], with the row's own options
  bag     a button that starts the recorder on a topic preset (bagger.PRESETS), or with
          action='stop' stops it (POST /api/bag/stop, what the Recording card's Stop does)
  follow  a button that engages FOLLOW (POST /api/dtp/follow?set=on); the pill is /dtp/lock_state
"""

MODES = ('observe', 'live')

PHASES = [
    ('bringup', 'Bring-up', 'Aircraft on the ground at the takeoff spot, nose down the corridor: the map and the geofence are anchored where FAST-LIO starts.'),
    ('follower', 'Follower', 'Observe only commands nothing. Live publishes goals to SUPER; nothing flies until the pilot arms and flips channel 11, and channel 11 OFF is the abort.'),
    ('after', 'After landing', ''),
]

STEPS = [
    # ---- bring-up
    dict(id='stack', phase='bringup', kind='proc', key='stack', opts={'config': 'follow'},
         title='Start the stack on the FOLLOW config', detail='Livox + FAST-LIO + fsm_node on follow_test_ros2.yaml. Odom must read >= 9.5 Hz before the follower starts.'),
    dict(id='odom_bridge', phase='bringup', kind='proc', key='odom_bridge',
         title='Start the odometry bridge', detail='FAST-LIO -> PX4 EKF2. The Estimator card should show position aiding within seconds.'),
    dict(id='offboard_bridge', phase='bringup', kind='proc', key='offboard_bridge',
         title='Start the trajectory bridge', detail='Offboard heartbeat at 50 Hz. It sends setpoints only while PX4 is in Offboard (channel 11), so the trajectory_setpoint chip reads silent until then.'),
    dict(id='field_check', phase='bringup', kind='check', key='field_check',
         title='FAST-LIO gates', detail='No foreign subscriber on the cloud, no stamp lag. A laptop RViz on /cloud_registered fails it: close it, wait 20 s, re-run.'),
    # ---- follower
    dict(id='dtp_observe', phase='follower', kind='proc', key='dtp', opts={'mode': 'observe'}, modes=['observe'],
         title='Start the follower OBSERVE-ONLY', detail='Goals and yaw targets go to /dtp/shadow/*; nothing reaches the planner. Wait for "GPU cloud verified" and "extrinsic verified" in its log.'),
    dict(id='dtp_live', phase='follower', kind='proc', key='dtp', opts={'mode': 'live'}, modes=['live'],
         title='Start the follower LIVE', detail='command_mux publishes /planning/follow_goal and holds a NAV_LAND abort armed. Refused unless fsm_node is on the follow config, odom >= 9.5 Hz and the FC says disarmed.'),
    # The recorder goes up BEFORE the preflight, as in deploy_follow.sh, so the preflight sees
    # it instead of warning that it is absent.
    dict(id='bag_observe', phase='follower', kind='bag', preset='stage_a', modes=['observe'],
         title='Start the recording', detail='The Stage A topic list incl. camera frames (needs Admin control). Never open the bag while it records.'),
    dict(id='bag_live', phase='follower', kind='bag', preset='stage_b', modes=['live'],
         title='Start the recording', detail='Setpoints, goal status, mux state, abort, perception, the loss oracle and camera frames (needs Admin control). Never open the bag while it records.'),
    dict(id='preflight_observe', phase='follower', kind='check', key='stage_a_preflight', modes=['observe'],
         title='Preflight (~90 s)', detail='Must say GO; extrinsic_est_en is an expected warning. It carries the 60 s estimator window.'),
    dict(id='preflight_live', phase='follower', kind='check', key='live_preflight', modes=['live'],
         title='Preflight (~90 s)', detail='The flags deploy_follow.sh passes: --max-vel 1.0 and the single-IMU waiver. It FAILs on auto_reacquire_after_lost by design (set true 2026-09-15): read the log and check that is the ONLY fail. Carries the 60 s estimator window.'),
    dict(id='engage', phase='follower', kind='follow',
         title='Engage FOLLOW: walker at 4-6 m and moving, wait for LOCKED', detail='The same button re-engages after a LOST. Anyone within 1.5 m of the aircraft holds the mux in HOLD|PROXIMITY.'),
    dict(id='estimator', phase='follower', kind='check', key='estimator',
         title='Estimator preflight (60 s) -- before every LATER arm', detail='The preflight above covers the first arm. GO or no arm.'),
    # ---- after
    dict(id='bag_stop', phase='after', kind='bag', action='stop',
         title='Disarmed: stop the recording', detail='rosbag2 writes metadata.yaml only on a clean stop.'),
    dict(id='dtp_stop', phase='after', kind='proc', key='dtp', action='stop',
         title='Stop the follower', detail='Also how to change mode: a running follower is never switched, and after a mux ABORT it must be restarted.'),
    dict(id='faults_after', phase='after', kind='check', key='faults',
         title='Scan the fault card', detail='A reset in flight leaves a dump. Archive it into firmware/faults/ before clearing.'),
]


def config():
    """What /api/config carries: phases and steps, verbatim."""
    return {'phases': [{'id': i, 'title': t, 'note': n} for i, t, n in PHASES],
            'steps': STEPS}
