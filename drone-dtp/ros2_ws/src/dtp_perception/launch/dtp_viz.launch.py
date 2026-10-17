"""dtp_viz.launch.py -- the DTP visualisation, deliberately SEPARATE from the follow stack.

WHY IT IS ITS OWN LAUNCH AND NOT PART OF follower.launch.py
-----------------------------------------------------------
Two reasons, both operational.

**It must be startable and killable independently of what it watches.** During a field test
view is the part you fiddle with -- restart RViz, change a topic, move the camera
view -- and none of
that may risk the stack that is flying. Folding it into `follower.launch.py` would mean a crashed
RViz takes down perception with it.

**RViz belongs on the operator's machine, not on the aircraft.** The Jetson is the flight computer;
it has no display, and rendering a point cloud on it competes for the GPU with the detector. The
measured consequence of loading that GPU is in plan/dtp-imp/21: perception at 15 Hz already
saturates its executor and starves its own transform callbacks. So `rviz:=true` is for a laptop;
on the aircraft run `rviz:=false` and let the operator's RViz subscribe over the network.

WHAT IT SHOWS, AND WHY THAT SET
-------------------------------
`dtp_viz` exists because RViz cannot render `dtp_msgs/TargetArray`, the `std_msgs/String` state
topics, or the proximity keep-out (a rule inside command_mux, not a published shape). Without it
there is no way to LOOK at the follower -- only to read a terminal -- which during a test is the
difference between seeing the veto fire and not.

`dtp_image_overlay` (added 2026-09-13) is the other half: the camera picture with perception's
person boxes drawn on it, as /dtp/image_annotated/compressed. It shows what the markers cannot --
whether the box is actually on the person -- and it is idle (no subscriptions at all) until a
viewer subscribes. On the laptop, `scripts/laptop_dtp_viewer.sh` (repo root) decodes it and opens
RViz with both; this file's own `rviz:=true` config does not show the picture.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('dtp_perception'), 'rviz', 'dtp_follow.rviz'])

    return LaunchDescription([
        # Default is the SHADOW topic, because the case where a picture matters most is Stage A
        # (observe-only flight): the goals are computed and command nothing, so the terminal shows
        # almost nothing and the only way to judge them is to see them. For a live follow, pass
        # goal_topic:=/planning/follow_goal.
        DeclareLaunchArgument(
            'goal_topic', default_value='/dtp/shadow/follow_goal',
            description='which goal stream to draw. Default is the Stage A shadow topic; use '
                        '/planning/follow_goal for a live follow.'),
        # FALSE on the aircraft. See the module docstring: the Jetson has no display, and rendering
        # competes for the GPU with the detector, which already saturates its executor at 15 Hz.
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='start RViz here. Leave false on the aircraft and run RViz on the '
                        'operator laptop against the same DDS domain.'),
        DeclareLaunchArgument(
            'rviz_config', default_value=rviz_config,
            description='RViz config; defaults to the packaged dtp_follow.rviz'),

        Node(
            package='dtp_perception', executable='dtp_viz', name='dtp_viz',
            output='screen',
            parameters=[{'goal_topic': LaunchConfiguration('goal_topic')}],
        ),

        # The boxes on the camera picture (/dtp/image_annotated/compressed). Started alongside
        # dtp_viz because it is just as passive, and it subscribes to nothing -- not even the
        # camera -- until a viewer subscribes to its output. See dtp_image_overlay's docstring.
        Node(
            package='dtp_perception', executable='dtp_image_overlay', name='dtp_image_overlay',
            output='screen',
        ),

        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
