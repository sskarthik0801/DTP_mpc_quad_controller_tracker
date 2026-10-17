import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'dtp_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'calib'), glob('calib/*.yaml')),
        # calib_sensors.launch.py brings the LiDAR up as PointCloud2 (xfer_format 0),
        # which calibration and koide3 need. The flight launch uses CustomMsg for FAST-LIO.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # dtp_viz.launch.py resolves its config through FindPackageShare, so the rviz/ dir
        # MUST be installed or the launch dies with a path that does not exist.
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krishna',
    maintainer_email='srajule@purdue.edu',
    description='dtp-imp human-follow perception nodes: fusion, DTP, target lock, replay.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # <exe name> = <module path>:<function>  -- this line is what `ros2 run dtp_perception
            # fusion_node` actually resolves to a Python callable.
            #
            # Publishes the ONE static transform that makes the TF chain resolve at all. Nothing in
            # this repository published <body> -> camera before it, so
            # lookup_transform('camera_init', 'camera_color_optical_frame') FAILED on the aircraft
            # and perception_node could not place a single detection in the world. It REFUSES TO
            # START without body_frame, which is deliberate -- see its module docstring.
            'static_calib_tf = dtp_perception.static_calib_tf:main',
            # RViz renders a fixed set of message types, and NONE of the things that show
            # whether this stack works are among them: dtp_msgs/TargetArray has no plugin,
            # the lock and arbitration state are std_msgs/String, and the proximity keep-out
            # is a rule inside command_mux rather than a published shape. Without this
            # converter there is no way to LOOK at the follower, only to read a terminal --
            # which during a field test is the difference between seeing a veto fire and
            # not. PASSIVE: publishes /dtp/viz only, and nothing in the flight path
            # consumes it, so it can be started, killed or omitted with no effect.
            'dtp_viz = dtp_perception.dtp_viz:main',
            # The camera picture with perception's person boxes drawn on it, as a JPEG for a
            # laptop RViz (/dtp/image_annotated/compressed). Also passive, and idle -- no
            # subscriptions at all -- until something subscribes to its output.
            'dtp_image_overlay = dtp_perception.dtp_image_overlay:main',
            # OFF BY DEFAULT (enable:=false). A debug view of the organized cloud for RViz and for
            # recording replay bags. 11.06 MB per message is why it is NOT in the perception path:
            # perception_node builds the identical array in-process via
            # dtp_perception.organized_cloud, which both of them share so a bag and the flight path
            # cannot deproject differently. organized_cloud is a library, not a node, so it has no
            # entry point of its own.
            'fusion_node = dtp_perception.fusion_node:main',
            'perception_node = dtp_perception.perception_node:main',
            'target_manager = dtp_perception.target_manager:main',
            'lidar_overlay_node = dtp_perception.lidar_overlay_node:main',
            # Detects a sensor that has gone SILENT while the process still looks healthy --
            # the Orbbec driver stalled that way on 2026-09-07 with nothing logged.
            'sensor_watchdog = dtp_perception.sensor_watchdog:main',
            'playback_node = dtp_perception.playback_node:main',
            # Reusable camera<->LiDAR calibration workflow (mounts change per airframe
            # revision, so this must be cheap to repeat -- see
            # plan/dtp-imp/19-camera-lidar-calibration-runbook.md).
            'capture_calib_data = dtp_perception.calibration.capture_calib_data:main',
            'validate_calib = dtp_perception.calibration.validate_calib:main',
            'import_koide3 = dtp_perception.calibration.import_koide3:main',
            'write_calib_yaml = dtp_perception.calibration.write_calib_yaml:main',
            # Independent accuracy gate: sweeps pixel offsets and requires the edge
            # alignment to PEAK at zero. in-view % and the overlays cannot catch a
            # few-pixel bias; solver residuals only re-measure what the solver fit.
            'score_alignment = dtp_perception.calibration.score_alignment:main',
            # Decides the motor-on gate: reduces two extrinsics to ONE number -- how far,
            # in pixels, the same LiDAR point lands differently. Raw mm and degrees are not
            # comparable to each other; pixels are what the perception stack consumes.
            'compare_calib = dtp_perception.calibration.compare_calib:main',
        ],
    },
)
