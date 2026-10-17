cmake_minimum_required(VERSION 3.8)
project(super_bringup)

find_package(ament_cmake REQUIRED)

# Pure integration package: install launch (and config, when added) only.
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# M2/M3 bridge nodes (rclpy; no ament_python build type needed for plain scripts).
install(PROGRAMS
  scripts/px4_odom_bridge.py
  scripts/px4_offboard_bridge.py
  scripts/offboard_viz.py
  # The MPC trajectory tracker: an ALTERNATIVE to px4_offboard_bridge.py, never run
  # beside it (docs/mpc-tracker.md). Not started by anything unless selected.
  scripts/px4_mpc_tracker.py
  DESTINATION lib/${PROJECT_NAME}
)

# The camera yaw servo's pure algorithm, imported by px4_offboard_bridge.py from its own
# directory (docs/plan-camera-yaw-servo.md section 2.1). A module, not a node, so FILES:
# installed beside the bridge, not executable, not listed by `ros2 run`.
# mpc_tracker_core.py is the MPC's maths (numpy only), imported by px4_mpc_tracker.py from
# its own directory the same way, and by scripts/px4/mpc_tracker_sim.py from the repo.
install(FILES
  scripts/yaw_servo.py
  scripts/mpc_tracker_core.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
