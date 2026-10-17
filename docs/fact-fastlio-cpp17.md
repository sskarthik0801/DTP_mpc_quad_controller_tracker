---
name: fast-lio-ros2-cpp17-build-fix
description: "FAST_LIO ROS2 branch won't build on Humble until its CMake C++ standard is bumped to 17"
metadata: 
  node_type: memory
  type: reference
  originSessionId: 44902c0c-5771-4b30-82cd-58ae6d892916
---

The hku-mars `FAST_LIO` ROS2 branch (`super_ws/src/FAST_LIO_ROS2`) hard-codes `-std=c++14` in `CMakeLists.txt` (lines ~8–18, redundantly 5×). Against the current `ros-humble-rclcpp` (16.0.19, whose headers are C++17-only) this fails with `error: using invalid field 'callback_variant_'` inside `any_subscription_callback.hpp` while instantiating the intra-process subscription for `livox_ros_driver2::msg::CustomMsg`.

**Fix:** change every `c++14`/`c++0x` to `c++17` and set `CMAKE_CXX_STANDARD 17`, then `rm -rf build/fast_lio install/fast_lio` and rebuild. A harmless GCC `note:` about `std::pair` ABI parameter passing remains (not an error). Build needs `~/ws_livox/install` sourced first (for the CustomMsg interface).

This reverts on every fresh clone. Also: a fresh `git clone` of SUPER wipes any in-tree additions, so FAST_LIO_ROS2 and `super_bringup` (the M1 launch package) are kept as **separate** `src/` packages. Related: [[super-human-follow-hardware]].
