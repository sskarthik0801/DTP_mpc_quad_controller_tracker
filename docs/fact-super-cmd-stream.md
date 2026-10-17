# Durable fact — the `/planning/pos_cmd` contract (SUPER → anything downstream)

**Established 2026-09-02 while building `px4_offboard_bridge` (M3).** Two properties of SUPER's
command topic are not documented anywhere upstream, are invisible in a topic listing, and each one
breaks a naively-written consumer. Both are load-bearing for M3.

Source: `super_planner/include/ros_interface/ros2/fsm_ros2.hpp` (`FsmRos2::init`,
`FsmRos2::pubCmdTimerCallback`, `FsmRos2::getOnePositionCommand`).

---

## 1. The topic is BEST_EFFORT / VOLATILE — a default subscription never connects

```cpp
// FsmRos2::init
const rclcpp::QoS qos(rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile());
cmd_pub_ = nh_->create_publisher<mars_quadrotor_msgs::msg::PositionCommand>(cfg_.cmd_topic, qos);
```

`rclpy`'s and `rclcpp`'s default subscription QoS is **RELIABLE**. A reliable subscriber and a
best-effort publisher are an *incompatible* pair: ROS 2 makes no match, no data flows, and
**nothing errors**. `ros2 topic list` still shows the topic; `ros2 topic info` shows the publisher.
The symptom is a consumer that simply never receives anything.

Any subscriber to `/planning/pos_cmd` must declare:

```python
QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
           durability=DurabilityPolicy.VOLATILE,
           history=HistoryPolicy.KEEP_LAST, depth=1)
```

(`ros2 topic echo` guesses compatible QoS automatically, which is why the topic looks fine from the
command line while your node sees nothing. Don't take `echo` working as proof your node will.)

## 2. The topic goes **completely silent** whenever SUPER is not following a trajectory

```cpp
void pubCmdTimerCallback() {
    if (stop) { return; }
    if (machine_state_ != FOLLOW_TRAJ && machine_state_ != EMER_STOP) { return; }
    ...
    cmd_pub_->publish(pid_cmd_);
}
```

The FSM's states are `INIT, WAIT_GOAL, GENERATE_TRAJ, FOLLOW_TRAJ, EMER_STOP` (`fsm/fsm.h:76`).
**In three of the five, nothing is published at all** — not a zero command, not a hold command,
nothing.

This is not an edge case. It is the *normal* idle state:

- Startup sits in `INIT` until the first `/goal_pose`. That can be minutes.
- Every time SUPER reaches its goal it drops to `WAIT_GOAL` and the stream stops.
- Every replan passes through `GENERATE_TRAJ`.

So **silence on `/planning/pos_cmd` is not a fault and must never be treated as one.** For M3 this
is the single most important property, because PX4 drops Offboard if the setpoint/heartbeat stream
lapses for `COM_OF_LOSS_T` (1.0 s here). A bridge that forwarded commands 1:1 would lose Offboard
the first time the drone reached a goal — in flight.

`px4_offboard_bridge` therefore drives its own 50 Hz output timer off the *last received* command
rather than forwarding, and holds the last commanded position when the input goes quiet. See its
docstring for the IDLE/TRACK/HOLD states.

## 3. Rate and content, as measured

- **~92–99 Hz** measured on hardware (a 10 ms wall timer, so 100 Hz nominal).
- `trajectory_flag` is **1** (`TRAJECTORY_STATUS_READY`) or **2** (`TRAJECTORY_STATUS_EMER`,
  meaning SUPER is on its backup/emergency trajectory). `getOnePositionCommand` only ever writes
  those two; the other constants in the `.msg` are unused by this planner.
  On a **stationary bench** with a goal set, flag flaps 1↔2 several times a second — expected: the
  vehicle isn't following the trajectory, so SUPER keeps falling back and replanning. It is not a
  bridge fault and not a sign of a bad plan.
- `position`, `velocity`, `acceleration`, `jerk`, `yaw`, `yaw_dot` are all populated from the
  differential-flatness output, so a bridge does **not** need to reimplement the flatness math.

## 4. `header.stamp` is unreliable — use arrival time

`Ros2Interface::getSimTime(int32_t &sec, uint32_t &nsec)` (`ros2_interface.hpp:98`) computes

```cpp
sec  = static_cast<int32_t>(now.seconds());
nsec = now.nanoseconds() - static_cast<uint32_t>(sec) * 1e9;
```

The subtraction happens in `double`. At ~1.79e18 ns the spacing between representable doubles is
**256 ns**, so the result is quantised, and when the true nanosecond remainder is near zero it can
go slightly **negative** — which then wraps to a huge value in the `uint32_t`. Timeout and
freshness logic downstream must use **message arrival time**, not this stamp.

(Note this is the same *class* of bug as, but unrelated to, the FAST-LIO odometry-latency defect in
[`fact-fastlio-latency.md`](fact-fastlio-latency.md).)

---

## Related

- [`ImplementationDoc.md`](ImplementationDoc.md) §M3 — the frame conversion and the two traps.
- [`fact-offboard-rc-arming.md`](fact-offboard-rc-arming.md) — what PX4 requires before it will
  accept Offboard, and what still blocks arming.
