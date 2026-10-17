# ROG-Map: effective height bounds ≠ configured, and where fsm_node's 2 GB goes

Two things measured on the Orin NX (2026-09-01) that the M1 notes never recorded, both
stationary full-stack. Neither is a bug; both change what you should expect.

## 1. `virtual_ground_height: -1.0` does NOT take effect as -1.0

ROG-Map **quantises and insets** the height bounds at init. The parameter loads correctly
(`Load param rog_map/virtual_ground_height success: -1`) and is then rewritten:

```
config value            effective value
virtual_ground_height   -1.0     ->   -0.5
virtual_ceil_height      3.5     ->    2.9
```

`rog_map_core/config.hpp` (ORIGIN_AT_CORNER build, `inflation_resolution: 0.2`,
`inflation_step: 2`):

```
ground: floor(-1.0/0.2) = -5,  -5 + 2 = -3  ->  -3(0.2) + 0.1 = -0.5
ceil:   floor( 3.5/0.2) = 17,  17 - 2 = 15  ->  15(0.2) - 0.1 =  2.9
```

So each bound is inset by `inflation_step` (2 voxels = 0.4 m) plus half-voxel centring —
the inflation margin, so inflated obstacles cannot spill outside the map. Confirmed in the
startup log: `resetMapSize: virtual_ceil_height: 2.9, virtual_ground_height: -0.5`.

**This validates the CLAUDE.md rule rather than contradicting it.** Upstream's `-0.1` gives
`floor(-0.1/0.2) = -1, -1+2 = 1 -> 1(0.2)+0.1 = +0.3` — an effective floor of **+0.3 m**,
i.e. everything at ground level rejected as "Odom below virtual ground". That is exactly the
reported symptom. `-1.0` is still the right setting.

**What to actually expect:** the usable Z band is **[-0.5, 2.9] m**, not [-1.0, 3.5]. Headroom
below the start point is **0.45 m**, not 1.0 m. Relevant when tuning altitude limits in M4/M5.
Measured odom on a stationary bench rig sits at z ≈ -0.05 m, so the margin is real but modest.

## 2. `fsm_node`'s ~2.03 GB is a startup preallocation, not a leak

`fsm_node` RSS sits at **2033 MB** (≈13 % of the NX's 16 GB) from the moment it starts, and is
**flat** — 2033.2 -> 2033.6 MB over 9.5 min. Prior M1 resource notes only ever tracked
FAST-LIO's RSS, so this number was never written down.

It is the A* search grid, allocated once in `super_planner/src/super_core/astar.cpp:38-41`:

```cpp
int map_buffer_size = cfg_.map_voxel_num(0) * cfg_.map_voxel_num(1) * cfg_.map_voxel_num(2);
...
i = new GridNode;          // one heap allocation PER VOXEL
```

With `astar/map_voxel_num: [500, 500, 100]` in `click_smooth_ros2.yaml`, config.hpp rounds each
axis to `2*(n/2)+1`, so the startup log prints `map vox_num: 501 501 101` = **25.34 M nodes**.
`GridNode` is 56 B (enum + int + Vec3i + 3 doubles + ptr, padded); with a ~16 B malloc header
that is ~1.83 GB, plus a 25.34 M x 8 B pointer vector (~203 MB) = **~2.03 GB**. Matches the
observed RSS.

Deterministic and bounded — but it is a fixed cost you pay before any planning happens, and it
scales as the **product** of `map_voxel_num`. Note the A* volume (50 x 50 x 10 m at 0.1 m) is
taller than ROG-Map's own `map_size: [50,50,6]`, so the Z term is oversized for this
configuration; halving `map_voxel_num[2]` would return ~1 GB if RAM ever gets tight.

## 3. The recurring "-- [Fsm] No odom." log is cosmetic

It appears intermittently during healthy operation (5 in 263 s, 21-29 in 90 s under load) and
looks like an odometry dropout. It is not.

`super_planner/src/super_core/fsm.cpp:101` (and `:118`) hardcode the staleness threshold at
**0.1 s** — exactly the 10 Hz odom period, so there is zero margin by construction:

```cpp
if ((!robot_state_.rcv || (ros_ptr_->getSimTime() - robot_state_.rcv_time) > 0.1)) {
    cout << YELLOW << " -- [Fsm] No odom." << RESET << endl;
```

Measured odom inter-arrival is 0.074-0.123 s, so whenever the 1 Hz print tick lands in the tail
past 0.1 s it fires. It is **unrelated** to the `rog_map/ros_callback/odom_timeout: 2` parameter,
which guards ROG-Map's *cloud* callback (`rog_map_ros2.hpp:113`, different message).

**It does not gate planning:** `SuperPlanner::PlanFromRest` (`super_planner.cpp:76`) checks only
`robot_state_.rcv == false` — "have we ever received odom" — never the 0.1 s value. The early
`return` at fsm.cpp:103 sits behind the 1 Hz print gate, so it skips ~0.02 % of the 100 Hz FSM
ticks. Expect this line; it is not a fault.

Note `rcv_time` is set from `getSystemWalltimeNow()` (`rog_map/src/rog_map/rog_map.cpp:271`) —
wall-clock **arrival**, not the message stamp. That is why this check stayed silent throughout
the 1.4 s stale-odometry bug in [`fact-fastlio-latency.md`](fact-fastlio-latency.md): SUPER
cannot see stamp staleness at all.
