#!/usr/bin/env python3
"""
Patch FAST_LIO_ROS2 laserMapping.cpp: fix ~1.35 s odometry latency.

The upstream ROS2 port runs rclcpp::spin() -- a SINGLE-threaded executor with no
callback groups. One thread serialises the 200 Hz IMU callback, the 10 Hz cloud
callback (which preprocesses ~20k points inline) and a 100 Hz timer whose callback
is slower than its 10 ms period. The timer is therefore permanently overdue, the
executor keeps servicing it, and the cloud subscription is starved: clouds sit in
the receive queue ~1.35 s before FAST-LIO ever sees them. Timestamps stay correct,
so the symptom is silent -- odometry is simply 1.4 s stale.

Fix: put the sensor subscriptions and the main timer in separate callback groups
and spin a MultiThreadedExecutor, so a busy timer cannot starve the subscriptions.
sync_packages() touches lidar_buffer/imu_buffer/time_buffer without holding
mtx_buffer -- safe only under a single-threaded executor -- so it must take the
lock once callbacks can run concurrently.
"""
import re, sys

SRC = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/ideas-ad/super_ws/src/FAST_LIO_ROS2/src/laserMapping.cpp"

s = open(SRC).read()
orig = s
applied = []

def sub(old, new, tag, count=1, marker=None):
    global s
    if (marker or new) in s:          # idempotent: already applied
        print("  --    %-28s (already applied)" % tag); return False
    if old not in s:
        print("  SKIP  %-28s (anchor not found)" % tag); return False
    s = s.replace(old, new, count)
    applied.append(tag); print("  ok    %s" % tag); return True

# 1) sync_packages: take mtx_buffer for the whole function (buffer ops only, cheap)
sub("""bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }""",
    """bool sync_packages(MeasureGroup &meas)
{
    // Executor is multi-threaded: the sensor callbacks push into these buffers
    // from another thread, so this whole function must hold mtx_buffer.
    std::lock_guard<std::mutex> sync_lock(mtx_buffer);
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }""", "sync_packages lock")

# 2) subscriptions -> dedicated callback group
sub("""        /*** ROS subscribe initialization ***/
        if (p_pre->lidar_type == AVIA)
        {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk);
        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk);
        }
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk);""",
    """        /*** ROS subscribe initialization ***/
        // Sensor intake and the heavy mapping timer live in separate callback
        // groups so the always-overdue 100 Hz timer cannot starve the cloud sub.
        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_timer_   = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sensor_sub_opt;
        sensor_sub_opt.callback_group = cb_group_sensors_;
        if (p_pre->lidar_type == AVIA)
        {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk, sensor_sub_opt);
        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk, sensor_sub_opt);
        }
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk, sensor_sub_opt);""",
    "subscription cb groups", marker="sensor_sub_opt.callback_group")

# 3) main timer -> its own callback group
sub("""        timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&LaserMappingNode::timer_callback, this));""",
    """        timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&LaserMappingNode::timer_callback, this), cb_group_timer_);""",
    "timer cb group")

# 4) member declarations
sub("""    rclcpp::TimerBase::SharedPtr timer_;""",
    """    rclcpp::CallbackGroup::SharedPtr cb_group_sensors_;
    rclcpp::CallbackGroup::SharedPtr cb_group_timer_;
    rclcpp::TimerBase::SharedPtr timer_;""",
    "member decls")

# 5) MultiThreadedExecutor
sub("""    rclcpp::spin(std::make_shared<LaserMappingNode>());""",
    """    {
        auto lm_node = std::make_shared<LaserMappingNode>();
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
        executor.add_node(lm_node);
        executor.spin();
    }""",
    "multi-threaded executor")

if s != orig:
    open(SRC, "w").write(s)
    print("PATCHED %s  (%d/5 hunks)" % (SRC, len(applied)))
else:
    print("NO CHANGE (already patched?)")

# --- second pass: queue depths -------------------------------------------
# The cloud subscription had depth 20. During the startup stall (ikd-Tree build)
# it fills, and because steady-state drain rate == input rate it NEVER empties:
# a permanent ~13-frame standing queue == ~1.3 s of stale odometry. For odometry
# we always want the freshest scan, so keep the cloud queue shallow and let old
# scans be dropped. The IMU queue must NOT drop -- those samples are integrated --
# so it goes the other way.
s2 = open(SRC).read(); o2 = s2
s2 = s2.replace(
    "sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk, sensor_sub_opt);",
    "sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 2, livox_pcl_cbk, sensor_sub_opt);")
s2 = s2.replace(
    "sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk, sensor_sub_opt);",
    "sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 400, imu_cbk, sensor_sub_opt);")
if s2 != o2:
    open(SRC, "w").write(s2); print("  ok    queue depths (cloud 20->2, imu 10->400)")
else:
    print("  SKIP  queue depths")
