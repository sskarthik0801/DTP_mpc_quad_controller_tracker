// Entry point only -- the actual class lives in include/follow_planner/follow_planner_node.hpp,
// matching mission_planner/Apps/ros2_waypoint_mission.cpp's split (plan/dtp-imp/07 M2.1).
//
// Deliberately NOT pulling in utils/backward.hpp's crash-stack-trace handler (which
// mission_planner/super_planner do) -- it adds a system lib dependency (-ldw) and a copied
// header. The scaffold skipped it because there was no logic to crash; now that there is, it is
// still deferred, but for a different reason: this package has never been compiled anywhere (the
// workspace only builds on the drone NX), and adding an unproven link dependency in the same
// change as the first real build turns one failure mode into two. Add it in the change that
// first builds green on the NX, not before.

#include "rclcpp/rclcpp.hpp"
#include "follow_planner/follow_planner_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("follow_planner_node");

    // ValidateParameters() throws on a config that cannot produce a safe goal, and a throw out of
    // main() would abort with no context beyond a terminate message. Print what happened and exit
    // non-zero so a launch file reports the node as failed rather than as mysteriously absent --
    // a follow_planner that never started is otherwise indistinguishable, from the outside, from
    // one that is running and holding, since both produce silence on /dtp/follow_goal.
    try {
        follow_planner::FollowPlanner fp(node);

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        // SIGTERM is the normal stop path for every node in this stack (ros2 launch sends it on
        // shutdown), and rclcpp has no ExternalShutdownException to catch the way rclpy does: the
        // signal handler invalidates the context and spin() simply returns. What DOES need
        // catching is a callback throwing during that teardown window -- a subscription firing
        // against a half-destroyed context -- which would otherwise turn an ordinary Ctrl-C or
        // launch shutdown into a stack trace and a non-zero exit that looks like a crash in the logs.
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_FATAL(node->get_logger(), "follow_planner_node exiting: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
