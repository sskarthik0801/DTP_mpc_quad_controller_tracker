# Every person detected this frame, world (odom) frame. Published on /dtp/targets by
# perception_node. This is the array command_mux's independent human-proximity veto reads
# directly (plan/dtp-imp/01 SS2, node/topic contract) -- it must see ALL people, not just
# the locked target, so a bug in target_manager's follow logic cannot defeat the veto.

std_msgs/Header header
dtp_msgs/Target[] targets
