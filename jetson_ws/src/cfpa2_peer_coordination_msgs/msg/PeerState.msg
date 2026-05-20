# Periodic broadcast of this robot's decentralised exploration state.

std_msgs/Header header
string robot_id
geometry_msgs/Pose pose
ClaimedFrontier[] claimed_frontiers
time last_interaction_attempt_stamp
time last_successful_interaction_stamp
uint8 protocol_version