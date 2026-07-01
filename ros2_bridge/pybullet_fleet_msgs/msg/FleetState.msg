# Aggregated state of the whole fleet, published on /fleet/states (Pattern 2).
# One O(1) topic for the entire fleet instead of O(N) per-robot /odom topics.
std_msgs/Header header
RobotState[] robots
