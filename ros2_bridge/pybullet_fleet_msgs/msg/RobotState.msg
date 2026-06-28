# One robot's state within the aggregated /fleet/states message (Pattern 2 batch
# API). Carrying N of these in a single FleetState avoids N per-robot /odom topics.
string name
float64 x      # world-frame position (m)
float64 y
float64 yaw    # heading (rad)
float64 vx     # world-frame linear velocity (m/s)
float64 vy
float64 vyaw   # angular velocity (rad/s)
bool moving
