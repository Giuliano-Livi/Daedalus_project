# Reference for the offboard controller, published by the trajectory generator at 20–50 Hz.
# Frame: ROS standard ENU (x East, y North, z Up), frame_id e.g. "map".
# Set a field to NaN if it is not commanded.
std_msgs/Header header          # stamp = time this setpoint was generated
geometry_msgs/Point   position  # m
geometry_msgs/Vector3 velocity  # m/s, feed-forward
geometry_msgs/Vector3 acceleration  # m/s^2, feed-forward (NaN if unused)
float64 yaw                     # rad, ENU (0 = East, counter-clockwise positive)
float64 yaw_rate                # rad/s
