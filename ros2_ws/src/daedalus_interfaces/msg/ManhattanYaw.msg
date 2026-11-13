# Heading of the drone relative to the warehouse's orthogonal structure (Manhattan world), measured on one
# depth image by daedalus_slam/manhattan_yaw. Used by vision_to_px4 as an absolute heading reference.
std_msgs/Header header   # stamp = depth image, frame_id = base_link
float64 yaw              # rad, in [-pi/4, pi/4): angle of a warehouse axis in the levelled base_link frame
                         # (FLU, counter-clockwise from forward); known modulo pi/2
float64 quality          # 0..1, concentration of the horizontal surface normals around the 4 axes
uint32 num_normals       # horizontal surface normals used
