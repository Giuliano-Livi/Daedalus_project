# Height of the drone above the floor, measured on one depth image by daedalus_slam/manhattan_yaw (lowest
# horizontal surface with enough points). Used by vision_to_px4 as an absolute height reference.
std_msgs/Header header   # stamp = depth image, frame_id = base_link
float64 height           # m, base_link above the floor (vertical, gravity-aligned)
uint32 num_points        # floor points used
float64 spread           # m, standard deviation of their heights
