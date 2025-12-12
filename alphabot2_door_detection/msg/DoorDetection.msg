std_msgs/Header header
bool detected # Whether the target is found in the middle of the fov
float32 bearing     # how far off the target if from center of field of view, in radians; 0 = ahead; +left, -right
float32 distance    # estimated distance to target, -1.0 if unknown