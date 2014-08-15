
# gripper opening before moving arm
float32 				premove_gripper_opening

# pose to acquire before moving in to the object
geometry_msgs/Pose   	pose

# pick gripper opening
float32 				postmove_gripper_opening

# pose to acquire after pickup
geometry_msgs/Pose   	postmove_pose

# wait time after movement
duration 				postmove_wait
