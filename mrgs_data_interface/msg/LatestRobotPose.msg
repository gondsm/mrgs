# A message that carries a pose along the network
uint8 id                                 # The ID of the robot in question
geometry_msgs/TransformStamped transform # The tf between the robot's map and odom frames
geometry_msgs/Pose pose	                 # The robot's latest known pose