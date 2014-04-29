# A message that carries a pose along the network
Header header
string mac                 # The sending robot's MAC address
geometry_msgs/TransformStamped transform	# The tf between the sending robot's map and odom frames
geometry_msgs/Pose pose	   # The sending robot's latest pose