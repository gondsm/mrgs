# A message that carries a transform between the complete_map and a remote robot's map frames
Header header
uint8 id  # The internal ID of the robot in question
geometry_msgs/TransformStamped transform  # The tf between the current robot's complete_map and remote robot's map frames
