# A message that carries maps along the network
string mac                 # The sending robot's MAC address
bool is_complete           # Indicates whether this is a complete or partial map
time grid_stamp            # Grid creation time
nav_msgs/MapMetaData info  # Only read if is_complete = true
uint32 decompressed_length # Used to allocate the destination buffer
int8[] compressed_data     # LZ4 compressed grid
