# A message that carries maps and transforms along the network
string mac                                      # The sending robot's MAC address
time grid_stamp                                 # Grid creation time
nav_msgs/MapMetaData info                       # Only read if is_complete = true
uint32 decompressed_length                      # Used to allocate the destination buffer
int8[] compressed_data                          # LZ4 compressed grid
geometry_msgs/TransformStamped map_to_base_link # Current map->base_link transform
