# Message type to hold a map from another robot
geometry_msgs/TransformStamped map_to_base_link # Current filtered_map->base_link transform
nav_msgs/OccupancyGrid filtered_map             # Filtered map. Should have been cropped at the Map Dam, hence the new
                                                # transform.
