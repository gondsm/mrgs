// Node created to test the performance of compression libraries, and to help determine which efficient communication
// strategy should be employed.
// ROS includes
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

// LZ4 include:
#include "lz4/lz4.h"

void compress(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  // Indicate we received a grid
  ROS_INFO("Received a grid, %dx%d, totalling %d bytes in data.", map->info.height, map->info.width, map->data.size());
  
  // Allocate entry and exit buffers, copy data to entry buffer
  int map_length = map->data.size();
  char* decompressed = new char [map_length];
  char* compressed = new char [LZ4_compressBound(map_length)]; // LZ4_compressBound returns the worst-case length
  for(int i = 0; i < map_length; i++)
    decompressed[i] = map->data.at(i);

  // Compress data
  ros::Time compression_init = ros::Time::now();
  int compressed_bytes = LZ4_compress(decompressed, compressed, map_length);
  ros::Duration compression_duration = ros::Time::now() - compression_init;
  
  // Inform
  ROS_INFO("%d bytes turned into %d bytes in %f seconds. Compression ratio: %f.", map_length, compressed_bytes, compression_duration.toSec(), (float)map_length/(float)compressed_bytes);
  
  // Decompress data
  ros::Time decompression_init = ros::Time::now();
  int decompressed_bytes = LZ4_decompress_safe(compressed, decompressed, compressed_bytes, map_length);
  ros::Duration decompression_duration = ros::Time::now() - decompression_init;
  
  // Inform
  ROS_INFO("%d bytes turned back into %d bytes in %f seconds.", compressed_bytes, decompressed_bytes, decompression_duration.toSec());
  
  // Free memory
  delete compressed, decompressed;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "compression_test_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("map", 1000, compress);
  ros::spin();
  return 0;
}
