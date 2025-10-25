#include <ros/ros.h>

#include <grid_maze_map/layer.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_mapping");

  MapLayer ml;
  ros::spin();

  return(0);
}