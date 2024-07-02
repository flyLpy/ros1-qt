#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hello_world_node");
  ros::NodeHandle nh;
  ROS_INFO("Hello, world!");
  ros::spin();
  return 0;
}

