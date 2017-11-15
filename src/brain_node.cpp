#include <ros/ros.h>
#include <ras_group8_brain/Brain.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_brain");
  ros::NodeHandle node_handle("~");

  ras_group8_brain::Brain main_object(node_handle);


  ros::spin();
  return 0;
}
