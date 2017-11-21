#include <ros/ros.h>
#include <ras_group8_brain/Brain.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_brain");
  ros::NodeHandle node_handle("~");

  ras_group8_brain::Brain main_object(node_handle);

  ros::Time start_time;
  ros::Time end_time;
  ros::Duration sleep_time(1);
  ros::Duration diff_time;

  while (ros::ok())
  {
      start_time=ros::Time::now();

      ros::spinOnce();
      main_object.stateMachine();


      end_time=ros::Time::now();
      diff_time=sleep_time-(end_time-start_time);
      ROS_INFO("time %f",diff_time.toSec());

      if(diff_time.toSec()>0)
      {
          diff_time.sleep();
      }
  }



  return 0;
}
