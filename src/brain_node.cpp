#include <ros/ros.h>
#include <ras_group8_brain/Brain.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_brain");
  ros::NodeHandle node_handle("~");

  ras_group8_brain::Brain main_object(node_handle);

  ros::Time start_time_spin;
  ros::Time end_time_spin;
  ros::Duration sleep_time(1);
  ros::Duration diff_time_spin;

  while (ros::ok())
  {
      start_time_spin=ros::Time::now();

      ros::spinOnce();
      main_object.stateMachine();


      end_time_spin=ros::Time::now();
      diff_time_spin=sleep_time-(end_time_spin-start_time_spin);
      //ROS_INFO("time %f",diff_time_spin.toSec());

      if(diff_time_spin.toSec()>0)
      {
          diff_time_spin.sleep();
      }
  }

  return 0;
}
