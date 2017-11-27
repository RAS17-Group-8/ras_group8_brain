#include <ras_group8_brain/Brain.hpp>

// STD
#include <string>

namespace ras_group8_brain {



void Brain::pathExectuionState()
{
   ROS_INFO("Path Execution State");

   if(obstacle_&&!home_)
   {
      state_=4;
      std_msgs::Bool stop;
      stop.data=true;
      path_stop_publisher_.publish(stop);
   }
   else if(home_&&picked_up_element_>=0)
   {
       state_=5;
   }
   else if(path_done_&&planned_element_>=0) //////////////think what happens if the robot is not able to find its goal
   {
       // robot turn a bit move a bit
       ros::spinOnce();
       if(!obstacle_)
       {
           ObstacleList_[planned_element_].try_counter++;
           state_=2;
       }
   }
   else if(round1_&&!go_home_&&(round_time_-(run_time_-ros::Time::now()).toSec())<1)
   {
      go_home_=true;
      ROS_INFO("Time to go home");
      state_=2;
   }
   else if(path_done_&&round1_)
   {
       if (planned_edge_>=0)
       {
            edges_[planned_edge_].explored=true;
            planned_edge_=-1;
       }
       state_=1;
   }
}


void Brain::pathDoneCallback(const std_msgs::Bool &msg)
{
    path_done_=true;
   ////////////////////check home eher in position subscriber
   ///
    ROS_INFO("Path_Message");

}

} /* namespace */
