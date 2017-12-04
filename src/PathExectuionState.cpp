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
   }
   else if(home_&&picked_up_element_>=0)
   {
       state_=5;
       std_msgs::Bool stop;
       stop.data=true;
       path_stop_publisher_.publish(stop);
   }
   else if(path_done_&&planned_element_>=0&&picked_up_element_-1) //////////////think what happens if the robot is not able to find its goal
   {
       // robot turn a bit move a bit
       ros::spinOnce();
       if(!obstacle_)
       {
           ObstacleList_[planned_element_].try_counter++;
           state_=2;
       }
   }

   else if(round1_&&!go_home_&&(round_time_-(ros::Time::now()-run_time_).toSec())<60)
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
   else if(path_done_)
   {
       state_=2;
   }
}


void Brain::pathDoneCallback(const std_msgs::Bool &msg)
{
    if(msg.data)
    {
        path_done_=true;
    }

    //ROS_INFO("Path_Message");

}

void Brain::robotPositionCallback(const geometry_msgs::PoseStamped &msg)
{
    actual_robot_position_=msg.pose;

    double dif_pose_x=std::abs(actual_robot_position_.position.x-robot_home_position_.position.x);
    double dif_pose_y=std::abs(actual_robot_position_.position.y-robot_home_position_.position.y); /////////////////Check the orientation
    if(dif_pose_x<home_accurancy_&&dif_pose_y<home_accurancy_)
    {
        home_=true;
    }
    else
    {
        home_=false;
    }


}

void Brain::newMapCallback(const std_msgs::Bool &msg)
{
    if (msg.data)
    {
        path_done_=true;

    }

}
} /* namespace */
