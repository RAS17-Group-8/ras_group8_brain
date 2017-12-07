#include <ras_group8_brain/Brain.hpp>

// STD
#include <string>

namespace ras_group8_brain {

bool  Brain::initState()
{
    //////////////////Only for Testing//////////////////////
     actual_robot_position_.position.x=0.2;
     actual_robot_position_.position.y=0.14;
    ////////////////////////////////////////////////////////////

   ROS_INFO("Init State");
   planned_element_=-1;
   planned_edge_=-1;
   picked_up_element_=-1;
   path_done_=false;
   obstacle_=false;
   home_=false;
   go_home_=false;
   recieved_pose_=false;

   std_msgs::String greeting;
   greeting.data="Hello I'm a robot";
   Brain::Speak(greeting);

   if(!Brain::readTextfile())
   {
       ROS_ERROR("InitState: Not possible to read obstacle file");
   }

   if (round1_)
   {
       state_=1;
   }
   else
   {
       state_=2;
   }
   return true;
}




} /* namespace */
