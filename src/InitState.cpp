#include <ras_group8_brain/Brain.hpp>

// STD
#include <string>

namespace ras_group8_brain {


bool  Brain::initState()
{
    //////////////////Only for Testing//////////////////////

     actual_robot_position_.position.x=0.15;
     actual_robot_position_.position.y=0.15;

     robot_goal_rviz_=actual_robot_position_;

    ////////////////////////////////////////////////////////////

   ROS_INFO("Init State");
   planned_element_=-1;
   picked_up_element_=-1;
   path_done_=false;
   obstacle_=false;
   home_=false;

   std_msgs::String greeting;
   greeting.data="Hello I'm a robot";
   Brain::Speak(greeting);

   if(!Brain::readTextfile())
   {
       ROS_ERROR("InitState: Not possible to read obstacle file");
       ////////////ADD a new state variable///////////////
   }

 //Change state
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
