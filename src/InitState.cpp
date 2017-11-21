#include <ras_group8_brain/Brain.hpp>

// STD
#include <string>

namespace ras_group8_brain {


bool  Brain::initState()
{
    //////////////////Only for Testing//////////////////////

//     struct Brain::Obstacle obstacle_test;

//     obstacle_test.position.x=1.4;
//     obstacle_test.position.y=1.7;
//     obstacle_test.number=2;
//     obstacle_test.value=10000;
//     obstacle_test.text="green cube";
//     obstacle_test.recovered=false;
//     pointVizualisation(obstacle_test.position);

//     ObstacleList_.push_back(obstacle_test);

//     obstacle_test.position.x=1.2;
//     obstacle_test.position.y=1.8;
//     obstacle_test.number=7;
//     obstacle_test.value=1000;
//     obstacle_test.text="yellow cube";
//     obstacle_test.recovered=false;
//     pointVizualisation(obstacle_test.position);

//     ObstacleList_.push_back(obstacle_test);

//     obstacle_test.position.x=0.15;
//     obstacle_test.position.y=1.6;
//     obstacle_test.number=6;
//     obstacle_test.value=100;
//     obstacle_test.text="orange cube";
//     obstacle_test.recovered=false;

//     ObstacleList_.push_back(obstacle_test);
//     pointVizualisation(obstacle_test.position);


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
