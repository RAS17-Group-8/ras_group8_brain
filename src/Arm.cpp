#include <ras_group8_brain/Brain.hpp>

// STD
#include <stdlib.h>
#include <time.h>
namespace ras_group8_brain {

bool  Brain::pickUpArm(geometry_msgs::Point position)
{
    ras_group8_arm_controller::MoveArm arm_msg;
    geometry_msgs::Point arm_position;

    arm_position.x=100*(position.y);
    arm_position.y=100*(position.x+0.045);
    arm_position.z=100*(position.z-0.14);

    //srand(time(NULL)); ??
    for (int i=0; i<pickup_attempt_; i++)
    {
        arm_msg.request.position.x=arm_position.x+rand()%pickup_range_;
        arm_msg.request.position.y=arm_position.y+rand()%pickup_range_;
        arm_msg.request.position.z=arm_position.z;

        ROS_INFO("%d x: %f, y:%f ",pickup_range_,arm_msg.request.position.x,arm_msg.request.position.y);

        if(!move_arm_up_client_.call(arm_msg))
        {
           ROS_ERROR("PickUpState: Movement was not possible");
           return false;
        }
        ros::spinOnce();


        if(!obstacle_) //obstacle lift up
        {
            return true;
        }
         ///////////should we change the position for each attempt//////////////////
        ///////////compare the obstacle value////////////////////////////////
//        else
//        {
//            arm_position.x=100*(new_obstacle_msg.position.y);
//            arm_position.y=100*(new_obstacle_msg.position.x+0.045);
//            arm_position.z=100*(new_obstacle_msg.position.z-0.14);

//        }

    }
  ROS_INFO("PickUpState: Arm was not able to pickup obstacle");
  return false;
}

bool  Brain::putDownArm(geometry_msgs::Point position)
{
    ras_group8_arm_controller::MoveArm arm_msg;
    arm_msg.request.position.x=100*(position.y);
    arm_msg.request.position.y=100*(position.x+0.045);
    arm_msg.request.position.z=100*(position.z-0.14);

    if(!move_arm_up_client_.call(arm_msg))
    {
        ROS_ERROR("PutDownState: Get no Service Response");
        return false;
    }
    return true;
}


} /* namespace */
