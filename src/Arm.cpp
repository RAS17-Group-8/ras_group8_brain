#include <ras_group8_brain/Brain.hpp>

// STD
#include <stdlib.h>
#include <time.h>
namespace ras_group8_brain {

bool  Brain::pickUpArm(geometry_msgs::Point position)
{
    //tf?????
    ras_group8_arm_controller::MoveArm arm_msg;

    //srand(time(NULL)); ??
    for (int i=0; i<pickup_attempt_; i++)
    {
        arm_msg.request.position.x=position.x+rand()%pickup_range_;
        arm_msg.request.position.y=position.y+rand()%pickup_range_;
        arm_msg.request.position.z=position.z;

        ROS_INFO("%i x: %f, y:%f ",pickup_range_,arm_msg.request.position.x,arm_msg.request.position.y);

        if(!move_arm_up_client_.call(arm_msg))
        {
           ROS_ERROR("PickUpState: Movement was not possible");
           return false;
        }


        if(i==5) //obstacle lift up
        {
            return true;
        }

    }
  ROS_INFO("PickUpState: Arm was not able to pickup obstacle");
  return false;
}

bool  Brain::putDownArm(geometry_msgs::Point position)
{
    ras_group8_arm_controller::MoveArm arm_msg;
    arm_msg.request.position.x=position.x;
    arm_msg.request.position.y=position.y;
    arm_msg.request.position.z=position.z;

    if(!move_arm_up_client_.call(arm_msg))
    {
        ROS_ERROR("PutDownState: Get no Service Response");
        return false;
    }
    return true;
}


} /* namespace */
