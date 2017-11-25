#include <ras_group8_brain/Brain.hpp>

// STD
#include <stdlib.h>
#include <time.h>
namespace ras_group8_brain {

bool  Brain::pickUpArm()
{
    ras_group8_arm_controller::MoveArm arm_msg;
    geometry_msgs::Point arm_position;

    for (int i=0; i<pickup_attempt_; i++)
    {
        ///////////should we change the position for each attempt//////////////////
        arm_position.x=100*(new_obstacle_msg_.position.point.y);
        arm_position.y=100*(new_obstacle_msg_.position.point.x+0.08);
        arm_position.z=100*(new_obstacle_msg_.position.point.z-0.16);

        arm_msg.request.position.x=arm_position.x+rand()%(2*pickup_range_)-pickup_range_;
        arm_msg.request.position.y=arm_position.y+rand()%(2*pickup_range_)-pickup_range_;
        arm_msg.request.position.z=arm_position.z;

        ROS_INFO("ArmCoordinates x: %f, y:%f Z:%f",arm_msg.request.position.x,arm_msg.request.position.y,arm_msg.request.position.z);

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
    }
  ROS_INFO("PickUpState: Arm was not able to pickup obstacle");
  return false;
}

bool  Brain::putDownArm(geometry_msgs::Point position)
{
    ras_group8_arm_controller::MoveArm arm_msg;
    arm_msg.request.position.x=100*(position.y);
    arm_msg.request.position.y=100*(position.x+0.08);
    arm_msg.request.position.z=100*(position.z-0.16);

    if(!move_arm_up_client_.call(arm_msg))
    {
        ROS_ERROR("PutDownState: Get no Service Response");
        return false;
    }
    return true;
}


} /* namespace */
