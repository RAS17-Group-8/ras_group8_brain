#include <ras_group8_brain/Brain.hpp>

// STD
#include <string>

namespace ras_group8_brain {


int  Brain::homeState()
{
    ROS_INFO("Home State");
    if(!putDownArm(home_obstacle_pos_))
        return false;
    if(picked_up_element_>=0 && picked_up_element_<ObstacleList_.size())
    {
        ObstacleList_[picked_up_element_].recovered=true;
    }
    picked_up_element_=-1;
    state_=2;
    return true;
}




} /* namespace */
