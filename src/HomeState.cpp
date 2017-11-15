#include <ras_group8_brain/Brain.hpp>

// STD
#include <string>

namespace ras_group8_brain {


int  Brain::homeState()
{
    if(!putDownArm(home_obstacle_pos_))
        return false;
    if(picked_up_element>=0 && picked_up_element<ObstacleList_.size())
    {
        ObstacleList_[picked_up_element].recovered=true;
    }
    return true;
}




} /* namespace */
