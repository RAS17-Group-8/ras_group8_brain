#include <ras_group8_brain/Brain.hpp>

// STD
#include <string>

namespace ras_group8_brain {



bool  Brain::obstacleState()
{

    //see a new obstacle
    actual_obstacle_.position.x=4;
    actual_obstacle_.position.y=4;
    actual_obstacle_.shape="cube";
    actual_obstacle_.color="green";
    actual_obstacle_.recovered=false;

    if(true) //valuable obstacle
    {
        if (!Brain::ValuableObstacle(&actual_obstacle_))
            return false;
    }
    else if(false) //removable obstacle
    {
        if (!Brain::RemovableObstacle(&actual_obstacle_))
            return false;
    }
    else
    {
        if (!Brain::SolidObstacle(&actual_obstacle_))
            return false;
    }

   return true;
}

bool Brain::ValuableObstacle(struct Brain::Obstacle *obstacle)
{
    int list_element;
    if(!Brain::defineObstacleValue(obstacle))
        return false;

    if(Brain::addObstacleToList(obstacle, &list_element))
    {
        std_msgs::String obstacle_text;
        obstacle_text.data="I see the "+obstacle->color+" "+obstacle->shape;
        Brain::Speak(obstacle_text);
        ROS_INFO("ObstacleState: %s ",obstacle_text.data.c_str());
    }

    if(list_element==planned_element)
    {
        ///////tf
        if(!pickUpArm(obstacle->position))
            return false;

        picked_up_element=planned_element;
        planned_element=-1;
    }



    return true;

}


bool Brain::addObstacleToList(struct Brain::Obstacle *obstacle, int *list_element)
{
    for (int i=0; i<ObstacleList_.size();i++)
    {
        double position_x=abs(ObstacleList_[i].position.x-obstacle->position.x);
        double position_y=abs(ObstacleList_[i].position.y-obstacle->position.y);

        if(position_x<obstacle_position_accurancy_&&position_y<obstacle_position_accurancy_&&
           !ObstacleList_[i].shape.compare(obstacle->shape)&&!ObstacleList_[i].color.compare(obstacle->color))
        {
            *list_element=i;
            ROS_INFO("ObstacleState: Element Exists ");
            return false;
        }
    }
    *list_element=ObstacleList_.size();
    ObstacleList_.push_back(*obstacle);
    return true;
}

bool Brain::defineObstacleValue(struct Brain::Obstacle *obstacle)
{
    if(!obstacle->shape.compare("cube")&& !obstacle->color.compare("red"))
      obstacle->value=value_group1_;

    else if(!obstacle->shape.compare("hollow cube")&& !obstacle->color.compare("red"))
      obstacle->value=value_group1_;

    else if(!obstacle->shape.compare("cube")&& !obstacle->color.compare("blue"))
      obstacle->value=value_group1_;

    else if(!obstacle->shape.compare("cube")&& !obstacle->color.compare("green"))
     obstacle->value=value_group2_;

    else if(!obstacle->shape.compare("cube")&& !obstacle->color.compare("yellow"))
     obstacle->value=value_group2_;

    else if(!obstacle->shape.compare("ball")&& !obstacle->color.compare("yellow"))
     obstacle->value=value_group2_;

    else if(!obstacle->shape.compare("ball")&& !obstacle->color.compare("red"))
     obstacle->value=value_group3_;

    else if(!obstacle->shape.compare("cylinder")&& !obstacle->color.compare("green"))
     obstacle->value=value_group3_;

    else if(!obstacle->shape.compare("triangle")&& !obstacle->color.compare("blue"))
     obstacle->value=value_group3_;

    else if(!obstacle->shape.compare("cross")&& !obstacle->color.compare("purble"))
     obstacle->value=value_group4_;

    else if(!obstacle->shape.compare("star")&& !obstacle->color.compare("purple"))
     obstacle->value=value_group4_;

    else if(!obstacle->shape.compare("star")&& !obstacle->color.compare("orange"))
     obstacle->value=value_group4_;

    else
    {
        ROS_ERROR("ObstacleState: Obstacle not known");

        obstacle->value=0;
        return false;

    }
    return true;
}

bool Brain::RemovableObstacle(struct Brain::Obstacle *obstacle)
{
    if(!pickUpArm(obstacle->position))
        return false;

    //Robo Turn round

    if(!putDownArm(home_obstacle_pos_))
        return false;

    //Robo Back

    return true;
}

bool Brain::SolidObstacle(struct Brain::Obstacle *obstacle)
{


    ///add to map
    return true;
}




} /* namespace */
