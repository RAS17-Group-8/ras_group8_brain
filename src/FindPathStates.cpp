#include <ras_group8_brain/Brain.hpp>

// STD

namespace ras_group8_brain {


bool Brain::findPathState()
{
    if(true)  //find element path()
    {
        if(!findElementPath())
            return false;
    }
    else   //find home path()
    {
        if(!findHomePath())
            return false;
    }

    return true;
}

bool Brain::explorState()
{

}

bool Brain::findElementPath()
{
    /////////////////////test

    actual_robot_position_.x=0.15;
    actual_robot_position_.y=0.15;

    ObstacleList_.clear();
    Brain::Obstacle new_obstacle;
    new_obstacle.position.x=2.2;
    new_obstacle.position.y=1.9;
    new_obstacle.value=10000;

    ObstacleList_.push_back(new_obstacle);

    new_obstacle.position.x=2.2;
    new_obstacle.position.y=0.2;
    new_obstacle.value=10000;

    ObstacleList_.push_back(new_obstacle);
    //////////////////////////////////
    int num_ob;
    nav_msgs::GetPlan path;
    int actual_gain=0;
    int last_gain=0;

    planned_element=-1;

    path.request.start.pose.position=actual_robot_position_;

    if((num_ob=ObstacleList_.size())==0)
    {
        ROS_ERROR("FindPathState: No Obstacles in the List");
        return false;
    }
    for(int i; i<num_ob; i++)
    {
        if (!ObstacleList_[i].recovered)
        {
            path.request.goal.pose.position=ObstacleList_[i].position;

            if(!get_path_client_.call(path))
            {
                ROS_ERROR("FindPathState: Not abele to compute Path");
                return false;
            }
            actual_gain=ObstacleList_[i].value-path.response.plan.header.seq;

            if(actual_gain>last_gain)
            {
                actual_path_=path.response.plan;
                last_gain=actual_gain;
                planned_element=i;
            }
            //ROS_INFO("PathCost %i",actual_gain);
        }
    }
    return true;

}

bool Brain::findHomePath()
{
    nav_msgs::GetPlan path;

    planned_element=-1;

    path.request.start.pose.position=actual_robot_position_;
    path.request.goal.pose.position=robot_home_position_;

    if(!get_path_client_.call(path))
    {
        ROS_ERROR("FindPathState: Not abele to compute Path");
        return false;
    }
    actual_path_=path.response.plan;

    return true;
}

} /* namespace */
