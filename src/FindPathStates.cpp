#include <ras_group8_brain/Brain.hpp>

// STD

namespace ras_group8_brain {


bool Brain::findPathState()
{
    ROS_INFO("Find Path State");
    if(picked_up_element_>=0)
    {
        if(!findHomePath())
        {
            state_=1;
            return false;
        }
    }
    else
    {       
        if(!findElementPath())
        {
            state_=1;
            return false;
        }
    }
    state_=3;
    pathVizualisation(&actual_path_);
    send_path_publisher_.publish(actual_path_);
    return true;
}

bool Brain::explorState()
{
    findGoalPath();
    ROS_INFO("Explore State");
    state_=3;
    pathVizualisation(&actual_path_);
    send_path_publisher_.publish(actual_path_);
    return true;
}

bool Brain::findElementPath()
{
    int num_ob;
    nav_msgs::GetPlan path;
    int actual_gain=0;
    int last_gain=0;

    planned_element_=-1;

    path.request.start.pose=actual_robot_position_;

    if((num_ob=ObstacleList_.size())==0)
    {
        ROS_ERROR("FindPathState: No Obstacles in the List");
        return false;
    }
    //ROS_ERROR("Number %d",num_ob);
    for(int i=0; i<num_ob; i++)
    {
        if (!ObstacleList_[i].recovered && ObstacleList_[i].try_counter<=obstacle_try_)
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
                planned_element_=i;
            }
            ROS_INFO("PathCost %i",actual_gain);
        }
    }
    return true;

}

bool Brain::findHomePath()
{
    nav_msgs::GetPlan path;

    planned_element_=-1;

    path.request.start.pose=actual_robot_position_;
    path.request.goal.pose=robot_home_position_;

    if(!get_path_client_.call(path))
    {
        ROS_ERROR("FindPathState: Not abele to compute Path");
        return false;
    }
    actual_path_=path.response.plan;

    return true;
}

bool Brain::findGoalPath()
{
    nav_msgs::GetPlan path;

    planned_element_=-1;

    path.request.start.pose=actual_robot_position_;
    path.request.goal.pose=robot_goal_rviz_;

    if(!get_path_client_.call(path))
    {
        ROS_ERROR("FindPathState: Not abele to compute Path");
        return false;
    }
    actual_path_=path.response.plan;

    return true;
}

void Brain::goalMessageCallback(const geometry_msgs::PoseStamped &msg)
{
    state_=1;
    ROS_INFO("FindPathState: Recive a goal");

    robot_goal_rviz_=msg.pose;

}

} /* namespace */
