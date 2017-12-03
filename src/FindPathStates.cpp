#include <ras_group8_brain/Brain.hpp>

// STD

namespace ras_group8_brain {


bool Brain::findPathState()
{
    ROS_INFO("Find Path State");
    path_done_=false;
    if(picked_up_element_>=0)
    {
        ROS_INFO("FindPathStates:Find home path");
        if(!findHomePath())
        {
            state_=1;
            return false;
        }
    }
    else
    {
        ROS_INFO("FindPathStates:Find obstacle");
        if(!findElementPath())
        {
            state_=1;
            return false;
        }
    }
    state_=3;
    pathVizualisation(&actual_path_);
    actual_path_.header.stamp=ros::Time::now();
    actual_path_.header.frame_id="/map";
    send_path_publisher_.publish(actual_path_);
    return true;
}

bool Brain::explorState()
{
    ROS_INFO("Explore State");
    path_done_=false;
    //findGoalPath();
    if(!findEdges())
    {
        return false;
    }
    state_=3;
    pathVizualisation(&actual_path_);
    actual_path_.header.stamp=ros::Time::now();
    actual_path_.header.frame_id="/map";
    send_path_publisher_.publish(actual_path_);
    return true;
}

bool Brain::findEdges()
{
    nav_msgs::GetPlan path;
    int actual_points=0;
    int last_points=0;

    planned_element_=-1;
    planned_edge_=-1;
    bool find_path=false;

    path.request.start.pose=actual_robot_position_;

    for(int i=0; i<3; i++) ///////////////change to size??
    {
        if (!edges_[i].explored)
        {
            path.request.goal.pose.position=edges_[i].point;

            if(!get_path_client_.call(path))
            {
                ROS_ERROR("ExploreState: Not abele to compute edge Path");
            }
            else
            {

                actual_points=path.response.plan.header.stamp.sec;

                if(actual_points>last_points)
                {
                    actual_path_=path.response.plan;
                    last_points=actual_points;
                    planned_edge_=i;
                }
                find_path=true;
                ROS_INFO("PathElements %i",actual_points);
            }
        }
    }
    if(!find_path)
    {
        int random_num;
        geometry_msgs::Point random_point;

        random_num=round(100*(maze_size_x_-0.3));
        random_point.x=0.01*(rand()%random_num)+0.15;
        random_num=round(100*(maze_size_y_-0.3));
        random_point.y=0.01*(rand()%random_num)+0.15;

        path.request.goal.pose.position=random_point;
        if(!get_path_client_.call(path))
        {
            ROS_ERROR("ExploreState: Not abele to compute random Path");
            return false;
        }
        actual_path_=path.response.plan;
        ROS_INFO("ExploreState:Go to random point x: %f, y:%f",random_point.x,random_point.y);
    }

    return true;
}


bool Brain::findElementPath()
{
    int num_ob;
    nav_msgs::GetPlan path;
    int actual_gain=0;
    int last_gain=-10000;
    bool find_path=false;

    planned_element_=-1;
    planned_edge_=-1;


    path.request.start.pose=actual_robot_position_;

    if((num_ob=ObstacleList_.size())==0)
    {
        ROS_ERROR("FindPathState: No Obstacles in the List");
        return false;
    }
    for(int i=0; i<num_ob; i++)
    {
        if (!ObstacleList_[i].recovered && ObstacleList_[i].try_counter<=obstacle_try_)
        {
            path.request.goal.pose.position=ObstacleList_[i].position;

            if(!get_path_client_.call(path))
            {
                ROS_ERROR("FindPathState: Not abele to compute Path");
            }
            else
            {
                double time_factor=(round_time_-(ros::Time::now()-run_time_).toSec())/(2*round_time_)+0.5;
                actual_gain=ObstacleList_[i].value-round(path.response.plan.header.seq/time_factor);

                if(actual_gain>last_gain)
                {
                    actual_path_=path.response.plan;
                    last_gain=actual_gain;
                    planned_element_=i;
                }
                ROS_INFO("PathCost %i Time factor %f",actual_gain,time_factor);
                find_path=true;
            }
        }
    }
    if(find_path)
        return true;
    else
        return false;

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
