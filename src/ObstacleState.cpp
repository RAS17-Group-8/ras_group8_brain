#include <ras_group8_brain/Brain.hpp>

// STD
#include <string>
#include <cmath>

namespace ras_group8_brain {



bool  Brain::obstacleState()
{
    ROS_INFO("Obstacle State");

    for (int i=0;i<3;i++)
    {
//        geometry_msgs::PointStamped obstacle_robot_frame=new_obstacle_msg_.position[i];
//        geometry_msgs::PointStamped obstacle_position_global;

//        try{
//            tf_listener_.transformPoint("/map",obstacle_robot_frame,obstacle_position_global);

//        }
//        catch(tf::TransformException ex){
//            ROS_ERROR("Transformation:%s",ex.what());
//        }
//        new_obstacle_global_.position=obstacle_position_global.point;
//        new_obstacle_global_.number=new_obstacle_msg_.number[i];

        //////////////////////////////////////Change message in global position////////////////
        new_obstacle_global_.position.x=0.8;  //change to message
        new_obstacle_global_.position.y=1.6;   //change to message
        new_obstacle_global_.number=new_obstacle_msg_.number[i];
        ///////////////////////////////////////////////////////////////////////////////////////


        if(new_obstacle_global_.number>0 && new_obstacle_global_.number<=14) //valuable obstacle
        {
            if (!Brain::ValuableObstacle(&new_obstacle_global_,i))
            {
                return false;
            }
            state_=3;
        }
        else if (new_obstacle_global_.number==15) //Solide obstacle
        {
            if (!Brain::SolidObstacle(&new_obstacle_global_))
                return false;
            if(round1_)
            {
                state_=1;
            }
            else
            {
                state_=2;
            }

        }
        else if(new_obstacle_global_.number==16) //removable obstacle
        {
            if (!Brain::RemovableObstacle(i))
                return false;
            state_=3;
        }
        else
        {
            ROS_INFO("ObstacleState:No_Obstacle");
            obstacle_=false;
            state_=3;
            return false;
        }
    }
    return true;
}

bool Brain::ValuableObstacle(struct Brain::Obstacle *obstacle_global, int msg_num)
{
    int list_element;
    bool pickup=true;

    if(Brain::addObstacleToList(obstacle_global, &list_element))
    {
        pointVizualisation(obstacle_global->position);
        std_msgs::String obstacle_text;
        obstacle_text.data="I see the "+obstacle_global->text;
        Brain::Speak(obstacle_text);
        ROS_INFO("ObstacleState: %s ",obstacle_text.data.c_str());
    }

    if(list_element==planned_element_)
    { ///////////////
//        nav_msgs::Path path;
//        path.poses.resize(1);
//        path.poses[0].pose.position=actual_robot_position_.position;
//        path.poses[0].pose.orientation=actual_robot_position_.orientation;
//        path.header.stamp=ros::Time::now();
//        path.header.frame_id="/map";
//        send_path_publisher_.publish(path);
      ////////////////
        if(!pickUpArm(msg_num))
            return false;

        picked_up_element_=planned_element_;
        planned_element_=-1;
    }
    if (list_element=!planned_element_||pickup==false)
    {
        //continue to drive
    }
    return true;

}


bool Brain::addObstacleToList(struct Brain::Obstacle *obstacle, int *list_element)
{
    for (int i=0; i<ObstacleList_.size();i++)
    {
        double position_x=std::abs(ObstacleList_[i].position.x-obstacle->position.x);
        double position_y=std::abs(ObstacleList_[i].position.y-obstacle->position.y);

     ////////////only for test
//        if(position_x<obstacle_position_accurancy_&&position_y<obstacle_position_accurancy_
//           &&ObstacleList_[i].number==obstacle->number)
        /////////////////////
        if(ObstacleList_[i].number==obstacle->number)
        {
            *list_element=i;
            ObstacleList_[i].recovered=false;
            ObstacleList_[i].try_counter=0;
            ROS_INFO("ObstacleState: Element Exists %d ",i);
            return false;
        }
    }
    *list_element=ObstacleList_.size();

    obstacle->text=possible_obstacle_[obstacle->number].name;
    obstacle->value=possible_obstacle_[obstacle->number].value;
    obstacle->recovered=false;
    obstacle->try_counter=0;


    ObstacleList_.push_back(*obstacle);
    Brain::writeTextfile(obstacle);
    pointVizualisation(obstacle->position);
    return true;
}


bool Brain::RemovableObstacle(int msg_num)
{
    if(!pickUpArm(msg_num))
        return false;
    ///////////////////Turn around//////////////////////////
//    nav_msgs::Path path;
//    path.poses.resize(1);
//    path.poses[0].pose.position=actual_robot_position_.position;
//    path.poses[0].pose.orientation=actual_robot_position_.orientation;
//    path.header.stamp=ros::Time::now();
//    path.header.frame_id="/map";
//    send_path_publisher_.publish(path);
    ////////////////////////
    if(!putDownArm(home_obstacle_pos_))
        return false;

    //Robo Back

    return true;
}

bool Brain::SolidObstacle(struct Brain::Obstacle *obstacle)
{

    /////////////////////
    ///add to map
    /// ////////////////////////
    return true;
}

void Brain::visionMessageCallback(const ras_group8_brain::Vision &msg)
{
    if (msg.number[0]!=0)
    {
        obstacle_=true;
        new_obstacle_msg_=msg;

    }
    else
    {
        obstacle_=false;
    }
}


} /* namespace */
