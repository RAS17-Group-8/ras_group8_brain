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

        if(new_obstacle_global_[i].number>0 && new_obstacle_global_[i].number<=14) //valuable obstacle
        {

            if (!Brain::driveToObstacle(i))
            {
                ROS_ERROR("ObstacleState: Not able to drive near to an obstacle");
            }

            std_msgs::Bool stop;
            stop.data=true;
            path_stop_publisher_.publish(stop);


            if (!Brain::ValuableObstacle(&new_obstacle_global_[i],i))
            {
                return false;
            }

            /////////////Send old path again//////////////////
            /////////////how do we extract finished points///////////////
            actual_path_.header.stamp=ros::Time::now();
            send_path_publisher_.publish(actual_path_);


            ///////////////////////////////////////////

            state_=3;
        }
        else if (new_obstacle_global_[i].number==15) //Solide obstacle
        {
            if (!Brain::SolidObstacle(&new_obstacle_global_[i]))
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
        else if(new_obstacle_global_[i].number==16) //removable obstacle
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
    {
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


        for (int i=0;i<3;i++)
        {
            ROS_INFO("ObstacleMesage1 %s",msg.position[i].header.frame_id.c_str());
            ROS_INFO("Vision Message x:%f y:%f z:%f",msg.position[i].point.x,msg.position[i].point.y,msg.position[i].point.z);

            geometry_msgs::PointStamped obstacle_robot_frame=new_obstacle_msg_.position[i];
            geometry_msgs::PointStamped obstacle_position_global;

            try{
                tf_listener_.transformPoint("/robot",obstacle_robot_frame,obstacle_position_global);

            }
            catch(tf::TransformException ex){
                ROS_ERROR("Transformation:%s",ex.what());
            }


            new_obstacle_global_[i].position=obstacle_position_global.point;

            //////////////////////
//                    obstacle_position_global.point.x=0.8;  //change to message
//                    obstacle_position_global.point.y=1.6;   //change to message
            /////////////////////
            new_obstacle_global_[i].number=new_obstacle_msg_.number[i];
        }
    }
    else
    {
        obstacle_=false;
    }
        ROS_INFO("ObstacleMesage2");
}

bool Brain::driveToObstacle(int msg_num)
{
    if(new_obstacle_msg_.position[msg_num].point.x <obstacle_detection_accurancy_&&new_obstacle_msg_.position[msg_num].point.y < obstacle_detection_accurancy_)
    {
        ROS_INFO("ObstacleState:Near to the obstalce");
        return true;
    }
    double orientation_obst=atan2(new_obstacle_msg_.position[msg_num].point.x,new_obstacle_msg_.position[msg_num].point.y); //Obstacle

    double siny=2*actual_robot_position_.orientation.w*actual_robot_position_.orientation.z;
    double cosy=1-2*actual_robot_position_.orientation.z*actual_robot_position_.orientation.z;
    double orientation=atan2(siny,cosy)+orientation_obst;

    geometry_msgs::Quaternion new_orientation;
    new_orientation.z=sin(0.5*orientation);
    new_orientation.w=cos(0.5*orientation);

    nav_msgs::Path path;
    path.poses.resize(1);
    path.poses[0].pose.position.x=new_obstacle_global_[msg_num].position.x-cos(orientation)*obstacle_detection_accurancy_;
    path.poses[0].pose.position.y=new_obstacle_global_[msg_num].position.y-sin(orientation)*obstacle_detection_accurancy_;
    path.poses[0].pose.orientation=new_orientation;
    path.header.stamp=ros::Time::now();
    path.header.frame_id="/map";
    send_path_publisher_.publish(path);


    ros::Duration wait_time(0.5);


    for (int i=0; i<6;i++)
    {
        ROS_INFO("Wait for driving");
        if(path_done_)
        {
            return true;
        }
        else
        {
            wait_time.sleep();

        }
        ros::spinOnce();
    }
    return false;

}

} /* namespace */
