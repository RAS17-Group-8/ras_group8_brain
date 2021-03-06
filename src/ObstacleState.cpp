#include <ras_group8_brain/Brain.hpp>

// STD
#include <string>
#include <cmath>

namespace ras_group8_brain {



bool  Brain::obstacleState()
{
    ROS_INFO("Obstacle State");
    speaking.data="Obstacle State";
    Brain::Speak(speaking);

    for (int i=0;i<3;i++)
    {

        if(new_obstacle_global_[i].number>0 && new_obstacle_global_[i].number<=15) //valuable obstacle
        {
            speaking.data="Valuable obstacle";
            Brain::Speak(speaking);

            if (!Brain::driveToObstacle(i))
            {
                ROS_ERROR("ObstacleState: Not able to drive near to an obstacle");
            }

            std_msgs::Bool stop;
            stop.data=true;
            path_stop_publisher_.publish(stop);

            if (!Brain::ValuableObstacle(&new_obstacle_global_[i],i))
            {
                ROS_INFO("Problem with valuable obastcle");
            }

            if(round1_)
            {
                state_=1;
            }
            else
            {
                state_=2;
            }

        }
        else if (new_obstacle_global_[i].number==16) //Solide obstacle
        {
            speaking.data="Solide Obstacle";
            Brain::Speak(speaking);

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
        else if(new_obstacle_global_[i].number==17) //removable obstacle
        {
            speaking.data="Trap";
            Brain::Speak(speaking);

            if (!Brain::RemovableObstacle(&new_obstacle_global_[i]))
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

    //Start the Rosbag
    char buffer[100];
    int nevidence;
    nevidence=sprintf(buffer, "rosrun ras_group8_lidar publish_evidence __name:=evidence %d %f %f",
                      obstacle_global->number,obstacle_global->position.x,obstacle_global->position.y);

    std::string evi=std::string(buffer);
    system(evi.c_str());

    std_msgs::String obstacle_text;
    obstacle_text.data="I see the "+obstacle_global->text;
    Brain::Speak(obstacle_text);
    ROS_INFO("ObstacleState: %s ",obstacle_text.data.c_str());

    if(Brain::addObstacleToList(obstacle_global, &list_element))
    {
        pointVizualisation(obstacle_global->position); 
    }

    if(list_element==planned_element_)
    {
        if(!pickUpArm(msg_num,planned_element_))
            return false;

        picked_up_element_=planned_element_;
        planned_element_=-1;

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
            speaking.data="I know this object already";
            Brain::Speak(speaking);

            *list_element=i;
            ObstacleList_[i].recovered=false;
            ObstacleList_[i].try_counter=0;
            ROS_INFO("ObstacleState: Element Exists %d ",i);
            if(picked_up_element_==i)
            {
                picked_up_element_=-1;
                path_done_=true;
                ROS_INFO("Delet Pickup Elment");
            }


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

    //Add Obstacle to the map
    visualization_msgs::MarkerArray new_marker;
    new_marker.markers.resize(1);
    new_marker.markers[0].header.frame_id="/map";
    new_marker.markers[0].header.stamp=ros::Time::now();
    new_marker.markers[0].type=2;
    new_marker.markers[0].pose.position=obstacle->position;
    new_marker.markers[0].scale.x=0.04;
    add_object_publisher_.publish(new_marker);

    return true;
}


bool Brain::RemovableObstacle(struct Brain::Obstacle *obstacle)
{
//    if(!pickUpArm(msg_num))
//        return false;
    ///////////////////Turn around//////////////////////////
//    nav_msgs::Path path;
//    path.poses.resize(1);
//    path.poses[0].pose.position=actual_robot_position_.position;
//    path.poses[0].pose.orientation=actual_robot_position_.orientation;
//    path.header.stamp=ros::Time::now();
//    path.header.frame_id="/map";
//    send_path_publisher_.publish(path);
    ////////////////////////
//    if(!putDownArm(home_obstacle_pos_))
//        return false;

    //Robo Back


    //Add to the map
    visualization_msgs::MarkerArray new_marker;
    new_marker.markers.resize(1);
    new_marker.markers[0].header.frame_id="/map";
    new_marker.markers[0].header.stamp=ros::Time::now();
    new_marker.markers[0].type=2;
    new_marker.markers[0].pose.position=obstacle->position;
    new_marker.markers[0].scale.x=0.08;
    add_object_publisher_.publish(new_marker);
    return true;
}

bool Brain::SolidObstacle(struct Brain::Obstacle *obstacle)
{

    //Add to the map
    visualization_msgs::MarkerArray new_marker;
    new_marker.markers.resize(1);
    new_marker.markers[0].header.frame_id="/map";
    new_marker.markers[0].header.stamp=ros::Time::now();
    new_marker.markers[0].type=2;
    new_marker.markers[0].pose.position=obstacle->position;
    new_marker.markers[0].scale.x=0.10;
    add_object_publisher_.publish(new_marker);

    return true;
}

void Brain::visionMessageCallback(const ras_group8_brain::Vision &msg)
{
    new_obstacle_msg_=msg;
    obstacle_=false;
    for (int i=0;i<3;i++)
    {
        if (msg.number[i]!=0)
        {
            obstacle_=true;
            //ROS_INFO("Vision Message x:%f y:%f z:%f",msg.position[i].point.x,msg.position[i].point.y,msg.position[i].point.z);

            geometry_msgs::PointStamped obstacle_robot_frame=new_obstacle_msg_.position[i];
            geometry_msgs::PointStamped obstacle_position_global;

            try{
                tf_listener_.waitForTransform("/base_link", "/camera_depth_optical_frame",
                              ros::Time::now(), ros::Duration(3.0));
                tf_listener_.transformPoint("/base_link",obstacle_robot_frame,obstacle_position_global);

            }
            catch(tf::TransformException ex){
                ROS_ERROR("Transformation:%s",ex.what());
            }

            new_obstacle_global_[i].position=obstacle_position_global.point;

            //////////////////////Testing
                  // obstacle_position_global.point.x=0.8;  //change to message
                   //obstacle_position_global.point.y=1.6;   //change to message
            /////////////////////
            new_obstacle_global_[i].number=new_obstacle_msg_.number[i];
        }
    }
}

bool Brain::driveToObstacle(int msg_num)
{
    if(new_obstacle_msg_.position[msg_num].point.x <obstacle_detection_accurancy_&&new_obstacle_msg_.position[msg_num].point.y < obstacle_detection_accurancy_)
    {
        ROS_INFO("ObstacleState:Near to the obstalce");
        return true;
    }
//    double orientation_obst=atan2(new_obstacle_msg_.position[msg_num].point.x,new_obstacle_msg_.position[msg_num].point.y); //Obstacle

//    double siny=2*actual_robot_position_.orientation.w*actual_robot_position_.orientation.z;
//    double cosy=1-2*actual_robot_position_.orientation.z*actual_robot_position_.orientation.z;
//    double orientation=atan2(siny,cosy)+orientation_obst;

//    geometry_msgs::Quaternion new_orientation;
//    new_orientation.z=sin(0.5*orientation);
//    new_orientation.w=cos(0.5*orientation);

//    nav_msgs::Path path;
//    path.poses.resize(1);
//    path.poses[0].pose.position.x=new_obstacle_global_[msg_num].position.x-cos(orientation)*obstacle_detection_accurancy_;
//    path.poses[0].pose.position.y=new_obstacle_global_[msg_num].position.y-sin(orientation)*obstacle_detection_accurancy_;
//    path.poses[0].pose.orientation=new_orientation;
//    path.header.stamp=ros::Time::now();
//    path.header.frame_id="/map";
//    send_path_publisher_.publish(path);

    nav_msgs::Path path;
    path.poses.resize(2);
    path.poses[0].pose.position.x=actual_robot_position_.position.x ;
    path.poses[0].pose.position.y=actual_robot_position_.position.y ;
    path.poses[1].pose.position.x=new_obstacle_msg_.position[msg_num].point.x ;
    path.poses[1].pose.position.y=new_obstacle_msg_.position[msg_num].point.y ;
    path.header.stamp=ros::Time::now();
    path.header.frame_id="/map";
    send_path_publisher_.publish(path);

    ros::Duration wait_time(0.5);


    for (int i=0; i<6;i++)
    {
        ROS_INFO("Wait for driving");
        if(new_obstacle_msg_.position[msg_num].point.x <obstacle_detection_accurancy_&&new_obstacle_msg_.position[msg_num].point.y < obstacle_detection_accurancy_)
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
