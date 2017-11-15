#include <ras_group8_brain/Brain.hpp>

// STD
#include <string>

namespace ras_group8_brain {

Brain::Brain(ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  
  //subscriber_ = node_handle_.subscribe(subscriber_topic_, 1, &Brain::topicCallback, this);

  //Services
  get_path_client_=node_handle_.serviceClient<nav_msgs::GetPlan>(get_path_service_);
  move_arm_up_client_=node_handle_.serviceClient<ras_group8_arm_controller::MoveArm>(move_arm_up_service_);
  move_arm_down_client_=node_handle_.serviceClient<ras_group8_arm_controller::MoveArm>(move_arm_down_service_);

  //Publisher
  speaker_publisher_=node_handle_.advertise<std_msgs::String>(speaker_topic_,1,true);


  ROS_INFO("Successfully launched node.");

  //////////////Test
//  geometry_msgs::Point position;
//  position.x=0;
//  position.y=0;

//  Brain::pickUpState (position);
//////////////////////////////////////////
//  std_msgs::String msg1;
//  msg1.data="test";

//  Brain::Speak(msg1);
////////////////////////////////
//  Brain::obstacleState();
//  Brain::obstacleState();

}

Brain::~Brain()
{
}

bool Brain::readParameters()
{
  if (!node_handle_.getParam("get_path_service", get_path_service_))
    return false;
  if (!node_handle_.getParam("move_arm_up_service", move_arm_up_service_))
    return false;
  if (!node_handle_.getParam("move_arm_down_service", move_arm_down_service_))
    return false;
  if (!node_handle_.getParam("speaker_topic", speaker_topic_))
    return false;
  if (!node_handle_.getParam("arm_up/pickup_attempt", pickup_attempt_))
    return false;
  if (!node_handle_.getParam("arm_up/pickup_range", pickup_range_))
    return false;
  if (!node_handle_.getParam("obstacle/position_accurancy", obstacle_position_accurancy_))
    return false;
  if (!node_handle_.getParam("obstacle/value_group1", value_group1_))
    return false;
  if (!node_handle_.getParam("obstacle/value_group2", value_group2_))
    return false;
  if (!node_handle_.getParam("obstacle/value_group3", value_group3_))
    return false;
  if (!node_handle_.getParam("obstacle/value_group4", value_group4_))
    return false;
  if (!node_handle_.getParam("home/obstacle_x", home_obstacle_pos_.x))
    return false;
  if (!node_handle_.getParam("home/obstacle_y", home_obstacle_pos_.y))
    return false;
  if (!node_handle_.getParam("home/obstacle_z", home_obstacle_pos_.z))
    return false;
  if (!node_handle_.getParam("home/position_x", robot_home_position_.x))
    return false;
  if (!node_handle_.getParam("home/position_y", robot_home_position_.y))
    return false;

  return true;
}



} /* namespace */
