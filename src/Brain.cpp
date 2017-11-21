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
  send_path_publisher_=node_handle_.advertise<nav_msgs::Path>(send_path_topic_,1,true);
  path_stop_publisher_=node_handle_.advertise<std_msgs::Bool>(path_stop_topic_,1,true);
  marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("visualisation_marker", 1,true);

  //Subscriber
  path_done_subscriber_=node_handle_.subscribe(path_done_topic_,1,&Brain::pathDoneCallback, this);
  vision_msg_subscriber_=node_handle_.subscribe(vision_msg_topic_,1,&Brain::visionMessageCallback, this);
  set_goal_subscriber_=node_handle_.subscribe("/move_base_simple/goal",1,&Brain::goalMessageCallback, this);

  state_=0;

  ROS_INFO("Successfully launched node.");

  possible_obstacle_[0].name="no obstacle";   possible_obstacle_[0].value=0;
  possible_obstacle_[1].name="green cube";   possible_obstacle_[1].value=value_group1_;
  possible_obstacle_[2].name="blue cube";   possible_obstacle_[2].value=value_group1_;
  possible_obstacle_[3].name="yellow cube";   possible_obstacle_[3].value=value_group1_;
  possible_obstacle_[4].name="red hollow cube";   possible_obstacle_[4].value=value_group2_;
  possible_obstacle_[5].name="green hollow cube";   possible_obstacle_[5].value=value_group2_;
  possible_obstacle_[6].name="red sphere";   possible_obstacle_[6].value=value_group2_;
  possible_obstacle_[7].name="yellow sphere";   possible_obstacle_[7].value=value_group3_;
  possible_obstacle_[8].name="blue hollow triangle";   possible_obstacle_[8].value=value_group3_;
  possible_obstacle_[9].name="red hollow cylinder";   possible_obstacle_[9].value=value_group3_;
  possible_obstacle_[10].name="green hollow cylinder";   possible_obstacle_[10].value=value_group4_;
  possible_obstacle_[11].name="purple hollow cross";   possible_obstacle_[11].value=value_group4_;
  possible_obstacle_[12].name="orange hollow cross";   possible_obstacle_[12].value=value_group4_;
  possible_obstacle_[13].name="orange star";   possible_obstacle_[13].value=value_group4_;
  possible_obstacle_[14].name="purple star";   possible_obstacle_[14].value=value_group4_;
  possible_obstacle_[15].name="solide obstacle";   possible_obstacle_[15].value=0;
  possible_obstacle_[16].name="removable obstacle";   possible_obstacle_[16].value=0;

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
  if (!node_handle_.getParam("send_path_topic", send_path_topic_))
    return false;
  if (!node_handle_.getParam("path_stop_topic", path_stop_topic_))
    return false;
  if (!node_handle_.getParam("path_done_topic", path_done_topic_))
    return false;
  if (!node_handle_.getParam("vision_msg_topic", vision_msg_topic_))
    return false;
  if (!node_handle_.getParam("arm_up/pickup_attempt", pickup_attempt_))
    return false;
  if (!node_handle_.getParam("arm_up/pickup_range", pickup_range_))
    return false;
  if (!node_handle_.getParam("obstacle/position_accurancy", obstacle_position_accurancy_))
    return false;
  if (!node_handle_.getParam("obstacle/obstacle_try", obstacle_try_))
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
  if (!node_handle_.getParam("home/position_x", robot_home_position_.position.x))
    return false;
  if (!node_handle_.getParam("home/position_y", robot_home_position_.position.y))
    return false;
  if (!node_handle_.getParam("home/orientation_z", robot_home_position_.orientation.z))
    return false;
  if (!node_handle_.getParam("home/orientation_w", robot_home_position_.orientation.w))
    return false;
  if (!node_handle_.getParam("init/first_round", round1_))
    return false;
  if (!node_handle_.getParam("init/obstacle_file", obstacle_file_))
    return false;

  return true;
}

void Brain::stateMachine()
{
  ros::spinOnce();
  ROS_ERROR("State: %i, NumObstacles %i, Home %d, Obstcle:%d", state_, ObstacleList_.size(),home_,obstacle_);
  ROS_ERROR("PlanElement: %i, PickUu_ELE: %i, pathdone: %d", planned_element_,picked_up_element_, path_done_);
  switch (state_)
  {
    case 0: Brain::initState();
            break;
    case 1: Brain::explorState();
            break;
    case 2: Brain::findPathState();
            break;
    case 3: Brain::pathExectuionState();
            break;
    case 4: Brain::obstacleState();
            break;
    case 5: Brain::homeState();
            break;

  }

}



} /* namespace */
