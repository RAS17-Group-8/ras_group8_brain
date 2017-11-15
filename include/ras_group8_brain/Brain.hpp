#pragma once

#include <ros/ros.h>
#include <string>
#include <vector>
#include <math.h>

#include <geometry_msgs/Point.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <ras_group8_arm_controller/MoveArm.h>
#include <std_msgs/String.h>

namespace ras_group8_brain {

class Brain
{
public:
  Brain(ros::NodeHandle& node_handle);
  virtual ~Brain();

private:

  /* Structs
   */
  struct Obstacle
  {
      geometry_msgs::Point position;
      int value;
      std::string shape;
      std::string color;
      bool recovered;

  };


  /* Brain States
   */
  int  initState();
  bool findPathState();
  bool explorState();
  bool pathExectuionState();
  bool  obstacleState();
  int   homeState();

  /* Functions
   */
  bool findElementPath();
  bool findHomePath();

  bool  pickUpArm(geometry_msgs::Point position);
  bool  putDownArm(geometry_msgs::Point position);

  bool readParameters();
  bool Speak(std_msgs::String msg);

  bool ValuableObstacle(struct Obstacle *obstacle);
  bool RemovableObstacle(struct Obstacle *obstacle);
  bool SolidObstacle(struct Obstacle *obstacle);

  bool addObstacleToList(struct Obstacle *obstacle, int* list_element);
  bool defineObstacleValue(struct Obstacle *obstacle);



  /* ROS Objects
   */
  ros::NodeHandle& node_handle_;
  //ros::Subscriber subscriber_; //Löschen

  ros::ServiceClient get_path_client_;
  ros::ServiceClient move_arm_up_client_;
  ros::ServiceClient move_arm_down_client_;
  ros::Publisher speaker_publisher_;

  /* Parameters
   */
  //std::string subscriber_topic_; //Löschen

  std::string get_path_service_;
  std::string move_arm_up_service_;
  std::string move_arm_down_service_;
  std::string speaker_topic_;




  /* Variables
   */

  //PathPlanning
  int planned_element;
  geometry_msgs::Point actual_robot_position_;
  geometry_msgs::Point robot_home_position_;

  nav_msgs::Path  actual_path_;

  std::vector<Obstacle> ObstacleList_;

  Obstacle actual_obstacle_;

  //Arm
  int pickup_attempt_;
  int pickup_range_;

  //Obstacle
  double obstacle_position_accurancy_;
  int value_group1_;
  int value_group2_;
  int value_group3_;
  int value_group4_;

  int picked_up_element;

  //home
  geometry_msgs::Point home_obstacle_pos_;




};

} /* namespace */
