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
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include "ras_group8_brain/Vision.h"

namespace ras_group8_brain {

class Brain
{
public:
  Brain(ros::NodeHandle& node_handle);
  virtual ~Brain();

  void stateMachine();

private:

  /* Structs
   */
  struct Obstacle
  {
      geometry_msgs::Point position;
      int value;
      int number;
      std::string text;
      bool recovered;
      int try_counter;

  };

  struct ObstacleDefinition
  {
      std::string name;
      int value;
  };


  /* Brain States
   */
  bool  initState();
  bool findPathState();
  bool explorState();
  void pathExectuionState();
  bool  obstacleState();
  int   homeState();

  /* Functions
   */
  bool findElementPath();
  bool findHomePath();
  bool findGoalPath();
  void goalMessageCallback(const geometry_msgs::PoseStamped &msg);

  bool  pickUpArm(geometry_msgs::Point position);
  bool  putDownArm(geometry_msgs::Point position);

  bool readParameters();
  bool Speak(std_msgs::String msg);

  bool ValuableObstacle(struct Obstacle *obstacle_global, ras_group8_brain::Vision *obstacle_msg);
  bool RemovableObstacle(struct Obstacle *obstacle);
  bool SolidObstacle(struct Obstacle *obstacle);
  bool addObstacleToList(struct Obstacle *obstacle, int* list_element);

  void pathDoneCallback(const std_msgs::Bool &msg);
  void visionMessageCallback(const ras_group8_brain::Vision &msg);

  bool pathVizualisation(nav_msgs::Path *path);
  bool pointVizualisation(geometry_msgs::Point point);

  bool readTextfile();
  bool writeTextfile(Obstacle *newObstacle);



  /* ROS Objects
   */
  ros::NodeHandle& node_handle_;

  ros::ServiceClient get_path_client_;
  ros::ServiceClient move_arm_up_client_;
  ros::ServiceClient move_arm_down_client_;

  ros::Publisher speaker_publisher_;
  ros::Publisher send_path_publisher_;
  ros::Publisher marker_publisher_;
  ros::Publisher path_stop_publisher_;

  ros::Subscriber path_done_subscriber_;
  ros::Subscriber vision_msg_subscriber_;
  ros::Subscriber set_goal_subscriber_;

  /* Parameters
   */

  std::string get_path_service_;
  std::string move_arm_up_service_;
  std::string move_arm_down_service_;
  std::string speaker_topic_;
  std::string send_path_topic_;
  std::string path_stop_topic_;
  std::string path_done_topic_;
  std::string vision_msg_topic_;

  std::string obstacle_file_;




  /* Variables
   */
  //State variables
  int state_;
  bool round1_;
  int picked_up_element_;
  int planned_element_;
  bool path_done_;
  bool obstacle_;
  bool home_;

  //PathPlanning
  geometry_msgs::Pose actual_robot_position_;
  geometry_msgs::Pose robot_goal_rviz_;
  geometry_msgs::Pose robot_home_position_;

  nav_msgs::Path  actual_path_;

  std::vector<Obstacle> ObstacleList_;

  Obstacle new_obstacle_global_;
  ras_group8_brain::Vision new_obstacle_msg_;

  //Arm
  int pickup_attempt_;
  int pickup_range_;

  //Obstacle
  double obstacle_position_accurancy_;
  int value_group1_;
  int value_group2_;
  int value_group3_;
  int value_group4_;
  int obstacle_try_;


  ObstacleDefinition possible_obstacle_[17];

  //home
  geometry_msgs::Point home_obstacle_pos_;

  //Vizualistion
  visualization_msgs::Marker object_points;
};

} /* namespace */
