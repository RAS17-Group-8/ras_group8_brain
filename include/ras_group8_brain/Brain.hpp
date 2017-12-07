#pragma once

#include <ros/ros.h>
#include <string>
#include <vector>
#include <math.h>

#include <geometry_msgs/Point.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ras_group8_arm_controller/MoveArm.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
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

  struct Edge
  {
      geometry_msgs::Point point;
      bool explored;

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
  bool findEdges();
  void goalMessageCallback(const geometry_msgs::PoseStamped &msg);

  bool pickUpArm(int msg_num, int obs_num);
  bool putDownArm(geometry_msgs::Point position);

  bool readParameters();
  bool Speak(std_msgs::String msg);

  bool ValuableObstacle(struct Obstacle *obstacle_global, int msg_num);
  bool RemovableObstacle(struct Obstacle *obstacle);
  bool SolidObstacle(struct Obstacle *obstacle);
  bool addObstacleToList(struct Obstacle *obstacle, int* list_element);
  bool driveToObstacle(int msg_num);

  void pathDoneCallback(const std_msgs::Bool &msg);
  void robotPositionCallback(const nav_msgs::Odometry &msg);
  void visionMessageCallback(const ras_group8_brain::Vision &msg);
  bool pathVizualisation(nav_msgs::Path *path);
  bool pointVizualisation(geometry_msgs::Point point);

  void newMapCallback(const std_msgs::Bool &msg);

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
  ros::Publisher add_object_publisher_;

  ros::Subscriber path_done_subscriber_;
  ros::Subscriber vision_msg_subscriber_;
  ros::Subscriber set_goal_subscriber_;
  ros::Subscriber robot_position_subscriber_;
  ros::Subscriber new_map_subscriber_;

  tf::TransformListener tf_listener_;

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
  std::string robot_position_topic_;
  std::string new_map_topic_;
  std::string add_object_map_topic_;

  std::string obstacle_file_;

  ros::Time run_time_;


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
  bool go_home_;

  //PathPlanning
  geometry_msgs::Pose actual_robot_position_;
  geometry_msgs::Pose robot_goal_rviz_;
  geometry_msgs::Pose robot_home_position_;
  Edge edges_[3];
  int planned_edge_;
  double maze_size_x_;
  double maze_size_y_;
  bool recieved_pose_;

  nav_msgs::Path  actual_path_;

  std::vector<Obstacle> ObstacleList_;

  Obstacle new_obstacle_global_[3];
  ras_group8_brain::Vision new_obstacle_msg_;

  //Arm
  int pickup_attempt_;
  int pickup_range_;

  //Obstacle
  double obstacle_position_accurancy_;
  double obstacle_detection_accurancy_;
  int value_group1_;
  int value_group2_;
  int value_group3_;
  int value_group4_;
  int obstacle_try_;
  ObstacleDefinition possible_obstacle_[18];

  //Path Exectuion
  double round_time_;

  //home
  geometry_msgs::Point home_obstacle_pos_;
  double home_accurancy_;

  //Vizualistion
  visualization_msgs::Marker object_points;
};

} /* namespace */
