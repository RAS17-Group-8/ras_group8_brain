#include <ras_group8_brain/Brain.hpp>
//#include <iostream>
//#include <fstream>
#include <stdio.h>

namespace ras_group8_brain {


bool Brain::readTextfile()
{
    ROS_INFO("5");
    Brain::Obstacle newObstacle;
    FILE* obstacleFile=fopen(obstacle_file_.c_str(),"r");

    if (obstacleFile == NULL) {
        ROS_ERROR("Failed to open target file");
        return false;
    }

    while(fscanf(obstacleFile,"%lf,%lf,%d\n",&newObstacle.position.x,&newObstacle.position.y,&newObstacle.number)!=EOF)
    {
        ROS_INFO("%f,%f,%d\n",newObstacle.position.x,newObstacle.position.y,newObstacle.number);

        newObstacle.value=possible_obstacle_[newObstacle.number].value;
        newObstacle.text=possible_obstacle_[newObstacle.number].name;
        newObstacle.recovered=false;
        newObstacle.try_counter=0;

        ROS_INFO("vizualisation");
        pointVizualisation(newObstacle.position);
        ObstacleList_.push_back(newObstacle);
    }

    fclose(obstacleFile);
    //Brain::writeTextfile(&newObstacle);
    return true;
}

bool Brain::writeTextfile(Brain::Obstacle *newObstacle)
{
     FILE* obstacleFile=fopen(obstacle_file_.c_str(),"a");
     if (obstacleFile == NULL) {
         ROS_ERROR("Failed to open target file");
         return false;
     }
     fprintf(obstacleFile,"%lf,%lf,%d",newObstacle->position.x,newObstacle->position.y,newObstacle->number);

     fclose(obstacleFile);
}

} /* namespace */
