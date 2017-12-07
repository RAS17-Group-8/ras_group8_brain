#include <ras_group8_brain/Brain.hpp>

namespace ras_group8_brain {


bool Brain::pathVizualisation(nav_msgs::Path *path)
{
    visualization_msgs::Marker path_points;

    for (int i=0; i<path->poses.size();i++)
    {
        geometry_msgs::Point p;
        p.x = path->poses[i].pose.position.x;
        p.y = path->poses[i].pose.position.y;
        p.z = 0;
        path_points.points.push_back(p);
    }

    path_points.header.frame_id = "/map";
    path_points.header.stamp = ros::Time();
    path_points.ns="map";
    path_points.action=visualization_msgs::Marker::ADD;
    path_points.pose.orientation.w =1.0;
    path_points.id=1;
    path_points.type = visualization_msgs::Marker::POINTS;
    //path_points.type = visualization_msgs::Marker::LINE_STRIP;
    path_points.scale.x = 0.01;
    path_points.scale.y = 0.01;
    path_points.color.b = 1.0;
    path_points.color.a = 1.0;
    marker_publisher_.publish(path_points);

  return true;
}

bool Brain::pointVizualisation(geometry_msgs::Point point)
{

    object_points.points.push_back(point);

    object_points.header.frame_id = "/map";
    object_points.header.stamp = ros::Time();
    object_points.ns="map";
    object_points.action=visualization_msgs::Marker::ADD;
    object_points.pose.orientation.w =1.0;
    object_points.id=2;
    object_points.type = visualization_msgs::Marker::POINTS;
    object_points.scale.x = 0.02;
    object_points.scale.y = 0.02;
    object_points.color.g = 1.0;
    object_points.color.a = 1.0;
    marker_publisher_.publish(object_points);

  return true;
}


} /* namespace */
