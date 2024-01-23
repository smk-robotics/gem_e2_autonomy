#pragma once

#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace waypoints_provider {

class WaypointsProvider 
{
public: 
  explicit WaypointsProvider();

private:
  nav_msgs::Path generatePathFromCsvFileWaypoints(const std::string &csv_file_path) const;
  nav_msgs::Path downsamplePath(const nav_msgs::Path &path, double distance) const;
  void publishPath(const nav_msgs::Path &path) const;
  void publishPathMarkers(const nav_msgs::Path &path) const;
  void publishPlanningGoal(const nav_msgs::Path &path) const;

private:
  ros::Publisher goal_publisher_;
  ros::Publisher path_publisher_;
  ros::Publisher path_markers_publisher_;
};

} // waypoints_provider