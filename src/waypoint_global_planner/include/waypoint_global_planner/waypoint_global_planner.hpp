#pragma once

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

namespace waypoint_global_planner
{

class WaypointGlobalPlanner : public nav_core::BaseGlobalPlanner
{
public:
  WaypointGlobalPlanner();
  WaypointGlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
  void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped &start_pose, const geometry_msgs::PoseStamped &goal,
    std::vector<geometry_msgs::PoseStamped> &plan);
  void waypointCallback(const geometry_msgs::PointStamped::ConstPtr &waypoint);
  void externalPathCallback(const nav_msgs::PathConstPtr &plan);
  void createAndPublishMarkersFromPath(const std::vector<geometry_msgs::PoseStamped> &path);
  void interpolatePath(nav_msgs::Path &path);

private: 
  nav_msgs::Path getPathFromCsvFile(const std::string &csv_file_path) const;
  void PublishPathMarkers(const nav_msgs::Path &path) const noexcept;
  void PublishPlanningGoal(const nav_msgs::Path &path) const noexcept;
  nav_msgs::Path downsamplePath(const nav_msgs::Path &path, double threshold) const;

private:
  ros::Subscriber waypoint_sub_;
  ros::Subscriber external_path_sub_;
  ros::Publisher waypoint_marker_pub_;
  ros::Publisher goal_pub_;
  ros::Publisher plan_pub_;
  double epsilon_;
  int waypoints_per_meter_;
  std::vector<geometry_msgs::PoseStamped> waypoints_;
  nav_msgs::Path path_;
  bool clear_waypoints_;
  std::string waypoints_file_;
  bool initialized_ = false;
};

}  // namespace waypoint_global_planner