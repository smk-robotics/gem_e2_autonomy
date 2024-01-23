#include "waypoints_provider/waypoints_provider.hpp"
#include <fstream>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>

namespace waypoints_provider
{

WaypointsProvider::WaypointsProvider()
{
  ros::NodeHandle pnh("~");
  std::string waypoints_file;
  pnh.getParam("waypoints_file", waypoints_file);
  goal_publisher_         = pnh.advertise<geometry_msgs::PoseStamped>("output/planning_goal", 10);
  path_publisher_         = pnh.advertise<nav_msgs::Path>("output/path", 10);
  path_markers_publisher_ = pnh.advertise<visualization_msgs::Marker>("output/path_markers", 10);
  const auto waypoints_path = generatePathFromCsvFileWaypoints(waypoints_file);
  const auto downsampled_path = downsamplePath(waypoints_path, 1.0);
  publishPlanningGoal(downsampled_path);
  ros::Duration(3.0).sleep();
  publishPath(downsampled_path);
  publishPathMarkers(downsampled_path);
}

nav_msgs::Path WaypointsProvider::generatePathFromCsvFileWaypoints(const std::string &csv_file_path) const
{
  if (csv_file_path.empty())
  {
    throw std::invalid_argument("[WaypointGlobalPlanner]: Given path to the csv file is empty!");
  }
  
  std::ifstream file(csv_file_path);
  
  if (!file.is_open())
  {
    throw("[WaypointGlobalPlanner]: Failed to open CSV file: %s", csv_file_path);
  }

  nav_msgs::Path path;
  path.header.stamp = ros::Time().now();
  path.header.frame_id = "map";
  std::string line;
  
  while (std::getline(file, line))
  {
    std::istringstream iss(line);
    std::string token;
    auto x = 0.0;
    auto y = 0.0;
    auto yaw = 0.0;
    if (std::getline(iss, token, ','))
    {
      x = std::stod(token);
    }
    else
    {
      ROS_ERROR(
          "[WaypointGlobalPlanner]: Invalid CSV line. Can't get x coordinate from line: %s", line.c_str());
      continue;
    }
    if (std::getline(iss, token, ','))
    {
      y = std::stod(token);
    }
    else
    {
      ROS_ERROR(
          "[WaypointGlobalPlanner]: Invalid CSV line. Can't get y coordinate from line: %s", line.c_str());
      continue;
    }
    if (std::getline(iss, token))
    {
      yaw = std::stod(token);
    }
    else
    {
      ROS_ERROR(
          "[WaypointGlobalPlanner]: Invalid CSV line. Can't get yaw angle from line: %s", line.c_str());
      continue;
    }
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    path.poses.push_back(pose);
  }
  
  file.close();
  return path;
}

nav_msgs::Path WaypointsProvider::downsamplePath(const nav_msgs::Path &path, double threshold) const
{
  nav_msgs::Path downsampled_path;
  downsampled_path.header = path.header;
  downsampled_path.poses.emplace_back(path.poses.front());
  for (const auto &pose : path.poses)
  {
    const auto last_downsampled_pose = downsampled_path.poses.back();
    const auto dx = pose.pose.position.x - last_downsampled_pose.pose.position.x;
    const auto dy = pose.pose.position.y - last_downsampled_pose.pose.position.y;
    const auto distance = std::hypot(dx, dy);
    if (distance > threshold)
    {
      downsampled_path.poses.emplace_back(pose);
    }
  }
  return downsampled_path;
}

void WaypointsProvider::publishPathMarkers(const nav_msgs::Path &path) const
{
  visualization_msgs::Marker path_markers;
  path_markers.header = path.header;
  path_markers.type = visualization_msgs::Marker::SPHERE_LIST;
  path_markers.action = visualization_msgs::Marker::ADD;
  path_markers.pose.position.x = 0.0;
  path_markers.pose.position.y = 0.0;
  path_markers.pose.position.z = 0.0;
  path_markers.pose.orientation.x = 0.0;
  path_markers.pose.orientation.y = 0.0;
  path_markers.pose.orientation.z = 0.0;
  path_markers.pose.orientation.w = 1.0;
  path_markers.scale.x = 0.25;
  path_markers.scale.y = 0.25;
  path_markers.scale.z = 0.25;
  path_markers.color.a = 1.0;
  path_markers.color.r = 0.0;
  path_markers.color.g = 1.0;
  path_markers.color.b = 0.0;
  for (const auto &pose : path.poses)
  {
    path_markers.points.push_back(pose.pose.position);
  }
  path_markers_publisher_.publish(path_markers);
}

void WaypointsProvider::publishPath(const nav_msgs::Path &path_msg) const
{
  ros::Rate loop_rate(10);
  for (auto i = 0; i < 2; ++i)
  {
    ROS_INFO("[WaypointsProvider]: Path was published.");
    path_publisher_.publish(path_msg);
    loop_rate.sleep();
  }
}

void WaypointsProvider::publishPlanningGoal(const nav_msgs::Path &path_msg) const
{
  auto goal = path_msg.poses.back();
  goal.header.stamp = ros::Time(0);
  ros::Rate loop_rate(10);
  for (auto i = 0; i < 5; ++i)
  {
    goal_publisher_.publish(goal);
    ROS_INFO("[WaypointsProvider]: Planning goal was published.");
    loop_rate.sleep();
  }
}

} // namespace waypoints_provider

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoints_provider_node");
  auto node = waypoints_provider::WaypointsProvider();
  ros::spin();
  return 0;
}