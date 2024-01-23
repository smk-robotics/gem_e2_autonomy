#include "waypoint_global_planner/waypoint_global_planner.hpp"
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>

PLUGINLIB_EXPORT_CLASS(waypoint_global_planner::WaypointGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace waypoint_global_planner
{

WaypointGlobalPlanner::WaypointGlobalPlanner() 
  : costmap_ros_(NULL), initialized_(false) 
{}

WaypointGlobalPlanner::WaypointGlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  : costmap_ros_(NULL),  initialized_(false)
{
  initialize(name, costmap_ros);
}

void WaypointGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
  if (!initialized_) {
    costmap_ros_ = costmap_ros;
    costmap_     = costmap_ros_->getCostmap();
    world_model_ = new base_local_planner::CostmapModel(*costmap_);

    ros::NodeHandle pnh("~" + name);

    pnh.param("epsilon", epsilon_, 1e-1);
    pnh.param("waypoints_per_meter", waypoints_per_meter_, 20);

    ros::Rate loop_rate(10);
    while (!pnh.getParam("waypoints_file", waypoints_file_)) {
      ROS_INFO_STREAM("[WaypointGlobalPlanner]: Waiting for waypoints_file param...");
      loop_rate.sleep();
    }

    waypoint_sub_              = pnh.subscribe("/clicked_point", 100, &WaypointGlobalPlanner::waypointCallback, this);
    external_path_sub_         = pnh.subscribe("external_path", 1, &WaypointGlobalPlanner::externalPathCallback, this);
    waypoint_marker_pub_       = pnh.advertise<visualization_msgs::MarkerArray>("waypoints", 1);
    goal_pub_                  = pnh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    plan_pub_                  = pnh.advertise<nav_msgs::Path>("global_plan", 1);
  
    try {
      const auto waypoints_path_ = getPathFromCsvFile(waypoints_file_);
      PublishPathMarkers(waypoints_path_);
      PublishPlanningGoal(waypoints_path_);
    } catch (std::exception &ex) {
      ROS_ERROR("[WaypointGlobalPlanner]: %s", ex.what());
      throw;
    }

    initialized_ = true;
  } else {
    ROS_WARN("This planner has already been initialized... doing nothing");
  }
}

nav_msgs::Path WaypointGlobalPlanner::getPathFromCsvFile(const std::string &csv_file_path) const
{
  if (csv_file_path.empty()) {
    throw std::invalid_argument("[WaypointGlobalPlanner]: Given path to the csv file is empty!");
  }

  std::ifstream file(csv_file_path);

  if (!file.is_open()) {
    throw("[WaypointGlobalPlanner]: Failed to open CSV file: %s", csv_file_path);
  }

  nav_msgs::Path path;
  path.header.stamp    = ros::Time().now();
  path.header.frame_id = "map";
  std::string line;
  
  while (std::getline(file, line)) {
      std::istringstream iss(line);
      std::string token;

      auto x   = 0.0;
      auto y   = 0.0; 
      auto yaw = 0.0;

      if (std::getline(iss, token, ',')) {
        x = std::stod(token);
      } else {
        ROS_ERROR(
          "[WaypointGlobalPlanner]: Invalid CSV line. Can't get x coordinate from line: %s", line.c_str());
        continue;
      }

      if (std::getline(iss, token, ',')) {
        y = std::stod(token);
      } else {
          ROS_ERROR(
            "[WaypointGlobalPlanner]: Invalid CSV line. Can't get y coordinate from line: %s", line.c_str());
        continue;
      }

      if (std::getline(iss, token)) {
        yaw = std::stod(token);
      } else {
          ROS_ERROR(
            "[WaypointGlobalPlanner]: Invalid CSV line. Can't get yaw angle from line: %s", line.c_str());
        continue;
      }

      geometry_msgs::PoseStamped pose;
      pose.pose.position.x  = x;
      pose.pose.position.y  = y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      path.poses.push_back(pose);
  }

  file.close();
  return path;
}

void WaypointGlobalPlanner::PublishPathMarkers(const nav_msgs::Path &path) const noexcept 
{
  // clear previous markers
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;
  marker.header  = path.header;
  marker.ns      = "/move_base/waypoint_global_planner";
  marker.type    = visualization_msgs::Marker::SPHERE;
  marker.action  = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;

  for (size_t i = 0; i < path.poses.size(); i++) {
    marker.id   = i;
    marker.pose = path.poses[i].pose;
    markers.markers.push_back(marker);
  }

  waypoint_marker_pub_.publish(markers);
}

void WaypointGlobalPlanner::PublishPlanningGoal(const nav_msgs::Path &path) const noexcept 
{
  geometry_msgs::PoseStamped goal;
  goal.header.stamp    = path.header.stamp;
  goal.header.frame_id = "map";
  goal.pose            = path.poses.back().pose;
  goal_pub_.publish(goal);
}

bool WaypointGlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start_pose, 
  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
  const auto waypoints_path_ = getPathFromCsvFile(waypoints_file_);
  PublishPathMarkers(waypoints_path_);
  PublishPlanningGoal(waypoints_path_);
  ROS_INFO("Published global plan");
  
  plan.clear();

  for (const auto &pose : waypoints_path_.poses) {
    plan.emplace_back(pose);
  }

  plan_pub_.publish(waypoints_path_);
  return true;
}


void WaypointGlobalPlanner::waypointCallback(const geometry_msgs::PointStamped::ConstPtr& waypoint)
{
  if (clear_waypoints_) {
    waypoints_.clear();
    clear_waypoints_ = false;
  }

  // add waypoint to the waypoint vector
  waypoints_.push_back(geometry_msgs::PoseStamped());
  waypoints_.back().header = waypoint->header;
  waypoints_.back().pose.position = waypoint->point;
  waypoints_.back().pose.orientation.w = 1.0;

  // create and publish markers
  createAndPublishMarkersFromPath(waypoints_);

  if (waypoints_.size() < 2)
    return;

  geometry_msgs::Pose *p1 = &(waypoints_.end()-2)->pose;
  geometry_msgs::Pose *p2 = &(waypoints_.end()-1)->pose;

  // calculate orientation of waypoints
  double yaw = atan2(p2->position.y - p1->position.y, p2->position.x - p1->position.x);
  p1->orientation = tf::createQuaternionMsgFromYaw(yaw);

  // calculate distance between latest two waypoints and check if it surpasses the threshold epsilon
  double dist = hypot(p1->position.x - p2->position.x, p1->position.y - p2->position.y);
  if (dist < epsilon_)
  {
    p2->orientation = p1->orientation;
    path_.header = waypoint->header;
    path_.poses.clear();
    path_.poses.insert(path_.poses.end(), waypoints_.begin(), waypoints_.end());
    goal_pub_.publish(waypoints_.back());
    clear_waypoints_ = true;
    ROS_INFO("Published goal pose");
  }
}

void WaypointGlobalPlanner::interpolatePath(nav_msgs::Path& path)
{
  std::vector<geometry_msgs::PoseStamped> temp_path;
  for (int i = 0; i < static_cast<int>(path.poses.size()-1); i++)
  {
    // calculate distance between two consecutive waypoints
    double x1 = path.poses[i].pose.position.x;
    double y1 = path.poses[i].pose.position.y;
    double x2 = path.poses[i+1].pose.position.x;
    double y2 = path.poses[i+1].pose.position.y;
    double dist =  hypot(x1-x2, y1-y2);
    int num_wpts = dist * waypoints_per_meter_;

    temp_path.push_back(path.poses[i]);
    geometry_msgs::PoseStamped p = path.poses[i];
    for (int j = 0; j < num_wpts - 2; j++)
    {
      p.pose.position.x = x1 + static_cast<double>(j) / num_wpts * (x2 - x1);
      p.pose.position.y = y1 + static_cast<double>(j) / num_wpts * (y2 - y1);
      temp_path.push_back(p);
    }
  }

  // update sequence of poses
  for (size_t i = 0; i < temp_path.size(); i++)
    temp_path[i].header.seq = static_cast<int>(i);

  temp_path.push_back(path.poses.back());
  path.poses = temp_path;
}

void WaypointGlobalPlanner::externalPathCallback(const nav_msgs::PathConstPtr& plan)
{
  path_.poses.clear();
  clear_waypoints_ = true;
  path_.header = plan->header;
  path_.poses = plan->poses;
  createAndPublishMarkersFromPath(path_.poses);
  goal_pub_.publish(path_.poses.back());
}


void WaypointGlobalPlanner::createAndPublishMarkersFromPath(const std::vector<geometry_msgs::PoseStamped>& path)
{
  // clear previous markers
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;
  marker.header = path[0].header;
  marker.ns = "/move_base/waypoint_global_planner";
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.id = 0;
  markers.markers.push_back(marker);
  waypoint_marker_pub_.publish(markers);
  marker.action = visualization_msgs::Marker::ADD;
  markers.markers.clear();

  for (size_t i = 0; i < path.size(); i++)
  {
    marker.id = i;
    marker.pose.position = path[i].pose.position;
    markers.markers.push_back(marker);
  }

  waypoint_marker_pub_.publish(markers);
}

}  // namespace waypoint_global_planner