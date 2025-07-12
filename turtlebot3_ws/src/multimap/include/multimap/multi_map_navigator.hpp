#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include "multimap/action/navigate_to_waypoint.hpp"
#include "multimap/db_interface.hpp"
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace std::chrono_literals;

class MultiMapNavigator : public rclcpp::Node {
public:
  using NavigateToWaypoint = multimap::action::NavigateToWaypoint;
  using GoalHandleNavigateToWaypoint = rclcpp_action::ServerGoalHandle<NavigateToWaypoint>;

  MultiMapNavigator();
  void set_initial_pose(float x, float y, float yaw);
  bool load_map(const std::string &map_name);
  bool navigate_to_pose(float x, float y, float yaw);
  std::vector<std::string> find_path(const std::string& start, const std::string& end);

private:
  rclcpp_action::Server<NavigateToWaypoint>::SharedPtr action_server_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr map_loader_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;

  std::string current_map_;
  std::unordered_map<std::string, std::string> map_paths_;
  bool navigation_in_progress_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const NavigateToWaypoint::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToWaypoint> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToWaypoint> goal_handle);

  void execute(const std::shared_ptr<GoalHandleNavigateToWaypoint> goal_handle);
  
  void dfs(const std::string& current, const std::string& end, 
           const std::unordered_map<std::string, std::vector<std::string>>& graph,
           std::vector<std::string>& path, std::unordered_set<std::string>& visited,
           std::vector<std::string>& result);
};
