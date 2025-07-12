#include "multimap/multi_map_navigator.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <chrono>
#include <thread>
#include <unordered_set>
#include <filesystem>
#include <cstdlib>

using namespace std::chrono_literals;

MultiMapNavigator::MultiMapNavigator()
  : Node("multi_map_navigator"), current_map_("room1"), navigation_in_progress_(false)
{
  RCLCPP_INFO(this->get_logger(), "MultiMapNavigator starting up...");

  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

  nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
  map_loader_client_ = this->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");

  action_server_ = rclcpp_action::create_server<NavigateToWaypoint>(
    this,
    "navigate_to_waypoint",
    std::bind(&MultiMapNavigator::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MultiMapNavigator::handle_cancel, this, std::placeholders::_1),
    std::bind(&MultiMapNavigator::handle_accepted, this, std::placeholders::_1)
  );

  // Get current working directory and construct map paths
  std::string pwd = std::filesystem::current_path().string();
  RCLCPP_INFO(this->get_logger(), "Current working directory: %s", pwd.c_str());
  
  map_paths_["room1"] = pwd + "/maps/room1.yaml";
  map_paths_["room2"] = pwd + "/maps/room2.yaml";
  map_paths_["room3"] = pwd + "/maps/room3.yaml";
  map_paths_["room4"] = pwd + "/maps/room4.yaml";

  // Wait for navigation server to be available
  RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server...");
  if (!nav_client_->wait_for_action_server(10s)) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available after 10 seconds");
  } else {
    RCLCPP_INFO(this->get_logger(), "Nav2 action server connected");
  }

  std::this_thread::sleep_for(2s);
  set_initial_pose(-0.3, -3.3, -1.57);
  
  RCLCPP_INFO(this->get_logger(), "MultiMapNavigator initialized successfully");
}

rclcpp_action::GoalResponse MultiMapNavigator::handle_goal(
  const rclcpp_action::GoalUUID &uuid,
  std::shared_ptr<const NavigateToWaypoint::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal to map: %s", goal->target_map.c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MultiMapNavigator::handle_cancel(
  const std::shared_ptr<GoalHandleNavigateToWaypoint> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MultiMapNavigator::handle_accepted(const std::shared_ptr<GoalHandleNavigateToWaypoint> goal_handle)
{
  std::thread{std::bind(&MultiMapNavigator::execute, this, goal_handle)}.detach();
}

void MultiMapNavigator::execute(const std::shared_ptr<GoalHandleNavigateToWaypoint> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<NavigateToWaypoint::Result>();
  auto feedback = std::make_shared<NavigateToWaypoint::Feedback>();

  RCLCPP_INFO(this->get_logger(), "Received navigation request to map: %s at waypoint: [%.2f, %.2f, %.2f]", 
              goal->target_map.c_str(), goal->waypoint[0], goal->waypoint[1], goal->waypoint[2]);

  // Check if target map exists
  if (map_paths_.find(goal->target_map) == map_paths_.end()) {
    RCLCPP_ERROR(this->get_logger(), "Target map %s not found", goal->target_map.c_str());
    result->success = false;
    result->message = "Target map " + goal->target_map + " not found";
    goal_handle->abort(result);
    return;
  }

  // If already in target map, navigate directly to waypoint
  if (goal->target_map == current_map_) {
    RCLCPP_INFO(this->get_logger(), "Already in target map %s, navigating to waypoint", goal->target_map.c_str());
    
    feedback->current_status = "Navigating to waypoint in current map";
    goal_handle->publish_feedback(feedback);
    
    if (!navigate_to_pose(goal->waypoint[0], goal->waypoint[1], goal->waypoint[2])) {
      result->success = false;
      result->message = "Navigation to waypoint failed";
      goal_handle->abort(result);
      return;
    }
    
    result->success = true;
    result->message = "Successfully navigated to waypoint [" + std::to_string(goal->waypoint[0]) + 
                     ", " + std::to_string(goal->waypoint[1]) + ", " + std::to_string(goal->waypoint[2]) + 
                     "] in map " + goal->target_map;
    goal_handle->succeed(result);
    return;
  }

  // Multi-map navigation needed
  RCLCPP_INFO(this->get_logger(), "Planning path from %s to %s", current_map_.c_str(), goal->target_map.c_str());
  
  feedback->current_status = "Planning path through maps";
  goal_handle->publish_feedback(feedback);

  // Get path through maps
  auto map_path = find_path(current_map_, goal->target_map);
  if (map_path.empty() || map_path.size() < 2) {
    RCLCPP_ERROR(this->get_logger(), "No valid path found from %s to %s", current_map_.c_str(), goal->target_map.c_str());
    result->success = false;
    result->message = "No valid path found from " + current_map_ + " to " + goal->target_map;
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Path through maps found with %zu steps", map_path.size());

  // Navigate through each map transition
  for (size_t i = 0; i < map_path.size() - 1; ++i) {
    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Goal was cancelled";
      goal_handle->canceled(result);
      return;
    }

    const std::string& from_map = map_path[i];
    const std::string& to_map = map_path[i + 1];
    
    RCLCPP_INFO(this->get_logger(), "Transitioning from %s to %s", from_map.c_str(), to_map.c_str());
    
    feedback->current_status = "Transitioning from " + from_map + " to " + to_map;
    goal_handle->publish_feedback(feedback);

    // Get wormhole data for transition
    WormholeData wormhole_data = get_wormhole_data(from_map, to_map);
    if (!wormhole_data.valid) {
      RCLCPP_ERROR(this->get_logger(), "No wormhole data found for transition from %s to %s", from_map.c_str(), to_map.c_str());
      result->success = false;
      result->message = "No wormhole data found for transition from " + from_map + " to " + to_map;
      goal_handle->abort(result);
      return;
    }

    // Navigate to wormhole point in current map
    RCLCPP_INFO(this->get_logger(), "Navigating to wormhole point in %s", from_map.c_str());
    feedback->current_status = "Navigating to wormhole in " + from_map;
    goal_handle->publish_feedback(feedback);
    
    if (!navigate_to_pose(wormhole_data.from_pose_x, wormhole_data.from_pose_y, wormhole_data.from_yaw)) {
      result->success = false;
      result->message = "Failed to navigate to wormhole point in " + from_map;
      goal_handle->abort(result);
      return;
    }

    // Load the next map
    RCLCPP_INFO(this->get_logger(), "Loading next map: %s", to_map.c_str());
    feedback->current_status = "Loading map " + to_map;
    goal_handle->publish_feedback(feedback);
    
    if (!load_map(to_map)) {
      result->success = false;
      result->message = "Failed to load map " + to_map;
      goal_handle->abort(result);
      return;
    }

    // Set robot's position in new map
    RCLCPP_INFO(this->get_logger(), "Setting initial pose in new map %s", to_map.c_str());
    set_initial_pose(wormhole_data.to_pose_x, wormhole_data.to_pose_y, wormhole_data.to_yaw);
    
    // Update current map
    current_map_ = to_map;
    
    // Allow time for proper localization and map server stabilization
    std::this_thread::sleep_for(3s);

    // Verify navigation server is ready
    if (!nav_client_->wait_for_action_server(5s)) {
      RCLCPP_WARN(this->get_logger(), "Navigation server not ready after map transition");
    }
  }

  // Now in target map, navigate to final waypoint
  RCLCPP_INFO(this->get_logger(), "Arrived in target map %s, navigating to waypoint [%.2f, %.2f, %.2f]", 
              goal->target_map.c_str(), goal->waypoint[0], goal->waypoint[1], goal->waypoint[2]);
  
  feedback->current_status = "Navigating to final waypoint";
  goal_handle->publish_feedback(feedback);

  if (!navigate_to_pose(goal->waypoint[0], goal->waypoint[1], goal->waypoint[2])) {
    result->success = false;
    result->message = "Navigation to waypoint failed";
    goal_handle->abort(result);
    return;
  }

  result->success = true;
  result->message = "Successfully navigated to waypoint [" + std::to_string(goal->waypoint[0]) + 
                   ", " + std::to_string(goal->waypoint[1]) + ", " + std::to_string(goal->waypoint[2]) + 
                   "] in map " + goal->target_map;
  goal_handle->succeed(result);
}

bool MultiMapNavigator::load_map(const std::string &map_name)
{
  if (map_paths_.find(map_name) == map_paths_.end()) {
    RCLCPP_ERROR(this->get_logger(), "Map %s not found in available maps", map_name.c_str());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Loading map: %s from %s", map_name.c_str(), map_paths_[map_name].c_str());

  if (!map_loader_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Map loader service not available");
    return false;
  }

  auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  request->map_url = map_paths_[map_name];

  try {
    auto future = map_loader_client_->async_send_request(request);
    
    // Wait for service response using manual polling
    auto timeout = std::chrono::steady_clock::now() + 10s;
    while (std::chrono::steady_clock::now() < timeout) {
      if (future.wait_for(100ms) == std::future_status::ready) {
        break;
      }
    }

    if (future.wait_for(0ms) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "Service call timed out for loading map: %s", map_name.c_str());
      return false;
    }

    auto response = future.get();
    if (response->result == nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Successfully loaded map: %s", map_name.c_str());
      
      // Increased delay for map loading stabilization
      std::this_thread::sleep_for(3s);
      
      // Reconnect navigation client after map change
      RCLCPP_INFO(this->get_logger(), "Reconnecting to navigation server...");
      if (!nav_client_->wait_for_action_server(10s)) {
        RCLCPP_WARN(this->get_logger(), "Navigation server not responding after map change");
      }
      
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to load map: %s, result: %d", map_name.c_str(), response->result);
      return false;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during map loading: %s", e.what());
    return false;
  }
}

void MultiMapNavigator::set_initial_pose(float x, float y, float yaw)
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = this->now();
  pose.pose.pose.position.x = x;
  pose.pose.pose.position.y = y;
  pose.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  pose.pose.pose.orientation = tf2::toMsg(q);

  // Set covariance matrix for pose uncertainty matching Python implementation
  pose.pose.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0,          // x variance
                          0.0, 0.25, 0.0, 0.0, 0.0, 0.0,          // y variance  
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,           // z variance
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,           // roll variance
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,           // pitch variance
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.068539};     // yaw variance

  RCLCPP_INFO(this->get_logger(), 
              "Setting robot initial pose in %s: x=%.2f, y=%.2f, yaw=%.2f rad", 
              current_map_.c_str(), x, y, yaw);

  // Publish multiple times for reliability
  for (int i = 0; i < 3; ++i) {
    pose_publisher_->publish(pose);
    std::this_thread::sleep_for(500ms);
  }

  RCLCPP_INFO(this->get_logger(), "Initial pose published");
}

bool MultiMapNavigator::navigate_to_pose(float x, float y, float yaw)
{
  RCLCPP_INFO(this->get_logger(), "Navigating to pose: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);

  // Ensure action client is connected
  if (!nav_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available");
    return false;
  }

  // Create goal pose
  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = this->now();
  
  // Set position
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;
  goal_msg.pose.pose.position.z = 0.0;

  // Set orientation (from yaw)
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  RCLCPP_INFO(this->get_logger(), "Sending navigation goal...");

  // Send goal and wait for acceptance using manual polling
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  auto future_goal = nav_client_->async_send_goal(goal_msg, send_goal_options);

  // Wait for goal to be sent with manual polling
  auto timeout = std::chrono::steady_clock::now() + 10s;
  while (std::chrono::steady_clock::now() < timeout) {
    if (future_goal.wait_for(100ms) == std::future_status::ready) {
      break;
    }
  }

  if (future_goal.wait_for(0ms) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Goal send timed out");
    return false;
  }

  auto goal_handle = future_goal.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Navigation goal rejected");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Navigation goal accepted, waiting for result...");

  // Wait for result using manual polling
  auto result_future = nav_client_->async_get_result(goal_handle);
  timeout = std::chrono::steady_clock::now() + 200s;
  while (std::chrono::steady_clock::now() < timeout) {
    if (result_future.wait_for(100ms) == std::future_status::ready) {
      break;
    }
  }

  if (result_future.wait_for(0ms) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Navigation goal timed out");
    return false;
  }

  auto result = result_future.get();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(), "Navigation completed successfully");
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Navigation failed with status %d", static_cast<int>(result.code));
    return false;
  }
}

std::vector<std::string> MultiMapNavigator::find_path(const std::string& start, const std::string& end)
{
  auto wormholes = get_all_pairs();
  
  // Build adjacency list
  std::unordered_map<std::string, std::vector<std::string>> graph;
  for (const auto& pair : wormholes) {
    graph[pair.first].push_back(pair.second);
  }
  
  // DFS to find a path
  std::vector<std::string> path = {start};
  std::unordered_set<std::string> visited;
  std::vector<std::string> result;
  
  dfs(start, end, graph, path, visited, result);
  
  return result;
}

void MultiMapNavigator::dfs(const std::string& current, const std::string& end,
                           const std::unordered_map<std::string, std::vector<std::string>>& graph,
                           std::vector<std::string>& path, std::unordered_set<std::string>& visited,
                           std::vector<std::string>& result)
{
  if (current == end) {
    result = path;
    return;
  }
  
  visited.insert(current);
  
  auto it = graph.find(current);
  if (it != graph.end()) {
    for (const std::string& neighbor : it->second) {
      if (visited.find(neighbor) == visited.end() && result.empty()) {
        path.push_back(neighbor);
        dfs(neighbor, end, graph, path, visited, result);
        if (result.empty()) {
          path.pop_back();
        }
      }
    }
  }
  
  visited.erase(current);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiMapNavigator>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
