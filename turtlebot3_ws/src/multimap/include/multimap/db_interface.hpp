#pragma once

#include <string>
#include <vector>

struct WormholeData
{
  std::string from_room;
  std::string to_room;
  float from_pose_x;
  float from_pose_y;
  float from_yaw;
  float to_pose_x;
  float to_pose_y;
  float to_yaw;
  bool valid = false;  // Set true only when DB data was successfully loaded
};

// Fetches full wormhole data (coordinates + orientation) between two maps
WormholeData get_wormhole_data(const std::string &from_room, const std::string &to_room);

// Returns a list of (from_room, to_room) available in the multimap DB table
std::vector<std::pair<std::string, std::string>> get_all_pairs();
