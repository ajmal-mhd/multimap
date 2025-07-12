#include "multimap/db_interface.hpp"
#include <pqxx/pqxx>
#include <rclcpp/rclcpp.hpp>

WormholeData get_wormhole_data(const std::string &from_room, const std::string &to_room)
{
  WormholeData data;
  try {
    pqxx::connection conn("dbname=map_db user=postgres password=password host=localhost port=5432");
    pqxx::work txn(conn);

    pqxx::result res = txn.exec_params(
      "SELECT * FROM multimap WHERE from_room = $1 AND to_room = $2",
      from_room, to_room);

    if (!res.empty()) {
      auto row = res[0];
      data.from_room = row["from_room"].as<std::string>();
      data.to_room = row["to_room"].as<std::string>();
      data.from_pose_x = row["from_pose_x"].as<float>();
      data.from_pose_y = row["from_pose_y"].as<float>();
      data.from_yaw = row["from_yaw"].as<float>();
      data.to_pose_x = row["to_pose_x"].as<float>();
      data.to_pose_y = row["to_pose_y"].as<float>();
      data.to_yaw = row["to_yaw"].as<float>();
      data.valid = true;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("db_interface"), "DB Error: %s", e.what());
  }
  return data;
}

std::vector<std::pair<std::string, std::string>> get_all_pairs()
{
  std::vector<std::pair<std::string, std::string>> pairs;
  try {
    pqxx::connection conn("dbname=map_db user=postgres password=password host=localhost port=5432");
    pqxx::work txn(conn);

    pqxx::result res = txn.exec("SELECT from_room, to_room FROM multimap");
    for (auto row : res) {
      pairs.emplace_back(row["from_room"].as<std::string>(), row["to_room"].as<std::string>());
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("db_interface"), "DB Error: %s", e.what());
  }
  return pairs;
}
