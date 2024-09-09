/*********************************************************************
 * Copyright (c) 2024, SoftBank Corp.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************/

#ifndef RESOURCE_MONITOR_HPP
#define RESOURCE_MONITOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_obstacle_msgs/msg/obstacles.hpp>
#include <std_msgs/msg/string.hpp>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <cmath>
#include <vector>
#include <string>
#include <memory>

class ResourceMonitor : public rclcpp::Node
{
public:
  // Constructor
  ResourceMonitor();

private:
  // Values set by rosparam
  std::string robot_id_;
  std::string building_id_;
  std::string server_url_;
  std::string resource_config_file_;
  float resource_registration_distance_;  // Register to resources within this distance.
  float resource_release_distance_;       // Release registered resources at a distance away from this value.
  // Values changed by topic
  std::string current_floor_id_;
  // Structure to represent a single resource  // TODO: Consider a cuboid area using the center position?
  struct Resource
  {
    std::string resource_id_;
    std::string floor_id_;
    float coord_x_;
    float coord_y_;
    bool registration_state_;
  };

  void loadResourcesFromYaml(const std::string& yaml_file);
  double calculateDistance(const geometry_msgs::msg::Pose& position1, const float coord_x, const float coord_y);
  nlohmann::json accessResourceServer(const Resource& resource, const std::string& api_endpoint);
  void checkAndAccessResources();

  void timerCallback();
  void fleetCallback(const std::shared_ptr<const rmf_fleet_msgs::msg::FleetState>& msg);
  void publishObstacle(const float x, const float y);

  // スケジュールトピックのコールバック関数
  // void schedule_callback(const std_msgs::msg::String::SharedPtr msg);

  // スケジュールデータからリソースを抽出する関数
  // std::vector<Resource> extract_resources_from_schedule(const std::string &schedule_data);

  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr fleet_subscription_;
  rclcpp::Publisher<rmf_obstacle_msgs::msg::Obstacles>::SharedPtr obstacle_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;  // Timer for periodic resource checks

  bool first_fleet_message_received_;          // Flag to show that the first fleet message has been received
  geometry_msgs::msg::Pose current_position_;  // The target robot's current position
  std::vector<Resource> route_resources_;      // List of resources managed by the server, read from yaml config file
  std::string registered_resource_;  // The resource that the robot is currently registered to (currently only one)
};

#endif  // RESOURCE_MONITOR_HPP
