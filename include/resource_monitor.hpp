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
  bool block_on_failure_;                 // Publish obstacle info when failed to access the server
  // Values changed by topic
  std::string current_floor_id_;
  // Structure to represent a single resource
  struct Resource
  {
    std::string resource_id_;
    std::string floor_id_;
    float center_x_;
    float center_y_;
    float size_x_;
    float size_y_;
    float size_z_;
    float registration_distance_;
    float release_distance_;
    bool registration_state_;
  };

  // Helper functions
  void loadResourcesFromYaml(const std::string& yaml_file);
  double calculateDistance(const geometry_msgs::msg::Pose& position1, const float coord_x, const float coord_y);
  void registerResource(Resource& resource);
  void releaseResource(Resource& resource);
  std::string createRegistrationJson(const Resource& resource);
  std::string createReleaseJson(const Resource& resource);
  nlohmann::json accessResourceServer(const std::string& api_endpoint, const std::string& json_data);
  void checkAndAccessResources();

  void timerCallback();
  void fleetCallback(const std::shared_ptr<const rmf_fleet_msgs::msg::FleetState>& msg);
  void publishObstacle(const Resource& resource);

  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr fleet_subscription_;
  rclcpp::Publisher<rmf_obstacle_msgs::msg::Obstacles>::SharedPtr obstacle_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;  // Timer for periodic resource checks

  bool first_fleet_message_received_;          // Flag to show that the first fleet message has been received
  geometry_msgs::msg::Pose current_position_;  // The target robot's current position
  std::vector<Resource> route_resources_;      // List of resources managed by the server, read from yaml config file
};

long long getUnixTimestamp();
bool validateDistances(const float registration_distance, const float release_distance);

#endif  // RESOURCE_MONITOR_HPP
