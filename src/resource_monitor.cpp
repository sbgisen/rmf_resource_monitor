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

#include "resource_monitor.hpp"

static size_t writeCallback(void* contents, size_t size, size_t nmemb, void* userp)
{
  ((std::string*)userp)->append((char*)contents, size * nmemb);
  return size * nmemb;
}

// Constructor
ResourceMonitor::ResourceMonitor() : Node("resource_monitor")
{
  this->declare_parameter<std::string>("robot_id", "robot");
  this->declare_parameter<std::string>("building_id", "");
  this->declare_parameter<std::string>("server_url", "http://127.0.0.1:5000");
  this->declare_parameter<std::string>("resource_config_file", "");
  this->declare_parameter<double>("resource_registration_distance", 3.4);
  this->declare_parameter<double>("resource_release_distance", 4.0);

  // Get parameters from parameter server
  robot_id_ = this->get_parameter("robot_id").as_string();
  building_id_ = this->get_parameter("building_id").as_string();
  server_url_ = this->get_parameter("server_url").as_string();
  resource_config_file_ = this->get_parameter("resource_config_file").as_string();
  resource_registration_distance_ = this->get_parameter("resource_registration_distance").as_double();
  resource_release_distance_ = this->get_parameter("resource_release_distance").as_double();
  RCLCPP_INFO(this->get_logger(), "Loading resource info from file: %s", resource_config_file_.c_str());
  loadResourcesFromYaml(resource_config_file_);

  // Initialize internal variables
  current_floor_id_ = "";
  registered_resource_ = "";  // TODO: Manage more than one resource at a time?

  // Subscribe to /fleet_states to obtain current fleet info
  fleet_subscription_ = this->create_subscription<rmf_fleet_msgs::msg::FleetState>(
      "/fleet_states", 10, std::bind(&ResourceMonitor::fleetCallback, this, std::placeholders::_1));

  // Publish unavailable resources as obstacles
  obstacle_publisher_ = this->create_publisher<rmf_obstacle_msgs::msg::Obstacles>("/rmf_obstacles", 10);

  // Run the check_and_access_resources function after receiving the first fleet message (1Hz)
  timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&ResourceMonitor::timerCallback, this));
}

void ResourceMonitor::fleetCallback(const std::shared_ptr<const rmf_fleet_msgs::msg::FleetState>& msg)
{
  for (const auto& robot : msg->robots)
  {
    if (robot.name != robot_id_)
    {
      continue;
    }
    current_position_.position.x = robot.location.x;
    current_position_.position.y = robot.location.y;
    current_floor_id_ = robot.location.level_name;

    RCLCPP_INFO(this->get_logger(), "Robot %s position: x=%.2f, y=%.2f, level_name=%s", robot.name.c_str(),
                current_position_.position.x, current_position_.position.y, current_floor_id_.c_str());
    // When the first fleet message is received, start the periodic resource checks
    if (!first_fleet_message_received_)
    {
      first_fleet_message_received_ = true;
      RCLCPP_INFO(this->get_logger(), "First fleet message received for %s. Starting periodic resource checks.",
                  robot_id_.c_str());
    }
    break;
  }
}

// Periodically Run: Function to check and access resources
void ResourceMonitor::checkAndAccessResources()
{
  for (auto& resource : route_resources_)
  {
    // Ignore resources that are not on the same floor
    if (resource.floor_id_ != current_floor_id_)
    {
      continue;
    }

    double distance = calculateDistance(current_position_, resource.coord_x_, resource.coord_y_);

    // Request registration to server when the distance to the target resource is within the specified distance and the
    // robot is not passing
    if (distance <= resource_registration_distance_ && registered_resource_ != resource.resource_id_)
    {
      if (rclcpp::ok() && !resource.registration_state_)
      {
        // Receive response from server
        nlohmann::json response_json = accessResourceServer(resource, "registration");

        if (!response_json.is_null())
        {
          int result = response_json.value("result", -1);

          if (result == 1)
          {
            RCLCPP_INFO(this->get_logger(), "Registration: Resource %s registered successfully.",
                        resource.resource_id_.c_str());
            resource.registration_state_ = true;
            registered_resource_ = resource.resource_id_;
          }
          else if (result == 2)
          {
            RCLCPP_WARN(this->get_logger(), "Registration: Resource %s is already registered by other robot.",
                        resource.resource_id_.c_str());
            publishObstacle(resource.coord_x_, resource.coord_y_);
            rclcpp::sleep_for(std::chrono::milliseconds(500));
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "Registration: Failed to register resource %s. Result code: %d",
                        resource.resource_id_.c_str(), result);
            rclcpp::sleep_for(std::chrono::milliseconds(500));
          }
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(),
                       "Registration: Received an invalid or empty response from the server for resource %s.",
                       resource.resource_id_.c_str());
          rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
      }
    }
    else if (distance >= resource_release_distance_ && registered_resource_ == resource.resource_id_)
    {
      if (resource.registration_state_)
      {
        nlohmann::json response_json = accessResourceServer(resource, "release");

        if (!response_json.is_null())
        {
          int result = response_json.value("result", -1);

          if (result == 1)
          {
            RCLCPP_INFO(this->get_logger(), "Release: Resource %s Successfully.", resource.resource_id_.c_str());
            resource.registration_state_ = false;
            registered_resource_ = "";
          }
          else if (result == 2)
          {
            RCLCPP_WARN(this->get_logger(), "Release: Resource %s Failed.", resource.resource_id_.c_str());
            rclcpp::sleep_for(std::chrono::milliseconds(500));
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "Release: Error resource %s. Result code: %d",
                         resource.resource_id_.c_str(), result);
            rclcpp::sleep_for(std::chrono::milliseconds(500));
          }
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(),
                       "Release: Received an invalid or empty response from the server for resource %s.",
                       resource.resource_id_.c_str());
          rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
      }
    }
  }
}

void ResourceMonitor::timerCallback()
{
  if (first_fleet_message_received_)
  {
    checkAndAccessResources();
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Fleet messages have't been subscribed");
  }
}

double ResourceMonitor::calculateDistance(const geometry_msgs::msg::Pose& position1, const float coord_x,
                                          const float coord_y)
{
  return std::sqrt(std::pow(position1.position.x - coord_x, 2) + std::pow(position1.position.y - coord_y, 2));
}

nlohmann::json ResourceMonitor::accessResourceServer(const Resource& resource, const std::string& api_endpoint)
{
  CURL* curl;
  CURLcode res;
  std::string response_data;
  nlohmann::json response_json;

  curl = curl_easy_init();

  if (curl)
  {
    std::string url = server_url_ + "/api/" + api_endpoint;
    std::string json_data;

    if (api_endpoint == "registration")
      json_data = "{\"api\":\"Registration\",\"bldg_id\":\"" + building_id_ + "\",\"resource_id_\":\"" +
                  resource.resource_id_ + "\",\"robot_id\":\"" + robot_id_ + "\",\"request_id\":\"Request01\"}";
    else if (api_endpoint == "release")
      json_data = "{\"api\":\"Release\",\"bldg_id\":\"" + building_id_ + "\",\"resource_id_\":\"" +
                  resource.resource_id_ + "\",\"robot_id\":\"" + robot_id_ + "\",\"request_id\":\"Request01\"}";
    // URLの設定
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

    // POSTデータの設定
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_data.c_str());

    // Content-Type ヘッダーの設定
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_data);

    res = curl_easy_perform(curl);

    if (res != CURLE_OK)
    {
      RCLCPP_ERROR(this->get_logger(), "curl_easy_perform() failed: %s", curl_easy_strerror(res));
    }
    else
    {
      long response_code;
      curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);

      if (response_code == 200)
      {
        try
        {
          response_json = nlohmann::json::parse(response_data);
          int result = response_json["result"];

          if (result == 1)
          {
            RCLCPP_INFO(this->get_logger(), "%s: Successful", api_endpoint.c_str());
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "%s: Failed with result code: %d", api_endpoint.c_str(), result);
          }
        }
        catch (nlohmann::json::parse_error& e)
        {
          RCLCPP_ERROR(this->get_logger(), "%s: Failed to parse response JSON: %s", api_endpoint.c_str(), e.what());
        }
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "%s: HTTP request failed with status code %ld", api_endpoint.c_str(),
                    response_code);
      }
    }

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
  }

  return response_json;
}

void ResourceMonitor::publishObstacle(const float x, const float y)
{
  // Generate obstacle message
  auto message = rmf_obstacle_msgs::msg::Obstacles();

  message.header.frame_id = "map";
  rmf_obstacle_msgs::msg::Obstacle obstacle;
  obstacle.header.frame_id = "map";
  obstacle.header.stamp = this->get_clock()->now();
  obstacle.id = 1;
  obstacle.classification = "example_obstacle";
  obstacle.source = "ResourceMonitor";
  obstacle.bbox.center.position.x = x;
  obstacle.bbox.center.position.y = y;
  obstacle.bbox.center.position.z = 0.0;
  obstacle.bbox.size.x = 2.0;
  obstacle.bbox.size.y = 2.0;
  obstacle.bbox.size.z = 2.0;
  obstacle.level_name = current_floor_id_;
  obstacle.lifetime = rclcpp::Duration(1, 0);  // Obstacle will be removed after 1 second

  message.obstacles.push_back(obstacle);

  RCLCPP_INFO(this->get_logger(), "Publishing obstacle");
  obstacle_publisher_->publish(message);
}

void ResourceMonitor::loadResourcesFromYaml(const std::string& yaml_file)
{
  try
  {
    YAML::Node config = YAML::LoadFile(yaml_file);
    for (const auto& resource : config["route_resources"])
    {
      Resource res;
      res.resource_id_ = resource["resource_id"].as<std::string>();
      res.floor_id_ = resource["floor_id"].as<std::string>();
      res.coord_x_ = resource["coord_x"].as<float>();
      res.coord_y_ = resource["coord_y"].as<float>();
      res.registration_state_ = false;
      route_resources_.emplace_back(res);
    }
  }
  catch (const YAML::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", e.what());
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ResourceMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
