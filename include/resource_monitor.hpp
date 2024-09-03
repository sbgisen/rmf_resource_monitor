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
  // コンストラクタ
  ResourceMonitor();

private:
  std::string robot_id_;
  std::string building_id_;
  std::string server_url_;
  std::string resource_config_file_;
  float resource_registration_distance_;  // Register to resources within this distance.
  float resource_release_distance_;       // Release registered resources at a distance away from this value.

  // リソースを表す構造体
  struct Resource
  {
    std::string resource_id_;
    std::string floor_id_;
    float coord_x_;
    float coord_y_;
    bool registration_state_;
  };

  // フリート状態トピックのコールバック関数
  void fleetCallback(const std::shared_ptr<const rmf_fleet_msgs::msg::FleetState>& msg);
  // 2つのポーズ間の距離を計算する関数
  double calculateDistance(const geometry_msgs::msg::Pose& position1, const float coord_x, const float coord_y);
  // 一定間隔でリソースを確認してアクセスする関数
  void checkAndAccessResources();
  // サーバーにリソース登録リクエストを送信する関数
  nlohmann::json accessResourceServer(const Resource& resource, const std::string& api_endpoint);

  // リソース登録解除を行う関数
  // void release_resource(const Resource &resource);

  //タイマーコールバック関数
  void timerCallback();

  // 障害物をパブリッシュする関数
  void publishObstacle(const float x, const float y);

  // YAMLファイルを読み込む関数
  void loadResourcesFromYaml(const std::string& yaml_file);

  // スケジュールトピックのコールバック関数
  // void schedule_callback(const std_msgs::msg::String::SharedPtr msg);

  // スケジュールデータからリソースを抽出する関数
  // std::vector<Resource> extract_resources_from_schedule(const std::string &schedule_data);

  // メンバ変数
  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr fleet_subscription_;  // フリート状態サブスクライバ
  rclcpp::Publisher<rmf_obstacle_msgs::msg::Obstacles>::SharedPtr obstacle_publisher_;  // 障害物パブリッシャ
  rclcpp::TimerBase::SharedPtr timer_;                                                  // 定期実行タイマー

  geometry_msgs::msg::Pose current_position_;  // 現在位置
  std::vector<Resource> route_resources_;      // 経路上のリソース
  std::string registered_resource_;            //ロボットが専有しているリソース

  bool first_fleet_message_received_;  //初めてフリートメッセージをサブスクライブしたかかどうかのフラグ
};

#endif  // RESOURCE_MONITOR_HPP
