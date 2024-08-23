#ifndef RESOURCE_MONITOR_HPP
#define RESOURCE_MONITOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_obstacle_msgs/msg/obstacles.hpp>
#include <std_msgs/msg/string.hpp>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
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
    // リソースを表す構造体
    struct Resource
    {
        std::string resource_id;
        std::string floor_id;
        float coord_x;
        float coord_y;
    };

    // フリート状態トピックのコールバック関数
    void fleet_callback(const rmf_fleet_msgs::msg::FleetState::SharedPtr msg);

    // 2つのポーズ間の距離を計算する関数
    double calculate_distance(const geometry_msgs::msg::Pose &position1, const float coord_x, const float coord_y);

    // 一定間隔でリソースを確認してアクセスする関数
    void check_and_access_resources();

    // サーバーにリソース登録リクエストを送信する関数
    nlohmann::json access_resource_server(const Resource &resource);

    // 障害物をパブリッシュする関数
    //void publish_obstacle();

    // スケジュールトピックのコールバック関数
    //void schedule_callback(const std_msgs::msg::String::SharedPtr msg);

    // スケジュールデータからリソースを抽出する関数
    //std::vector<Resource> extract_resources_from_schedule(const std::string &schedule_data);

    // リソース登録解除を行う関数
   // void release_resource(const Resource &resource);


    // メンバ変数
    rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr fleet_subscription_;  // フリート状態サブスクライバ
    rclcpp::Publisher<rmf_obstacle_msgs::msg::Obstacles>::SharedPtr obstacle_publisher_;   // 障害物パブリッシャ
    rclcpp::TimerBase::SharedPtr timer_;  // 定期実行タイマー

    geometry_msgs::msg::Pose current_position_;  // 現在位置
    std::vector<Resource> route_resources_;  // 経路上のリソース

    bool resource_registered_;  // リソースが登録されているかどうかのフラグ
};

#endif // RESOURCE_MONITOR_HPP
