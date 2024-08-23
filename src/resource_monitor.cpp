/*実装したい機能
/schedule_markersトピックにsubscribeして経路情報を取得して、移動経路内に存在するリソースを確認
/fleet_statesトピックにsubscribeして現在位置情報えお取得し続けて距離が5m以内に到達した場合にリソース管理サーバーへのアクセス
利用登録に成功した場合は通過まで何もしない
利用登録に失敗した場合は、/rmf_obstacles(rmf_obstacle_msgs::msg::Obstacles型)トピックえおpublishして障害物を発生させる
利用登録に成功するまでアクセスを繰り返す
リソース通過後利用登録を解除
*/

#include "resource_monitor.hpp"

// グローバル関数の実装
static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

ResourceMonitor::ResourceMonitor() : Node("resource_monitor"), resource_registered_(false), first_fleet_message_received_(false)
{
    // 初期リソース設定
    route_resources_ = {
        {"Resource01", "27F", 0.0, 0.0}
    };

    passing_resource = "";

    // /fleet_states トピックにサブスクライブ
    fleet_subscription_ = this->create_subscription<rmf_fleet_msgs::msg::FleetState>(
        "/fleet_states", 10, std::bind(&ResourceMonitor::fleet_callback, this, std::placeholders::_1));

    // 障害物トピックのパブリッシャーを設定
    obstacle_publisher_ = this->create_publisher<rmf_obstacle_msgs::msg::Obstacles>(
        "/rmf_obstacles", 10);

    // タイマーを設定して、初回のフリートメッセージを受信した後にcheck_and_access_resourcesを実行
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0),  // 1秒ごとに実行
        std::bind(&ResourceMonitor::timer_callback, this)
    );
}

// ロボットの現在位置(x, y)を取得し、その後にリソースを確認するcallback関数
void ResourceMonitor::fleet_callback(const rmf_fleet_msgs::msg::FleetState::SharedPtr msg)
{
    for (const auto &robot : msg->robots)
    {
        // ロボットの現在位置を取得
        current_position_.position.x = robot.location.x;
        current_position_.position.y = robot.location.y;

        // ログの出力
        RCLCPP_INFO(this->get_logger(), "Robot %s position: x=%.2f, y=%.2f",
                    robot.name.c_str(), current_position_.position.x, current_position_.position.y);
    }

    // 初めてフリートメッセージを受信した場合
    if (!first_fleet_message_received_)
    {
        first_fleet_message_received_ = true;
        RCLCPP_INFO(this->get_logger(), "First fleet message received. Starting periodic resource checks.");
    }
}

// タイマーコールバック関数
void ResourceMonitor::timer_callback()
{
    // 初めてのフリートメッセージが受信された後に実行
    if (first_fleet_message_received_)
    {
        check_and_access_resources();
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Fleet messages have't been subscribed") ;
    }
}

// 2つのpose間の距離を計算する関数
double ResourceMonitor::calculate_distance(const geometry_msgs::msg::Pose &position1, const float coord_x, const float coord_y)
{
    return std::sqrt(std::pow(position1.position.x - coord_x, 2) + 
                     std::pow(position1.position.y - coord_y, 2));
}

// リソースを確認しアクセスする関数
void ResourceMonitor::check_and_access_resources()
{
    // リソースのリストをループ
    for (const auto &resource : route_resources_)
    {
        // 距離を計算
        double distance = calculate_distance(current_position_, resource.coord_x, resource.coord_y);
        
        // 対象のリソースとの距離が5m以内かつ対象のリソースを通過中でない場合、サーバーに登録リクエストを送信
        if (distance <= 5.0 && passing_resource != resource.resource_id)
        {
            bool success = false;

            while (!success)
            {
                // サーバーにリクエストを送信し、レスポンスを取得
                nlohmann::json response_json = access_resource_server(resource);

                // レスポンスが空でないか、またはエラーフィールドの処理
                if (!response_json.is_null())
                {
                    int result = response_json.value("result", -1);

                    // レスポンスの結果に基づいて処理を分岐
                    if (result == 1)
                    {
                        RCLCPP_INFO(this->get_logger(), "Resource %s registered successfully.", resource.resource_id.c_str());
                        success = true;
                        passing_resource = resource.resource_id;
                    }
                    else if (result == 2)
                    {
                        RCLCPP_WARN(this->get_logger(), "Resource %s is already registered by other robot.", resource.resource_id.c_str());             
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "Failed to register resource %s. Result code: %d", resource.resource_id.c_str(), result);
                        rclcpp::sleep_for(std::chrono::seconds(2));  // 一定の待機時間を設定して再試行
                    }
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Received an invalid or empty response from the server for resource %s.", resource.resource_id.c_str());
                    rclcpp::sleep_for(std::chrono::seconds(2));  // 一定の待機時間を設定して再試行
                }
            }
        }
    }
}

nlohmann::json ResourceMonitor::access_resource_server(const Resource &resource)
{
    CURL *curl;
    CURLcode res;
    std::string response_data;  // サーバーからのレスポンスデータを格納する変数
    nlohmann::json response_json;

    curl = curl_easy_init();
    if (curl)
    {
        std::string url = "http://127.0.0.1:5000/api/registration";
        std::string json_data = "{\"api\":\"Registration\",\"bldg_id\":\"Takeshiba\",\"resource_id\":\"" + resource.resource_id + "\",\"robot_id\":\"Robot01\",\"request_id\":\"Request01\"}";

        // URLの設定
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

        // POSTデータの設定
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_data.c_str());

        // Content-Type ヘッダーの設定
        struct curl_slist *headers = NULL;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

        // レスポンスデータを受け取るためのコールバック関数を設定
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_data);

        // リクエストを実行
        res = curl_easy_perform(curl);

        // エラーチェック
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
                // JSONレスポンスを解析
                try
                {
                    response_json = nlohmann::json::parse(response_data);
                    int result = response_json["result"];

                    if (result == 1)
                    {
                        RCLCPP_INFO(this->get_logger(), "Resource registration successful");
                        resource_registered_ = true;
                    }
                    else if (result == 2)
                    {
                        RCLCPP_WARN(this->get_logger(), "Resource already registered");
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "Registration failed with result code: %d", result);
                        access_resource_server(resource); // 成功するまで再試行
                    }
                }
                catch (nlohmann::json::parse_error &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to parse response JSON: %s", e.what());
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "HTTP request failed with status code %ld", response_code);
            }
        }

        // ヘッダーの解放
        curl_slist_free_all(headers);
        // リソースの解放
        curl_easy_cleanup(curl);
    }

    // パースされたJSONレスポンスを返す
    return response_json;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ResourceMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
