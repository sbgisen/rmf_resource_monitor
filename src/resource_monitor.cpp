/*実装したい機能
/schedule_markersトピックにsubscribeして経路情報を取得して、移動経路内に存在するリソースを確認
/fleet_statesトピックにsubscribeして現在位置情報えお取得し続けて距離が5m以内に到達した場合にリソース管理サーバーへのアクセス
利用登録に成功した場合は通過まで何もしない
利用登録に失敗した場合は、/rmf_obstacles(rmf_obstacle_msgs::msg::Obstacles型)トピックえおpublishして障害物を発生させる
利用登録に成功するまでアクセスを繰り返す
リソース通過後利用登録を解除
*/

#include "resource_monitor.hpp"

ResourceMonitor::ResourceMonitor() : Node("resource_monitor"), resource_registered_(false)
{
    // /fleet_states トピックにサブスクライブ
    fleet_subscription_ = this->create_subscription<rmf_fleet_msgs::msg::FleetState>(
        "/fleet_states", 10, std::bind(&ResourceMonitor::fleet_callback, this, std::placeholders::_1));

    // 障害物トピックのパブリッシャーを設定
    obstacle_publisher_ = this->create_publisher<rmf_obstacle_msgs::msg::Obstacles>(
        "/rmf_obstacles", 10);

    // タイマーを設定して一定の周波数でcheck_and_access_resourcesを実行
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0),  // 1秒ごとに実行
        std::bind(&ResourceMonitor::check_and_access_resources, this)
    );
}


//ロボットの現在位置(x, y)を取得するcallback関数
void ResourceMonitor::fleet_callback(const rmf_fleet_msgs::msg::FleetState::SharedPtr msg)
{
    for (const auto &robot : msg->robots)
    {
        // ロボットの現在位置を取得
        current_position_.position.x = robot.location.x;
        current_position_.position.y = robot.location.y;
        //ログの出力
        RCLCPP_INFO(this->get_logger(), "Robot %s position: x=%.2f, y=%.2f",
                    robot.name.c_str(), current_position_.position.x, current_position_.position.y);
    }
}

//2つのpose間のデータを計算
double ResourceMonitor::calculate_distance(const geometry_msgs::msg::Pose &position1, const geometry_msgs::msg::Pose &position2)
{
    return std::sqrt(std::pow(position1.position.x - position2.position.x, 2) + 
                     std::pow(position1.position.y - position2.position.y, 2));
}


void ResourceMonitor::check_and_access_resources()
{
    // リソースのリストをループ
    for (const auto &resource : route_resources_)
    {
        // 距離を計算
        double distance = calculate_distance(current_position_, resource.position);
        
        // 距離が5m以内の場合、サーバーにリクエストを送信
        if (distance <= 5.0)
        {
            bool success = false;

            while (!success)
            {
                // サーバーにリクエストを送信し、レスポンスを取得
                json response_json = access_resource_server(resource);

                // レスポンスが空でないか、またはエラーフィールドの処理
                if (!response_json.is_null())
                {
                    int result = response_json.value("result", -1);

                    // レスポンスの結果に基づいて処理を分岐
                    if (result == 1)
                    {
                        RCLCPP_INFO(this->get_logger(), "Resource %s registered successfully.", resource.resource_id.c_str());
                        success = true;
                    }
                    else if (result == 2)
                    {
                        RCLCPP_WARN(this->get_logger(), "Resource %s is already registered.", resource.resource_id.c_str());
                        success = true; // 登録済みと判断して終了
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "Failed to register resource %s. Result code: %d", resource.resource_id.c_str(), result);
                        publish_obstacle();
                        // 一定の待機時間を設定して再試行
                        rclcpp::sleep_for(std::chrono::seconds(2));
                    }
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Received an invalid or empty response from the server for resource %s.", resource.resource_id.c_str());
                    // 一定の待機時間を設定して再試行
                    rclcpp::sleep_for(std::chrono::seconds(2));
                }
            }
        }
    }
}



json ResourceMonitor::access_resource_server(const Resource &resource)
{
    CURL *curl;
    CURLcode res;
    std::string response_data;  // サーバーからのレスポンスデータを格納する変数
    json response_json;

    curl = curl_easy_init();
    if (curl)
    {
        std::string url = "http://127.0.0.1:5000/api/registration";
        std::string json_data = "{\"api\":\"Registration\",\"bldg_id\":\"Building01\",\"resource_id\":\"" + resource.resource_id + "\",\"robot_id\":\"Robot01\",\"request_id\":\"Request01\"}";

        // URLの設定
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        // POSTデータの設定
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_data.c_str());

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
                    response_json = json::parse(response_data);
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
                        publish_obstacle();
                        access_resource_server(resource); // 成功するまで再試行
                    }
                }
                catch (json::parse_error &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to parse response JSON: %s", e.what());
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "HTTP request failed with status code %ld", response_code);
            }
        }

        // リソースの解放
        curl_easy_cleanup(curl);
    }

    // パースされたJSONレスポンスを返す
    return response_json;
}

static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}


/*
void ResourceMonitor::schedule_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // 経路情報の処理
    route_resources_ = extract_resources_from_schedule(msg->data);
    RCLCPP_INFO(this->get_logger(), "Received schedule markers: %s", msg->data.c_str());
}*/

/*
std::vector<ResourceMonitor::Resource> ResourceMonitor::extract_resources_from_schedule(const std::string &schedule_data)
{
    // スケジュールデータからリソースを抽出するロジックを実装
    // 仮のリソースリストを返す
    return {{"Resource01", geometry_msgs::msg::Pose()}}; // 例として空のPoseを使用
}
*/

/*
void ResourceMonitor::publish_obstacle()
{
    // 障害物をパブリッシュ
    auto obstacle_msg = rmf_obstacle_msgs::msg::Obstacles();
    // 障害物の詳細を obstacle_msg に設定
    obstacle_publisher_->publish(obstacle_msg);
    RCLCPP_INFO(this->get_logger(), "Published obstacle");
}
*/

/*
void ResourceMonitor::release_resource(const Resource &resource)
{
    // リソース登録解除の処理を追加
    CURL *curl;
    CURLcode res;

    curl = curl_easy_init();
    if (curl)
    {
        std::string url = "http://127.0.0.1:5000/api/release";
        std::string json_data = "{\"api\":\"Release\",\"bldg_id\":\"Building01\",\"resource_id\":\"" + resource.resource_id + "\",\"robot_id\":\"Robot01\",\"request_id\":\"Request01\"}";

        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_data.c_str());

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
                RCLCPP_INFO(this->get_logger(), "Resource release successful");
                resource_registered_ = false;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Resource release failed");
            }
        }

        curl_easy_cleanup(curl);
    }
}
*/

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ResourceMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
