cmake_minimum_required(VERSION 3.5)
project(rmf_resource_monitor)

# C++14を使用
set(CMAKE_CXX_STANDARD 14)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_obstacle_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ヘッダーファイルのディレクトリをインクルードパスに追加
include_directories(
  include
  ${CURL_INCLUDE_DIRS}
)

# 実行ファイルを追加
add_executable(resource_monitor_node src/resource_monitor.cpp)

# リンクするライブラリ
target_link_libraries(resource_monitor_node
  ${CURL_LIBRARIES}
  nlohmann_json::nlohmann_json
  yaml-cpp
)

# 依存関係を追加
ament_target_dependencies(resource_monitor_node
  rclcpp
  geometry_msgs
  std_msgs
  rmf_fleet_msgs
  rmf_obstacle_msgs
  nlohmann_json
  ament_index_cpp
)

# 実行可能ファイルのインストール
install(TARGETS resource_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

# ディレクトリのインストール
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)


ament_package()
