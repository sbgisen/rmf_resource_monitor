cmake_minimum_required(VERSION 3.5)
project(rmf_resource_monitor)

if(CMAKE_COMPILER_IS_GNUCXX
   OR CMAKE_CXX_COMPILER_ID
      MATCHES
      "Clang"
)
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

set(CMAKE_CXX_STANDARD 14)

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

add_executable(resource_monitor_node src/resource_monitor.cpp)

target_include_directories(resource_monitor_node PRIVATE ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(
  resource_monitor_node
  ${CURL_LIBRARIES}
  nlohmann_json::nlohmann_json
  yaml-cpp
)

ament_target_dependencies(
  resource_monitor_node
  rclcpp
  geometry_msgs
  std_msgs
  rmf_fleet_msgs
  rmf_obstacle_msgs
  nlohmann_json
  ament_index_cpp
)

install(
  TARGETS resource_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
