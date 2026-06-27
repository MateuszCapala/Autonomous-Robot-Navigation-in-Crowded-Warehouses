cmake_minimum_required(VERSION 3.28)
project(human_tracker)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(social_navigation_msgs REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)

set(DEPS
  rclcpp rclcpp_lifecycle sensor_msgs geometry_msgs visualization_msgs
  nav_msgs tf2_ros tf2_sensor_msgs pcl_ros pcl_conversions social_navigation_msgs
)

add_executable(human_tracker_node
  src/human_tracker_node.cpp
  src/lidar_preprocessor.cpp
  src/human_clusterer.cpp
  src/kalman_tracker.cpp
)
ament_target_dependencies(human_tracker_node ${DEPS})
target_include_directories(human_tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(human_tracker_node Eigen3::Eigen ${PCL_LIBRARIES})

install(TARGETS human_tracker_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kalman_tracker test/test_kalman_tracker.cpp
    src/kalman_tracker.cpp
  )
  target_include_directories(test_kalman_tracker PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_kalman_tracker Eigen3::Eigen)
endif()

ament_package()
