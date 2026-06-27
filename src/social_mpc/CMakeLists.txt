cmake_minimum_required(VERSION 3.28)
project(social_mpc)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(social_navigation_msgs REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# ACADOS
set(ACADOS_ROOT $ENV{HOME}/acados)
set(ACADOS_INCLUDE_DIRS
    ${ACADOS_ROOT}/include
    ${ACADOS_ROOT}/include/blasfeo/include
    ${ACADOS_ROOT}/include/hpipm/include
)
set(ACADOS_LIB_DIR ${ACADOS_ROOT}/lib)

set(ACADOS_GEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/acados_generated)

file(GLOB ACADOS_GEN_SOURCES
    ${ACADOS_GEN_DIR}/*.c
    ${ACADOS_GEN_DIR}/social_mpc_model/*.c
    ${ACADOS_GEN_DIR}/social_mpc_constraints/*.c
    ${ACADOS_GEN_DIR}/social_mpc_cost/*.c
)
list(FILTER ACADOS_GEN_SOURCES EXCLUDE REGEX "main_.*\\.c$")
list(FILTER ACADOS_GEN_SOURCES EXCLUDE REGEX "acados_sim_solver.*\\.c$")

set(DEPS
  rclcpp rclcpp_lifecycle geometry_msgs nav_msgs
  tf2_ros tf2_geometry_msgs social_navigation_msgs
)

add_executable(social_mpc_node
  src/social_mpc_node.cpp
  src/robot_kinematics.cpp
  src/mpc_solver.cpp
  ${ACADOS_GEN_SOURCES}
)
ament_target_dependencies(social_mpc_node ${DEPS})
target_include_directories(social_mpc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${ACADOS_INCLUDE_DIRS}
  ${ACADOS_GEN_DIR}
)
target_link_directories(social_mpc_node PUBLIC ${ACADOS_LIB_DIR})
target_link_libraries(social_mpc_node
  Eigen3::Eigen
  acados
  hpipm
  blasfeo
)

install(TARGETS social_mpc_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_robot_kinematics test/test_robot_kinematics.cpp
    src/robot_kinematics.cpp
  )
  target_include_directories(test_robot_kinematics PRIVATE include)
  target_link_libraries(test_robot_kinematics Eigen3::Eigen)
endif()

ament_package()
