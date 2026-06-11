cmake_minimum_required(VERSION 3.8)
project(cabin_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cabin_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
)

add_executable(thruster_controller src/thruster_controller.cpp)
add_executable(pwm_controller src/pwm_controller.cpp)

ament_target_dependencies(thruster_controller 
  rclcpp
  std_msgs
  geometry_msgs
  Eigen3
  Ceres
  cabin_interface
)

ament_target_dependencies(pwm_controller 
  rclcpp
  std_msgs
  cabin_interface
)

target_link_libraries(thruster_controller
  ${CERES_LIBRARIES}
  Eigen3::Eigen
)

install(TARGETS
  thruster_controller
  pwm_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
# 安装 scripts 里的所有 Python 节点
install(
  PROGRAMS scripts/serial_to_mcu.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
