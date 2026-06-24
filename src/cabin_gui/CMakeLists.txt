cmake_minimum_required(VERSION 3.8)
project(cabin_gui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_AUTOMOC ON)  #启用自动 moc 处理
set(CMAKE_AUTOUIC ON)  #自动处理 ui 文件
set(CMAKE_AUTORCC ON)  #自动处理资源文件

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cabin_interface REQUIRED) 
find_package(ping360_sonar_msgs REQUIRED)

#查找 Qt5 组件
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(Qt5 REQUIRED COMPONENTS Widgets Network)
#添加可执行文件
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/mainwindow.cpp
)
#链接库
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    cabin_interface
    ping360_sonar_msgs 
)

target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
    Qt5::Network
)
# 包含目录
target_include_directories(${PROJECT_NAME} PUBLIC
    ${Qt5Widgets_INCLUDE_DIRS}
)
# 安装规则
install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
