cmake_minimum_required(VERSION 3.8)
project(xcar_jiguang)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(lidar_loc src/lidar_loc.cpp)
target_include_directories(lidar_loc PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(lidar_loc ${OpenCV_LIBS})
ament_target_dependencies(lidar_loc
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS lidar_loc
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
