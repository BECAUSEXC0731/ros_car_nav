cmake_minimum_required(VERSION 3.8)
project(xcar_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# odom -> base_footprint 的降级方案（下位机没有 IMU 时使用）
add_executable(odom2tf src/odom2tf.cpp)
ament_target_dependencies(odom2tf
  rclcpp tf2 nav_msgs geometry_msgs tf2_ros
)
install(TARGETS odom2tf
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# 让 imu_repub.py / odom_cov_repub.py 可以作为
# ros2 run xcar_bringup <脚本名> 执行
install(PROGRAMS
  scripts/imu_repub.py
  scripts/odom_cov_repub.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
