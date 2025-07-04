cmake_minimum_required(VERSION 3.5)
project(robot_trajectory)


if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(square src/square.cpp)
ament_target_dependencies(square rclcpp geometry_msgs nav_msgs)

add_executable(square_odom src/square_odom.cpp)
ament_target_dependencies(square_odom rclcpp geometry_msgs nav_msgs)

add_executable(wandering src/wandering.cpp)
ament_target_dependencies(wandering rclcpp geometry_msgs sensor_msgs)

install(TARGETS
	square
	square_odom
	wandering
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch                     #para el correcto funcionamiento del launch file
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
