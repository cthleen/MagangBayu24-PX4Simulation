cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(circle src/tugas1/circle.cpp)
ament_target_dependencies(circle rclcpp px4_msgs)
install(TARGETS circle DESTINATION lib/${PROJECT_NAME})

add_executable(rectangle src/tugas2/rectangle.cpp)
ament_target_dependencies(rectangle rclcpp px4_msgs)
install(TARGETS rectangle DESTINATION lib/${PROJECT_NAME})

add_executable(star src/tugas3/star.cpp)
ament_target_dependencies(star rclcpp px4_msgs)
install(TARGETS star DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
