cmake_minimum_required(VERSION 3.5)
project(ros2_network_analysis)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)

rosidl_generate_interfaces( ${PROJECT_NAME}
  "msg/WirelessLink.msg"
  "msg/LinkUtilization.msg"
  "msg/NetworkErrors.msg"
  "msg/NetworkDelay.msg"
  "msg/NetworkAnalysis.msg"
  "msg/WirelessLinkVector.msg"
  "action/Ping.srv"
  DEPENDENCIES builtin_interfaces std_msgs 
 )

ament_export_dependencies(rosidl_default_runtime)

 include_directories(
  ${Boost_INCLUDE_DIRS}
)

add_executable(network_ping_client src/ping_client.cpp)
# add_dependencies(${target}${target_suffix} ${PROJECT_NAME})
ament_target_dependencies(network_ping_client rclcpp std_msgs)

add_executable(network_ping_service src/ping_service.cpp)
# add_dependencies(${target}${target_suffix} ${PROJECT_NAME})
ament_target_dependencies(network_ping_service rclcpp std_msgs)


install(TARGETS
network_ping_client
network_ping_service
DESTINATION lib/${PROJECT_NAME})

rosidl_target_interfaces(network_ping_client
      ${PROJECT_NAME} "rosidl_typesupport_cpp")
    
rosidl_target_interfaces(network_ping_service
      ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_python_install_package(${PROJECT_NAME})

install (PROGRAMS
  scripts/link_utilization.py
  scripts/network_errors.py
  scripts/wireless_quality.py
  DESTINATION lib/${PROJECT_NAME}
)


# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
