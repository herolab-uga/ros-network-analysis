cmake_minimum_required(VERSION 2.8.3)
project(network_analysis)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib
  actionlib_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   WirelessLink.msg
   LinkUtilization.msg
   NetworkErrors.msg
   NetworkDelay.msg
   NetworkAnalysis.msg
 )

## Generate actions in the 'action' folder
 add_action_files(
   DIRECTORY action   
   FILES
   Ping.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs actionlib_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES network_analysis
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime actionlib_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(network_analysis
#   src/${PROJECT_NAME}/network_analysis.cpp
# )

add_executable(pingactionserver src/server_network_delay.cpp)
target_link_libraries(pingactionserver ${catkin_LIBRARIES})
add_dependencies(pingactionserver network_analysis_EXPORTED_TARGETS)


add_executable(network_delay src/network_delay.cpp)
target_link_libraries(network_delay ${catkin_LIBRARIES})
add_dependencies(network_delay network_analysis_gencpp network_analysis_EXPORTED_TARGETS)


## Specify libraries to link a library or executable target against
# target_link_libraries(network_analysis_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS network_analysis network_analysis_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_network_analysis.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
