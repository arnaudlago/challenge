cmake_minimum_required(VERSION 3.0.2)

project(challenge_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  ackermann_msgs
  tf
  actionlib
  actionlib_msgs
  message_generation
  rostest
)

find_package(GTest REQUIRED)



## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetPath.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  Pause.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  
  nav_msgs
  actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 #INCLUDE_DIRS include
 INCLUDE_DIRS include
 LIBRARIES challenge_pkg 
 CATKIN_DEPENDS message_runtime std_msgs nav_msgs actionlib_msgs roscpp ackermann_msgs tf
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/lib
  ${GTEST_INCLUDE_DIRS}
)

## Link lib directory
link_directories(lib)

add_subdirectory(lib)

## Declare a C++ executable
add_executable(${PROJECT_NAME} 
  src/path_tracking_controller.cpp
  src/path_tracking_controller_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} pure_pursuit_lib)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  pure_pursuit_lib
)

add_executable(${PROJECT_NAME}_path_sender 
  src/path_sender.cpp
  src/path_sender_node.cpp)
target_link_libraries(${PROJECT_NAME}_path_sender ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_path_sender ${${PROJECT_NAME}_path_sender_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_pp_utest tests/pure_pursuit_utest.cpp)
  if(TARGET ${PROJECT_NAME}_pp_utest)
    target_link_libraries(${PROJECT_NAME}_pp_utest
      ${catkin_LIBRARIES}
      pure_pursuit_lib
      ${GTEST_LIBRARIES}  # Link with Google Test
      pthread
    )
  endif()

  add_rostest_gtest(${PROJECT_NAME}_ptc_utest launch/challenge.test tests/path_tracking_controller_utest.cpp)
  if(TARGET ${PROJECT_NAME}_ptc_utest)
    target_link_libraries(${PROJECT_NAME}_ptc_utest
      ${catkin_LIBRARIES}
      pure_pursuit_lib
      ${GTEST_LIBRARIES}  # Link with Google Test
      pthread
    )
  endif()
endif()

