cmake_minimum_required(VERSION 2.8.3)
project(velocity_project)

## This is where you put all of the dependencies for the package.
## When using catkin_create_pkg, this is one of the places where 
## the dependencies are auto-placed.  If you need to add more later
## on, you need to put them in manually. 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Tell catkin about your services here so it will properly generate
## the required header files

add_service_files(
	FILES
	amplitude_frequency_msg.srv
)

## Message / service dependencies go here!  Ie, if you have a 
## geometry_msgs/Twist in your message / service, you should
## put 'geometry_msgs' here.
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## This stuff tells catkin to compile your node!
add_executable(amplitude_frequency_client src/amplitude_frequency_client.cpp)
add_executable(velocity_commander src/velocity_commander.cpp)
add_executable(velocity_controller src/velocity_controller.cpp)
add_executable(velocity_simulator src/velocity_simulator.cpp)

target_link_libraries(amplitude_frequency_client

   ${catkin_LIBRARIES}
)

target_link_libraries(velocity_commander

   ${catkin_LIBRARIES}
)

target_link_libraries(velocity_controller

   ${catkin_LIBRARIES}
)

target_link_libraries(velocity_simulator

   ${catkin_LIBRARIES}
)