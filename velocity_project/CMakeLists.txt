cmake_minimum_required(VERSION 2.8.3)
project(velocity_project)

find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables
cs_add_executable(amplitude_frequency_client src/amplitude_frequency_client.cpp)
cs_add_executable(velocity_commander src/velocity_commander.cpp)
cs_add_executable(velocity_controller src/velocity_controller.cpp)
cs_add_executable(velocity_simulator src/velocity_simulator.cpp)

# target_link_library(example my_lib)

cs_install()
cs_export()
    