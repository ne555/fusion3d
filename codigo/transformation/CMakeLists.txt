cmake_minimum_required(VERSION 3.7)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_BUILD_TYPE Debug)

project(transformation)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb")
#add_executable(transformation transformation.cpp)
#add_executable(transformation standford.cpp)
#add_executable(transformation camera_pos.cpp)
add_executable(transformation partial.cpp)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem ~/.local/include/ -I../../" )

target_link_libraries(transformation ${PCL_LIBRARIES})


include(CTest)
#add_test(Runs transformation "/home/ne555/test/fusion_3d/databases/stanford_3d/bunny/data/bun000.ply") 
