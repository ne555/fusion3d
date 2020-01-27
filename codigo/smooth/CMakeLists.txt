cmake_minimum_required (VERSION 3.0 FATAL_ERROR)

project (smooth)

set (CMAKE_ROOT_PATH /home/walter/deps)
set (LIB_INSTALL_DIR=/home/walter/deps/lib)
set (CMAKE_PREFIX_PATH /home/walter/deps)
set (PCL_DIR /home/walter/deps/share/pcl-1.9)

find_package (PCL 1.9 REQUIRED)
#find_package (PCL 1.9 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package (flann REQUIRED)

include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )


set (CMAKE_CXX_FLAGS "")
set (CMAKE_C_FLAGS "")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -ggdb -isystem /home/walter/deps/include/")

add_executable  (smooth main.cpp)
target_link_libraries( smooth Eigen3::Eigen )
target_link_libraries( smooth ${PCL_LIBRARIES} )
