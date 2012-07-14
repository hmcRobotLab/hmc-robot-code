cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(iterative_closest_point)

find_package(PCL 1.6 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS regex filesystem)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories({${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_FLAGS "-g")
add_executable (big example.cpp)
target_link_libraries (big ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
