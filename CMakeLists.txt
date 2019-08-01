cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(percent)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Eigen3 REQUIRED)

find_package(Sophus REQUIRED)
include_directories( ${Sophus_INCLUDE_DIRS} )
set(Sophus_LIBRARIES "/usr/local/lib/libSophus.so")

set( CMAKE_CXX_FLAGS "-std=c++11 -O2" )

add_executable(percent a.cpp)
target_link_libraries(percent 
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${Sophus_LIBRARIES}
)
link_directories(${OpenCV_LIBRARY_DIRS})

