cmake_minimum_required(VERSION 3.5)
project(visual_kitti)

# 一定要加-march=native， 不然使用pcl的一些函数会报错
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -march=native")
#find_package(OpenMP)
#if (OPENMP_FOUND)
#    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#endif ()

find_package(catkin REQUIRED COMPONENTS
        tf
        roscpp
        rospy
        pcl_ros
        pcl_conversions
        nav_msgs
        std_msgs
        sensor_msgs
        geometry_msgs
        )

set(ALL_TARGET_LIBRARIES "")
include(cmake/eigen.cmake)
include(cmake/PCL.cmake)
include(cmake/boost.cmake)

catkin_package()

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/loadData.cpp
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${ALL_TARGET_LIBRARIES}
        )

# visual_kitti
add_executable(dataLoader node/dataLoader.cpp)
target_link_libraries(dataLoader ${PROJECT_NAME})

add_executable(mapping node/mapping.cpp)
target_link_libraries(mapping ${PROJECT_NAME})

