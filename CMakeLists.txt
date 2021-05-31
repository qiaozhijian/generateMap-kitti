cmake_minimum_required(VERSION 3.5)
project(generate_kitti)

# 一定要加-march=native， 不然使用pcl的一些函数会报错
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")
set(CMAKE_CXX_STANDARD 17)
#find_package(OpenMP)
#if (OPENMP_FOUND)
#    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#endif ()

set(ALL_TARGET_LIBRARIES "")
include(cmake/eigen.cmake)
include(cmake/PCL.cmake)
include(cmake/boost.cmake)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME}
        src/loadData.cpp
        )
target_link_libraries(${PROJECT_NAME}
        ${ALL_TARGET_LIBRARIES}
        )

# visual_kitti
add_executable(generate_map node/generate_map.cpp)
target_link_libraries(generate_map ${PROJECT_NAME})

