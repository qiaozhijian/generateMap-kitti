find_package(PCL REQUIRED)
message(PCL_INCLUDE_DIRS ${PCL_INCLUDE_DIRS})
message(PCL_LIBRARIES ${PCL_LIBRARIES})
link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES ${PCL_LIBRARIES})