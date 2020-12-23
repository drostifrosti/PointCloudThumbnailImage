cmake_minimum_required(VERSION 3.14)
project(PointCloudThumbnailImage)

add_executable (PointCloudThumbnailImage src/main.cpp)

find_package(PCL 1.8 REQUIRED)
if(PCL_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  link_directories(${PCL_LIBRARY_DIRS})
  target_link_libraries(PointCloudThumbnailImage ${PCL_LIBRARIES})
endif()

find_package(ApproxMVBB REQUIRED)
target_link_libraries(PointCloudThumbnailImage ApproxMVBB::Core)
