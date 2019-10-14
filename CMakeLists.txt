cmake_minimum_required(VERSION 2.8.3)
project(3dDefectDetection)

#SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_BUILD_TYPE "Release")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

include_directories(include)

#needs to be put after find_package()? yes. they depend on PCL OpenCV
#add_library() can be linked between sub_dirs? seems yes. to be confirmed.
add_subdirectory(src/tools)
add_subdirectory(src/libs)
add_subdirectory(src/demo)


##################################### build #####################################################
add_executable(normalEstimate src/normalEstimate.cpp )
target_link_libraries (normalEstimate ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} config pointcloud_helper)

add_executable(normalEstimate_pcl_website src/normalEstimate_pcl_website.cpp )
target_link_libraries (normalEstimate_pcl_website ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(fit_plane_from_file src/fit_plane_from_file.cpp )
target_link_libraries (fit_plane_from_file ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} config pointcloud_helper)


