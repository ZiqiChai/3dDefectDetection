set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

######################################## DEMO ##############################################
add_executable(normal_distributions_transform ./normal_distributions_transform.cpp )
target_link_libraries (normal_distributions_transform ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable (iterative_closest_point ./iterative_closest_point.cpp)
target_link_libraries (iterative_closest_point ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
