set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

######################################## TOOLS ##############################################

find_library(RUNTIME_LIB config)
message(${RUNTIME_LIB})

add_library(config SHARED config.cc)
add_library(Utility SHARED Utility.cc)
add_library(pointcloud_helper SHARED pointcloud_helper.cc)


######################################## install ##############################################
INSTALL(TARGETS config Utility
RUNTIME DESTINATION bin
LIBRARY DESTINATION lib
ARCHIVE DESTINATION libstatic
)
