set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)


######################################## TOOLS ##############################################

add_executable(ply2pcd ply2pcd.cpp )
target_link_libraries (ply2pcd ${PCL_LIBRARIES})

add_executable(pcd2ply pcd2ply.cpp )
target_link_libraries (pcd2ply ${PCL_LIBRARIES})

######################################## install ##############################################
INSTALL(TARGETS ply2pcd pcd2ply
RUNTIME DESTINATION bin
LIBRARY DESTINATION lib
ARCHIVE DESTINATION libstatic
)

#other files
#INSTALL(FILES files... DESTINATION <dir>
#[PERMISSIONS permissions...]
#[CONFIGURATIONS [Debug|Release|...]]
#[COMPONENT <component>]
#[RENAME <name>] [OPTIONAL])

#other script
#INSTALL(PROGRAMS files... DESTINATION <dir>
#[PERMISSIONS permissions...]
#[CONFIGURATIONS [Debug|Release|...]]
#[COMPONENT <component>]
#[RENAME <name>] [OPTIONAL])

#dirs
#INSTALL(DIRECTORY icons scripts/ DESTINATION share/myproj
#PATTERN "CVS" EXCLUDE
#PATTERN "scripts/*"
#PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READGROUP_EXECUTE GROUP_READ)