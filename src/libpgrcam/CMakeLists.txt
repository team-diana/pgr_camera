###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES pgr_camera
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


#this is orrbly wrong!!!! erase me
include_directories(../../include/)
# ----------
add_library(pgr_camera pgr_camera.cpp PgrCameraFactory.cpp)

message("CATKIN INSTALL ---- " ${CATKIN_PACKAGE_LIB_DESTINATION})
message("cmake INSTALL ---- " ${CMAKE_INSTALL_PREFIX})
## Mark executables and/or libraries for installation
 install(TARGETS pgr_camera
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION
)

