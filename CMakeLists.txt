cmake_minimum_required(VERSION 2.8.3)
project(pgr_camera)

find_package(catkin REQUIRED COMPONENTS message_generation dynamic_reconfigure roscpp std_msgs dynamic_reconfigure
          image_transport polled_camera camera_calibration_parsers camera_info_manager team_diana_lib)
find_package(Boost REQUIRED COMPONENTS system program_options)

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -lflycapture -std=c++11 -Wall" )

set(FLYCAPTURE_LIBRARIES "/usr/lib/libflycapture.so")
set(FLYCAPTURE_INCLUDE_PATH "/usr/include/flycapture")

include_directories(../../include/ ${catkin_INCLUDE_DIRS} ${FLYCAPTURE_INCLUDE_PATH})

generate_dynamic_reconfigure_options(
	cfg/PgrCamera.cfg
  cfg/PgrGigECamera.cfg
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   published.msg
#   Message1.msg
#   Message2.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   boolean.srv
   oneshot.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
 )

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
 INCLUDE_DIRS include
  LIBRARIES pgr_camera
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

#this is orrbly wrong!!!! erase me
include_directories(./include/  ${catkin_INCLUDE_DIRS} ${FLYCAPTURE_INCLUDE_PATH})

add_library(pgr_camera src/libflycapcam/FlycapCameraBase.cpp
                       src/libflycapcam/FlycapCameraGigE.cpp
                       src/libflycapcam/FlycapCameraManager.cpp)

add_executable(pgr_camera_node src/node/main.cpp
                              src/node/CameraNode.cpp
                              src/node/GigECameraNode.cpp)
add_dependencies(pgr_camera_node ${PROJECT_NAME}_gencfg)

target_link_libraries(
        pgr_camera_node
        pgr_camera
        ${catkin_LIBRARIES}
        ${FLYCAPTURE_LIBRARIES}
        ${Boost_LIBRARIES}
)


# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
##   PATTERN ".svn" EXCLUDE
# )

## Mark executables and/or libraries for installation
 install(TARGETS pgr_camera  pgr_camera_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


###########
## Build ##
###########

#set(CMAKE_CXX_FLAGS "-lflycapture -fpermissive")

#add_subdirectory(src)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(catkinPgrCamera
#   src/${PROJECT_NAME}/catkinPgrCamera.cpp
# )

## Declare a cpp executable
# add_executable(catkinPgrCamera_node src/catkinPgrCamera_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

## Specify libraries to link a library or executable target against
# target_link_libraries(catkinPgrCamera_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark executables and/or libraries for installation
install(TARGETS pgr_camera #pgr_camera_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_catkinPgrCamera.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
