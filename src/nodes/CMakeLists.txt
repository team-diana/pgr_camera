# make sure configure headers are built before any node using them
# ----------

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs dynamic_reconfigure image_transport polled_camera camera_calibration_parsers)
find_package(Boost REQUIRED COMPONENTS system program_options)

#this is orrbly wrong!!!! erase me
set(FLYCAPTURE_LIBRARIES "/usr/lib/libflycapture.so")
set(FLYCAPTURE_INCLUDE_PATH "/usr/include/flycapture")

include_directories(../../include/ ${catkin_INCLUDE_DIRS} ${FLYCAPTURE_INCLUDE_PATH})

add_executable(pgr_camera_node pgr_camera_node.cpp)
add_dependencies(pgr_camera_node ${PROJECT_NAME}_gencfg)
message("ASDFASDFADFS" ${Boost_LIBRARIES})
target_link_libraries(
	pgr_camera_node
	pgr_camera
	${catkin_LIBRARIES}
	${FLYCAPTURE_LIBRARIES}
)
