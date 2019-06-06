INCLUDE_DIRECTORIES(.)

OPTION(WITH_ROS "Support get point cloud by ros topic" ON)

IF(WITH_ROS)
	SET(SOURCES 
		${SOURCES} 
		pointcloud_ros.cpp
	)
ENDIF(WITH_ROS)

ectomodule(hirop_perception_sources DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} INSTALL
	${SOURCES} 
)
