INCLUDE_DIRECTORIES(.)

OPTION(WITH_ROS "Support get point cloud by ros topic" ON)
OPTION(WITH_PCL_FUSION "Support get point cloud by ros topic" ON)
OPTION(WITH_REGION "Support get point cloud by ros topic" ON)

IF(WITH_ROS)
        SET(SOURCES
                ${SOURCES}
                pointcloud_ros.cpp
        )
ENDIF(WITH_ROS)

IF(WITH_PCL_FUSION)
        SET(SOURCES
                ${SOURCES}
                pcl_fusion.cpp
        )
ENDIF(WITH_PCL_FUSION)

IF(WITH_REGION)
        SET(SOURCES
                ${SOURCES}
                region_filters.cpp
        )
ENDIF(WITH_REGION)


ectomodule(hirop_perception DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} INSTALL
    ${SOURCES}
)

link_ecto(hirop_perception
    ${roscpp_LIBRARIES}
    ${PCL_LIBRARIES}
)

