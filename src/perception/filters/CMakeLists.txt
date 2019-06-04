set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/hirop_perecption)

INCLUDE_DIRECTORIES(. ${PCL_INCLUDE_DIRS})

ectomodule(hirop_perception DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} INSTALL
	region_filters.cpp
        pcl_fusion.cpp
)
