cmake_minimum_required(VERSION 3.5)

set(TOD_name TODDetector)
##########################
set(ABSL_DIR /home/fshs/KongWork/TOD_install/tensorflow/downloads/absl) #下载目录
set(Protobuf_DIR /home/fshs/KongWork/TOD_install/protobuf-3.6.0/install)  #3.6.0
set(tesnsorflow_DIRS /home/fshs/KongWork/TOD_install/tensorflow) #tensorflow本身的目录
find_package(VTK 6.2 REQUIRED)
add_definitions(-std=c++11)
include_directories(${TOD_name}
    ${tesnsorflow_DIRS}/include
    ${Eigen_include_DIR} 
    ${ABSL_DIR}     
    ${Protobuf_DIR}/include
    ${tesnsorflow_DIRS}/tensorflowCCVisionLib
	${tensorflow_DIRS}/tensorflow/include
    ${hirop_vision_INCLUDE_DIRS}
    .
	/home/fshs/work/hirop/include/vision
	/home/fshs/work/hirop/include/utils
	${VTK_INCLUDE_DIRS}
	
)
link_directories(${TOD_name}
    ${Eigen_lib_DIR} 
    ${Protobuf_DIR}/lib
    ${tesnsorflow_DIRS}/	
	#${tesnsorflow_DIRS}/tensorflowCCVisionLib/build
	#${tesnsorflow_DIRS}/linemod_before/build/install
	${VTK_LIBRARY_DIRS}
	/home/fshs/work/hirop/build/install/lib
)


add_definitions(-std=c++11)

add_library(${TOD_name} SHARED TODDetector.cpp linemod_detector_Tool.cpp)

TARGET_LINK_LIBRARIES(${TOD_name}
	${object_recognition_renderer_LIBRARIES}
    ${opencv_candidate_LIBRARIES}
    -ltensorflowCCVisionLib
	${VTK_LIBRARIES}
    ${hirop_vision_LIBRARIES}
	#-lLinemodDetectorTool
)

INSTALL(TARGETS ${TOD_name} DESTINATION lib)
