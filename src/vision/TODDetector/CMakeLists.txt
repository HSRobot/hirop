cmake_minimum_required(VERSION 3.5)

set(TOD_name TODDetector)
##########################
set(ABSL_DIR /home/fshs/KongWork/TOD_install/tensorflow/downloads/absl) #下载目录
set(Protobuf_DIR /home/fshs/KongWork/TOD_install/protobuf-3.6.0/install)  #3.6.0
set(tesnsorflow_DIRS /home/fshs/KongWork/TOD_install/tensorflow) #tensorflow本身的目录

include_directories(${TOD_name}
    ${tesnsorflow_DIRS}/include
    ${Eigen_include_DIR} 
    ${ABSL_DIR}     
    ${Protobuf_DIR}/include
    /home/fshs/KongWork/tensorflow_CC_APIDemo/tensorflowCCVisionLib
#    /home/fshs/KongWork/TOD_install/tensorflow/include
    ${hirop_vision_INCLUDE_DIRS}
    .
    /home/fshs/KongWork/tensorflow_CC_APIDemo/linemod_before ##解藕的linemod
)
link_directories(${TOD_name}
    ${Eigen_lib_DIR} 
    ${Protobuf_DIR}/lib
    ${tesnsorflow_DIRS}/tensorflow_cc_sse/lib
    /home/fshs/KongWork/tensorflow_CC_APIDemo/tensorflowCCVisionLib/build
    /home/fshs/KongWork/tensorflow_CC_APIDemo/linemod_before/build/install
)


add_definitions(-std=c++11)

add_library(${TOD_name} SHARED TODDetector.cpp)

TARGET_LINK_LIBRARIES(${TOD_name}
    ${opencv_candidate_LIBRARIES}
    -ltensorflowCCVisionLib
    ${hirop_vision_LIBRARIES}
    -lLinemodDetectorTool
)

INSTALL(TARGETS ${TOD_name} DESTINATION lib)
