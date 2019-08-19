## 使用ORK的渲染工具
FIND_PACKAGE(object_recognition_renderer REQUIRED)
INCLUDE_DIRECTORIES(${object_recognition_renderer_INCLUDE_DIRS})
ADD_LIBRARY(LinemodDetector linemod_detector.cpp)

TARGET_LINK_LIBRARIES(LinemodDetector
    ${object_recognition_renderer_LIBRARIES}
    ${opencv_candidate_LIBRARIES})

INSTALL(TARGETS LinemodDetector DESTINATION lib)

############################ Start Simaple Detector ############################

ADD_LIBRARY(SimapleDetector SHARED simaple_detector.cpp)
INSTALL(TARGETS LinemodDetector DESTINATION lib)

############################ End Simaple Detector ############################
