## 使用ORK的渲染工具
FIND_PACKAGE(object_recognition_renderer REQUIRED)
INCLUDE_DIRECTORIES(${object_recognition_renderer_INCLUDE_DIRS})

ADD_LIBRARY(LinemodTrainer linemod_trainer.cpp)

TARGET_LINK_LIBRARIES(LinemodTrainer
    ${opencv_candidate_LIBRARIES}
    ${object_recognition_renderer_LIBRARIES}
    boost_filesystem)

INSTALL(TARGETS LinemodTrainer DESTINATION lib)
