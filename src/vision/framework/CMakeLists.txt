aux_source_directory(./ FRAMEWORK_SRCS)

FIND_PACKAGE(HPlugin REQUIRED)

ADD_LIBRARY(hirop_vision SHARED ${FRAMEWORK_SRCS} utils/py_lock_helper.cpp)


IF(WITH_PYTHON3)
    SET(PYTHON_LIB python3.5m)
ELSE()
    SET(PYTHON_LIB python2.7)
ENDIF()

TARGET_LINK_LIBRARIES(hirop_vision ${PYTHON_LIB}
    ${HPlugin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${PCL_LIBRARIES}
    boost_thread boost_system boost_filesystem)

CONFIGURE_FILE(hirop_vision.cmake.in ${PROJECT_BINARY_DIR}/hirop_visionConfig.cmake @ONLY)

## 安装目标文件
INSTALL(TARGETS hirop_vision
        EXPORT hirop_vision
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include)

## 安装头文件
INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include/hirop)

## 安装cmake配置文件
INSTALL(FILES "${PROJECT_BINARY_DIR}/hirop_visionConfig.cmake" DESTINATION lib/cmake/hirop_vision/)
