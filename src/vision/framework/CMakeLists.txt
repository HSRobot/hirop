aux_source_directory(./ FRAMEWORK_SRCS)

find_library(HPLUGIN_LIBS HPlugin ${PROJECT_SOURCE_DIR}/libs)


IF(NOT HPLUGIN_LIBS)
MESSAGE(FATAL_ERROR "Can't no found hplugin libs")
ENDIF(NOT HPLUGIN_LIBS)

add_library(hirop_vision ${FRAMEWORK_SRCS} utils/py_lock_helper.cpp)

TARGET_LINK_LIBRARIES(hirop_vision python3.5m
    ${HPLUGIN_LIBS}
    ${OpenCV_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    boost_thread boost_system boost_filesystem)

## 安装目标文件
install(TARGETS hirop_vision DESTINATION lib)
