aux_source_directory(./ FRAMEWORK_SRCS)

find_library(HPLUGIN_LIBS HPlugin ${PROJECT_SOURCE_DIR}/libs)


IF(NOT HPLUGIN_LIBS)
MESSAGE(FATAL_ERROR "Can't no found hplugin libs")
ENDIF(NOT HPLUGIN_LIBS)

find_library(YAMLCPP6_LIBS yaml-cpp ${PROJECT_SOURCE_DIR}/libs NO_DEFAULT_PATH)

IF(NOT YAMLCPP6_LIBS)
MESSAGE(FATAL_ERROR "Can't no found yaml-cpp libs")
ENDIF(NOT YAMLCPP6_LIBS)

add_library(hirop_vision ${FRAMEWORK_SRCS} utils/py_lock_helper.cpp)

TARGET_LINK_LIBRARIES(hirop_vision python3.5m ${YAMLCPP6_LIBS}
    ${HPLUGIN_LIBS}
    ${OpenCV_LIBRARIES}
    boost_thread boost_system)

## 安装目标文件
install(TARGETS hirop_vision DESTINATION lib)
