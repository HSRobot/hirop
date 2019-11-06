aux_source_directory(./ FRAMEWORK_SRCS)

FIND_PACKAGE(HPlugin REQUIRED)

ADD_LIBRARY(hirop_gripper SHARED ${FRAMEWORK_SRCS})

TARGET_LINK_LIBRARIES(hirop_gripper ${HPlugin_LIBRARIES} ${YAML_CPP_LIBRARIES})

CONFIGURE_FILE(hirop_gripper.cmake.in ${PROJECT_BINARY_DIR}/hirop_gripperConfig.cmake @ONLY)

SET(${PICKPLACE_PLUGIN_PATH} /usr/local/lib/)

## 安装目标文件
INSTALL(TARGETS hirop_gripper
        EXPORT hirop_gripper
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include)

## 安装头文件
INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include/hirop)

## 安装cmake配置文件
INSTALL(FILES "${PROJECT_BINARY_DIR}/hirop_gripperConfig.cmake" DESTINATION lib/cmake/hirop_gripper/)
