## 寻找源码文件
aux_source_directory(./ ASR_SRCS)
aux_source_directory(./jsoncpp JSON_CPP_SRCS)

## 定义动态库
add_library(hirop_asr SHARED ${ASR_SRCS} ${JSON_CPP_SRCS})

#########################start AIUI configure################################

## 需要关闭CXX11的ABI，否则运行时科大讯飞的AIUI会报错
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)

## 指定自定义AIUI的路径
set(AIUI_PATH "/home/maizitian/catkin_ws/Linux_aiui5.5.1059.0000_5e6c72df/")

## 指定自定义AIUI的APPID
set(AIUI_APPID "5e6c72df")

## 指定AIUI和msc文件夹所在的路径
set(AIUI_CONFIG_PATH "${AIUI_PATH}/samples/aiui_sample/build")

## 生成相关头文件
configure_file (
        "aiui_config.cmake.in"
        "${PROJECT_BINARY_DIR}/aiui_config.h"
)

## 导入AIUI相关的头文件路径
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/3rds/include")

# 使得工程可以找到通过cmake生成的头文件 aiui_config.h
INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})

# 寻找AIUI的库
FIND_LIBRARY(AIUI_LIBS aiui "${PROJECT_SOURCE_DIR}/3rds/libs" NO_DEFAULT_PATH)

if(NOT AIUI_LIBS)
    MESSAGE(FATAL_ERROR "Can't No found AIUI libs")
endif()

#########################end AIUI configure################################

## 定义target
TARGET_LINK_LIBRARIES(hirop_asr ${AIUI_LIBS})

## 生成Cmake配置文件
CONFIGURE_FILE(hirop_asr.cmake.in ${PROJECT_BINARY_DIR}/hirop_asrConfig.cmake @ONLY)

## 安装cmake配置文件
INSTALL(FILES "${PROJECT_BINARY_DIR}/hirop_asrConfig.cmake" DESTINATION lib/cmake/hirop_asr/)

## 安装AIUI头文件
INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/3rds/include/ DESTINATION include/)

## 安装AIUI库文件
INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/3rds/libs/ DESTINATION lib/)

## 安装库文件和头文件
INSTALL(TARGETS hirop_asr
        EXPORT hirop_asr
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include)
