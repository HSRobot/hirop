## 寻找源码文件
aux_source_directory(./ DATAMANAGER_SRCS)

INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR}/include)

## 使用Boost的对象序列化功能
FIND_PACKAGE(Boost COMPONENTS serialization filesystem REQUIRED)

## 定义动态库
add_library(hirop_datamanager SHARED ${DATAMANAGER_SRCS})

## 生成Cmake配置文件
CONFIGURE_FILE(hirop_datamanager.cmake.in ${PROJECT_BINARY_DIR}/hirop_datamanagerConfig.cmake @ONLY)

## 安装cmake配置文件
INSTALL(FILES "${PROJECT_BINARY_DIR}/hirop_datamanagerConfig.cmake" DESTINATION lib/cmake/hirop_datamanager/)

## 定义target的依赖项
TARGET_LINK_LIBRARIES(hirop_datamanager ${Boost_LIBRARIES})

## 安装库文件和头文件
INSTALL(TARGETS hirop_datamanager
        EXPORT hirop_datamanager
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include)
