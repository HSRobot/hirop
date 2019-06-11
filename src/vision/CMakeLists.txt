INCLUDE_DIRECTORIES(/usr/include/python3.5)

## 插件化模块的安装路径
set(HPLUGIN_DIR /home/fshs/work/hirop_vision/HPlugin/)
INCLUDE_DIRECTORIES(${HPLUGIN_DIR}/include)

## 使用ORK的渲染工具
FIND_PACKAGE(object_recognition_renderer REQUIRED)
INCLUDE_DIRECTORIES(${object_recognition_renderer_INCLUDE_DIRS})

## 支持C++11 因为Yaml-cpp使用了c++11
add_definitions(-std=c++11)

## 核心框架代码
add_subdirectory(framework)

## 检测器代码
add_subdirectory(detector)

## 训练器代码
add_subdirectory(trainer)


