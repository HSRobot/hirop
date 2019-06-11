INCLUDE_DIRECTORIES(/usr/include/python3.5)

## 插件化模块的安装路径
set(HPLUGIN_DIR /home/eima/Work/out)

INCLUDE_DIRECTORIES(${HPLUGIN_DIR}/include)

## 支持C++11 因为Yaml-cpp使用了c++11
add_definitions(-std=c++11)

## 核心框架代码
add_subdirectory(framework)

## 检测器代码
add_subdirectory(detector)

## 训练器代码
add_subdirectory(trainer)


