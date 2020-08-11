FIND_PACKAGE(yaml-cpp REQUIRED)

## 支持C++11 因为Yaml-cpp使用了c++11
add_definitions(-std=c++11)

## 核心框架代码
add_subdirectory(framework)
## 算法实现代码
add_subdirectory(hsImpenderr)

