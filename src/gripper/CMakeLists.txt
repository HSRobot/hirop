FIND_PACKAGE(yaml-cpp REQUIRED)

#debug print
add_definitions(-D_PRINT_)

## 支持C++11 因为Yaml-cpp使用了c++11
add_definitions(-std=c++11)

#debug print
add_definitions(-D_PRINT_)
#add_definitions(-DDEBUG)

## 核心框架代码
add_subdirectory(framework)
## 生成器代码
add_subdirectory(inspire)
## 执行器代码
add_subdirectory(airpowere)
