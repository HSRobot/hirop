## 支持C++11 因为Yaml-cpp使用了c++11
add_definitions(-std=c++11)

add_executable(train_test train_test.cpp)

target_link_libraries(train_test
    hirop_vision)

install(TARGETS train_test DESTINATION bin)
