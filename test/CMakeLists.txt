## 支持C++11 因为Yaml-cpp使用了c++11
add_definitions(-std=c++11)

add_executable(train_test train_test.cpp)
add_executable(detection_test detection_test.cpp)
target_link_libraries(train_test
    hirop_vision)
target_link_libraries(detection_test
    hirop_vision)

install(TARGETS train_test DESTINATION bin)
install(TARGETS detection_test DESTINATION bin)
