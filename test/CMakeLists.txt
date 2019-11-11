# 支持C++11 因为Yaml-cpp使用了c++11

FIND_PACKAGE(PCL REQUIRED)


add_definitions(-std=c++11)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
add_executable(detection_test detection_test.cpp)
target_link_libraries(detection_test
    hirop_vision ${PCL_LIBRARIES})
install(TARGETS detection_test DESTINATION bin)


#add_executable(train_test train_test.cpp)
#target_link_libraries(train_test
#    hirop_vision)


#install(TARGETS train_test DESTINATION bin)


#find_package(catkin REQUIRED COMPONENTS
#  roscpp
#  nav_msgs)

#include_directories(${catkin_INCLUDE_DIRS})

#add_executable(nav_test navigation_test.cpp)

#target_link_libraries(nav_test
#    hirop_nav)

#install(TARGETS nav_test DESTINATION bin)

#INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/3rds/include")

#add_executable(asr_test asr_test.cpp)

#target_link_libraries(asr_test
#    hirop_asr)

#install(TARGETS asr_test DESTINATION bin)

############################## Start Data Manager Test #############################

#add_executable(dm_test dm_test.cpp)

#target_link_libraries(dm_test
#    hirop_datamanager)

#install(TARGETS dm_test DESTINATION bin)

############################# End Data Manager Test #############################
