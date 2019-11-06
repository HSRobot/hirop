##　气动夹爪驱动
FIND_PACKAGE(hirop_msgs REQUIRED)
INCLUDE_DIRECTORIES(${hirop_msgs_INCLUDE_DIRS})
SET(ROS_LIB_PATH /opt/ros/kinetic/lib)

LINK_DIRECTORIES(${ROS_LIB_PATH})
ADD_LIBRARY(AirpowereGripper airpowere_gripper.cpp)

TARGET_LINK_LIBRARIES(AirpowereGripper tf HPlugin yaml-cpp)

INSTALL(TARGETS AirpowereGripper DESTINATION lib)

############################ Start Simaple Generator ############################

#ADD_LIBRARY(ClassicGenerator SHARED simaple_generator.cpp)
#INSTALL(TARGETS ClassicGenerator DESTINATION lib)

############################ End Simaple Generator ############################
