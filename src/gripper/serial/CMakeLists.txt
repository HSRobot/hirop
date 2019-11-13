## 因时二指夹爪
FIND_PACKAGE(serial REQUIRED)
INCLUDE_DIRECTORIES(${serial_INCLUDE_DIRS})
SET(ROS_LIB_PATH /opt/ros/kinetic/lib)

LINK_DIRECTORIES(${ROS_LIB_PATH})
ADD_LIBRARY(SerialGripper serial_gripper.cpp)

TARGET_LINK_LIBRARIES(SerialGripper serial HPlugin yaml-cpp)

INSTALL(TARGETS SerialGripper DESTINATION lib)

############################ Start Simaple Generator ############################

#ADD_LIBRARY(ClassicGenerator SHARED simaple_generator.cpp)
#INSTALL(TARGETS ClassicGenerator DESTINATION lib)

############################ End Simaple Generator ############################
