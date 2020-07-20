## 因时二指夹爪
FIND_PACKAGE(serial REQUIRED)
INCLUDE_DIRECTORIES(${serial_INCLUDE_DIRS} /usr/local/include)
SET(ROS_LIB_PATH /opt/ros/kinetic/lib)

LINK_DIRECTORIES(${ROS_LIB_PATH} /usr/local/lib)

ADD_LIBRARY(SerialGripper SHARED serial_gripper.cpp)
TARGET_LINK_LIBRARIES(SerialGripper serial HPlugin yaml-cpp)

ADD_LIBRARY(fiveFingerSerialGripper SHARED serial_gripper.cpp fiveFingerSerialGripper.cpp)
TARGET_LINK_LIBRARIES(fiveFingerSerialGripper serial HPlugin yaml-cpp)

add_executable(serialGripperTest test.cpp serial_gripper.cpp fiveFingerSerialGripper.cpp)
TARGET_LINK_LIBRARIES(serialGripperTest gtest yaml-cpp serial SerialGripper hirop_gripper boost_filesystem dl)
#add_test(serialGripperTest uuid_generator COMMAND uuid_generator)

INSTALL(TARGETS SerialGripper DESTINATION lib)
INSTALL(TARGETS fiveFingerSerialGripper DESTINATION lib)

############################ Start Simaple Generator ############################

#ADD_LIBRARY(ClassicGenerator SHARED simaple_generator.cpp)
#INSTALL(TARGETS ClassicGenerator DESTINATION lib)

############################ End Simaple Generator ############################
