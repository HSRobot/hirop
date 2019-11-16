## 经典生成器
FIND_PACKAGE(tf REQUIRED)

FIND_PACKAGE(REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_perception
)

SET(ROS_LIB_PATH /opt/ros/kinetic/lib)

LINK_DIRECTORIES(${ROS_LIB_PATH})

INCLUDE_DIRECTORIES(${tf_INCLUDE_DIRS} /usr/include/eigen3)

ADD_LIBRARY(ClassicActuator classic_actuator.cpp)

TARGET_LINK_LIBRARIES(ClassicActuator
    tf
    moveit_planning_interface
    moveit_planning_scene_interface
    moveit_move_group_interface
    moveit_collision_detection
    moveit_kinematic_constraints
    HPlugin
    yaml-cpp)

INSTALL(TARGETS ClassicActuator DESTINATION lib)

############################ Start Simaple Generator ############################

#ADD_LIBRARY(ClassicGenerator SHARED simaple_generator.cpp)
#INSTALL(TARGETS ClassicGenerator DESTINATION lib)

############################ End Simaple Generator ############################
