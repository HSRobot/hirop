## 经典生成器
FIND_PACKAGE(tf REQUIRED)
INCLUDE_DIRECTORIES(${tf_INCLUDE_DIRS})
SET(ROS_LIB_PATH /opt/ros/kinetic/lib)

LINK_DIRECTORIES(${ROS_LIB_PATH})
ADD_LIBRARY(ClassicGenerator classic_generator.cpp)

TARGET_LINK_LIBRARIES(ClassicGenerator tf HPlugin yaml-cpp)

INSTALL(TARGETS ClassicGenerator DESTINATION lib)

############################ Start Simaple Generator ############################

#ADD_LIBRARY(ClassicGenerator SHARED simaple_generator.cpp)
#INSTALL(TARGETS ClassicGenerator DESTINATION lib)

############################ End Simaple Generator ############################
