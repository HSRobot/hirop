##　IO控制夹爪驱动
add_definitions(-D_LINUX_)

##导入华数三型头文件
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/3rds/include/hsc3/")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/3rds/include/")

FIND_LIBRARY(HSC3_LIBS Hsc3Api "${PROJECT_SOURCE_DIR}/3rds/libs/hsc3/hsc3libs" NO_DEFAULT_PATH)
if(NOT HSC3_LIBS)
    MESSAGE(FATAL_ERROR "Can't No found HSC3 libs")
endif()

LINK_DIRECTORIES(${HSC3_LIBS})

ADD_LIBRARY(RobotIOGripper robotIO_gripper.cpp)

TARGET_LINK_LIBRARIES(RobotIOGripper HPlugin yaml-cpp ${HSC3_LIBS})

## 安装HSC3头文件
INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/3rds/include/ DESTINATION include/)

## 安装HSC3库文件
INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/3rds/libs/hsc3/hsc3libs/ DESTINATION lib/)

## 安装库
INSTALL(TARGETS RobotIOGripper DESTINATION lib)

############################ Start Simaple Generator ############################

#ADD_LIBRARY(ClassicGenerator SHARED simaple_generator.cpp)
#INSTALL(TARGETS ClassicGenerator DESTINATION lib)

############################ End Simaple Generator ############################
