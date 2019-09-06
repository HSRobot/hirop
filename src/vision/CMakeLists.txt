## 使用opnecv
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(opencv_candidate REQUIRED)
FIND_PACKAGE(yaml-cpp REQUIRED)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})


OPTION(WITH_PYTHON3 "USE PYTHON3 " ON)

# 使用PYthon3版本
IF(WITH_PYTHON3)
    SET(PYTHON_PATH /usr/include/python3.5/)
    ADD_DEFINITIONS(-DWITH_PYTHON3)
ELSE()
    SET(PYTHON_PATH /usr/include/python2.7/)
ENDIF()


IF(${Python_FOUND})
	INCLUDE_DIRECTORIES(${Python_INCLUDE_DIRS})
ELSEIF(${PYTHON_INCLUDE_PATH})
	INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
ELSE()
        INCLUDE_DIRECTORIES(${PYTHON_PATH})
ENDIF()

## 支持C++11 因为Yaml-cpp使用了c++11
add_definitions(-std=c++11)

## 核心框架代码
add_subdirectory(framework)
##TOD
#add_subdirectory(TODDetector)
## 检测器代码
add_subdirectory(detector)
## 训练器代码
#add_subdirectory(trainer)


