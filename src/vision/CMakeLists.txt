
## 使用PYthon3版本
FIND_PACKAGE(Python 3 COMPONENTS Development)

## 使用ORK的渲染工具
FIND_PACKAGE(object_recognition_renderer REQUIRED)

## 使用opnecv
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(opencv_candidate REQUIRED)
FIND_PACKAGE(yaml-cpp REQUIRED)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${object_recognition_renderer_INCLUDE_DIRS})

IF(${Python_FOUND})
	INCLUDE_DIRECTORIES(${Python_INCLUDE_DIRS})
ELSEIF(${PYTHON_INCLUDE_PATH})
	INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
ELSE()
	INCLUDE_DIRECTORIES(/usr/include/python3.5/)
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
add_subdirectory(trainer)


