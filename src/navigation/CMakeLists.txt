aux_source_directory(./ NAVIGATION_SRCS)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  nav_msgs
  actionlib_msgs)

include_directories(${catkin_INCLUDE_DIRS})


add_library(hirop_nav ${NAVIGATION_SRCS})

TARGET_LINK_LIBRARIES(hirop_nav
    ${catkin_LIBRARIES}
    boost_thread boost_system boost_filesystem)

## 安装目标文件
INSTALL(TARGETS hirop_nav
        EXPORT hirop_nav
        LIBRARY DESTINATION lib)
