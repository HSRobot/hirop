##　hsImpenderr
add_definitions(-D_LINUX_)
SET(PROJ_NAME hsImpenderrForce)

aux_source_directory(./ FRAMEWORK_SRCS)
INCLUDE_DIRECTORIES(./)

ADD_LIBRARY(${PROJ_NAME} ${FRAMEWORK_SRCS})

TARGET_LINK_LIBRARIES(${PROJ_NAME}  HPlugin yaml-cpp)

INSTALL(TARGETS ${PROJ_NAME} DESTINATION lib)


