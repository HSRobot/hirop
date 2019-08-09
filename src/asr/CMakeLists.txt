aux_source_directory(./ ASR_SRCS)

aux_source_directory(./jsoncpp JSON_CPP_SRCS)

## 需要关闭CXX11的ABI，否则运行时科大讯飞的AIUI会报错
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)

set(AIUI_PATH "/home/fshs/work/Linux_awaken_exp/")

set(AIUI_APPID "5a950e5a")
set(AIUI_CONFIG_PATH "${AIUI_PATH}/samples/aiui_sample/build")

configure_file (
        "aiui_config.cmake.in"
        "${PROJECT_BINARY_DIR}/aiui_config.h"
)

add_library(hirop_asr SHARED ${ASR_SRCS} ${JSON_CPP_SRCS})

INCLUDE_DIRECTORIES(${AIUI_PATH}/include ${PROJECT_BINARY_DIR})

TARGET_LINK_LIBRARIES(hirop_asr ${AIUI_PATH}/libs/x64/libaiui.so)

INSTALL(TARGETS hirop_asr
        EXPORT hirop_asr
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include)
