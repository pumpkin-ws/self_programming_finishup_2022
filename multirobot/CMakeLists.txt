project(multirobot VERSION 0.1.0)
set(AUBO_DIR ${multirobot_SOURCE_DIR}/third_party/aubo_sdk)

include_directories(${AUBO_DIR}/log4cplus/linux_x64/inc/log4cplus)
include_directories(${AUBO_DIR}/robotSDK/inc)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${multirobot_SOURCE_DIR}/lib/)
add_library(
    AuboDriver SHARED
    source/aubo_move_utils.cpp
    source/aubo_move_utils.hpp
    source/aubo_move.cpp
    source/aubo_move.cpp
)
target_link_libraries(AuboDriver ${log4plus_LIBS} ${auboSDK_LIBS})
