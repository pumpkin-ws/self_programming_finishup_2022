project(multirobot VERSION 0.1.0)
set(AUBO_DIR ${multirobot_SOURCE_DIR}/third_party/aubo_sdk)

include_directories(${AUBO_DIR}/log4cplus/linux_x64/inc/log4cplus)
include_directories(${AUBO_DIR}/robotSDK/inc)

find_package(ur_rtde REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${multirobot_SOURCE_DIR}/lib/)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${multirobot_SOURCE_DIR}/bin/)

add_library(
    AuboDriver SHARED
    source/aubo_move_utils.cpp
    source/aubo_move_utils.hpp
    source/aubo_move.cpp
    source/aubo_move.cpp
)
target_link_libraries(AuboDriver ${log4plus_LIBS} ${auboSDK_LIBS})

add_library(
    URDriver SHARED
    source/ur_move.cpp
)
target_link_libraries(
    URDriver
    ur_rtde::rtde
)

add_executable(ur-test UR_test.cpp)
target_link_libraries(ur-test ur_rtde::rtde)

add_executable(eigen-test2 eigen-test.cpp)

add_executable(redis-test redis-test.cpp)
target_link_libraries(redis-test redis++ hiredis)

add_executable(redis-sub redis-subscribe.cpp)
target_link_libraries(redis-sub redis++ hiredis)

