cmake_minimum_required(VERSION 3.0.0)
project(sp-finish VERSION 0.1.0)

include(CTest)
enable_testing()

find_package(glog REQUIRED)

link_directories(${PROJECT_SOURCE_DIR}/lib/)

add_subdirectory(multicam_reg)

add_subdirectory(multirobot)

add_executable(sp-finish main.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
