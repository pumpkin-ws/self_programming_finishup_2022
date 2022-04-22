cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${sp-finish_SOURCE_DIR}/lib/)

find_package(Boost REQUIRED filesystem)

add_library(
    file_manip SHARED
    file_manip.cpp
    file_manip.hpp
)
target_link_libraries(
    file_manip PUBLIC ${Boost_LIBRARIES}
)

add_library(
    calib_intrinsic SHARED
    calib_intrinsics.cpp
    calib_intrinsics.hpp
)
target_link_libraries(
    calib_intrinsic PUBLIC ${OpenCV_LIBS} matrix_util
)

add_library(
    matrix_util SHARED 
    matrix_util.cpp
    matrix_util.hpp
)
target_link_libraries(
    matrix_util PUBLIC ${OpenCV_LIBS}
)

add_library(
    rs_driver SHARED
    camera_base.hpp
    realsense_driver.hpp
    realsense_driver.cpp
)
target_link_libraries(
    rs_driver PUBLIC realsense2 ${OpenCV_LIBS} ${PCL_LIBRARIES} pthread
)
