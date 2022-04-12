set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin/)

find_package(OpenCV 4 REQUIRED)

find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(source)

add_executable(merge_app merge_app.cpp)
target_link_libraries(merge_app ${PCL_LIBRARIES})

add_executable(calib_cam calibrate_cam.cpp)
target_link_libraries(calib_cam ${OpenCV_LIBS} glog::glog)

add_executable(acquire_img acquire_image.cpp)
target_link_libraries(acquire_img ${OpenCV_LIBS} pthread rs_driver)

