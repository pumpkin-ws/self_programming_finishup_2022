set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin/)

find_package(OpenCV 4 REQUIRED)

find_package(Boost REQUIRED filesystem)

find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(glog REQUIRED)

add_subdirectory(source)

add_executable(merge_app merge_app.cpp)
target_link_libraries(merge_app ${OpenCV_LIBS} pthread rs_driver ${Boost_LIBRARIES} calib_intrinsic file_manip ${PCL_LIBRARIES})

add_executable(calib_cam calibrate_cam.cpp)
target_link_libraries(calib_cam ${OpenCV_LIBS} glog::glog)

add_executable(acquire_img acquire_image.cpp)
target_link_libraries(acquire_img ${OpenCV_LIBS} pthread rs_driver ${Boost_LIBRARIES})

add_executable(multicam_calib multicam_calib.cpp)
target_link_libraries(multicam_calib PUBLIC ${OpenCV_LIBS} pthread rs_driver ${Boost_LIBRARIES} calib_intrinsic file_manip)

find_package(Eigen3 REQUIRED)
add_executable(eigen-test eigen-test.cpp)
target_include_directories(eigen-test PUBLIC ${EIGEN3_INCLUDE_DIRS})

add_executable(thread-test thread-test.cpp)
target_link_libraries(thread-test pthread)

add_executable(rs-test rs-driver-test.cpp)
target_link_libraries(rs-test rs_driver ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(glog-test glog-test.cpp)
target_link_libraries(glog-test glog::glog ${Boost_LIBRARIES})


