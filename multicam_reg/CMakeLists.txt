set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin/)

find_package(PCL 1.10 REQUIRED)

add_executable(merge_app merge_app.cpp)

