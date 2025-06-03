cmake_minimum_required(VERSION 3.10)
project(boundary_tracing)

find_package(OpenCV REQUIRED)

add_executable(main main.cpp graham_scan.cpp)
target_link_libraries(main ${OpenCV_LIBS})
target_include_directories(main PRIVATE ${OpenCV_INCLUDE_DIRS})
