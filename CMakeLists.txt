cmake_minimum_required(VERSION 3.10)

project(barjokart)

set(CMAKE_CXX_STANDARD 20)

find_package(X11 REQUIRED)
link_libraries(${X11_LIBRARIES})
include_directories(${X11_INCLUDE_DIR})
find_package(Threads)
add_executable(barjokart main.cpp CImg.h astar.cpp astar.h)
target_link_libraries(barjokart m ${CMAKE_THREAD_LIBS_INIT})