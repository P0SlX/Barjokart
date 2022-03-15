cmake_minimum_required(VERSION 3.10)

project(barjokart)

set(CMAKE_CXX_STANDARD 20)

find_package(X11 REQUIRED)
link_libraries(${X11_LIBRARIES})
include_directories(${X11_INCLUDE_DIR})
find_package(Threads)
add_executable(barjokart main.cpp CImg.h astar.cpp astar.h map.h map.cpp node.cpp node.h)
target_link_libraries(barjokart m ${CMAKE_THREAD_LIBS_INIT})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -O2")

add_custom_command(TARGET barjokart POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/input ${CMAKE_BINARY_DIR}/input
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/output
        )
