cmake_minimum_required(VERSION 3.10)
project(barjokart)

set(CMAKE_CXX_STANDARD 20)


# Change pas l'ordre
find_package(X11 REQUIRED)
link_libraries(${X11_LIBRARIES})
include_directories(${X11_INCLUDE_DIR})
find_package(Threads)
add_executable(barjokart main.cpp CImg.h astar.cpp astar.h map.h map.cpp node.cpp node.h)
target_link_libraries(barjokart m ${CMAKE_THREAD_LIBS_INIT})

# Retire tout les png du build-debug
add_custom_command(TARGET barjokart POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_SOURCE_DIR}/*.png
        ${CMAKE_BINARY_DIR}
        )

# Et recopie les png dans build-debug
# Pour être sur que c'est les mêmes que dans le root du projet
add_custom_command(TARGET barjokart POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_SOURCE_DIR}/*.png
        ${CMAKE_BINARY_DIR}
)

# Désactive les options de compilation pour avoir un meilleur profilage
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -O0")