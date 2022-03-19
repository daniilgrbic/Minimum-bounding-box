cmake_minimum_required(VERSION 3.16)
project(Minimum_bounding_box)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(
        Minimum_bounding_box main.cpp
        utility/loadshaders.h
        utility/loadshaders.cpp
        utility/loadobj.h
        utility/loadobj.cpp
        utility/controls.h
        utility/controls.cpp
        convhull_3d.h
        mbb.h
        mbb.cpp)

target_link_libraries(Minimum_bounding_box -lGLEW -lglfw -lGL -O1)
