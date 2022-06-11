cmake_minimum_required(VERSION 3.16)
project(Minimum_bounding_box)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_CXX_FLAGS "-std=c++17 -stdlib=libc++ -O1")

if(APPLE)
        find_package(GLEW REQUIRED)
        find_package(glfw3 REQUIRED)
        add_compile_definitions(GL_SILENCE_DEPRECATION)
endif()

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

if(APPLE) 
        target_link_libraries(Minimum_bounding_box GLEW::GLEW glfw)
elseif(UNIX)
        target_link_libraries(Minimum_bounding_box -lGL -lGLEW -lglfw)
endif()