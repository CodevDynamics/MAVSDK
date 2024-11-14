cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(camera_r3)

add_executable(camera_r3
    camera_r3.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(camera_r3
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(camera_r3 PRIVATE -Wall -Wextra)
else()
    add_compile_options(camera_r3 PRIVATE -WX -W2)
endif()
