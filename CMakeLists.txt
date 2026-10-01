cmake_minimum_required(VERSION 3.16)
project(MoonLanderProject)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

include(CTest)

add_subdirectory(backend)
add_subdirectory(ui)
add_subdirectory(interface)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()