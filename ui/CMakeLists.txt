cmake_minimum_required(VERSION 3.16)
project(MoonLanderUI)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

file(GLOB
    UI_SOURCES *.cpp
    UI_HEADERS *.h
)

qt_add_resources(UI_RESOURCES resources.qrc)

add_executable(Moonlander_UI
    ${UI_SOURCES}
    ${UI_HEADERS}
    ${UI_RESOURCES})


target_link_libraries(Moonlander_UI
    PUBLIC
        Eigen3::Eigen
    PRIVATE
        Qt6::Widgets
        moonlander_backend
)
