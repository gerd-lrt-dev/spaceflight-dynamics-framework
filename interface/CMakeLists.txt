find_package(Qt6 REQUIRED COMPONENTS Core)

add_library(interface STATIC
    TelemetryMapper.cpp
    TelemetryMapper.h
)

target_include_directories(interface
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE
        ${CMAKE_SOURCE_DIR}/backend
        ${CMAKE_SOURCE_DIR}/ui
)

target_link_libraries(interface
    PUBLIC
        moonlander_backend
        ui
        Eigen3::Eigen
    PRIVATE
        Qt6::Core
)
