find_package(Qt6 REQUIRED COMPONENTS Core)

add_library(interface STATIC
    TelemetryMapper.cpp
    TelemetryMapper.h
)

target_include_directories(interface
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE
        ${CMAKE_SOURCE_DIR}/backend/include
)

target_link_libraries(interface
    PUBLIC
        Eigen3::Eigen
        Qt6::Core
    PRIVATE
        moonlander_backend
)
