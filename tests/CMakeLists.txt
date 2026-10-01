include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.17.0
)

FetchContent_MakeAvailable(googletest)

add_subdirectory(unit)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/integration/CMakeLists.txt")
    add_subdirectory(integration)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/verification/CMakeLists.txt")
    add_subdirectory(verification)
endif()