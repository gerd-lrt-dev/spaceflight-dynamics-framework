add_executable(
    verification_tests
    VER_TRA_001_FreeTranslation.cpp
)

target_link_libraries(
    verification_tests
    PRIVATE
        moonlander_backend
        GTest::gtest_main
    )

include(GoogleTest)

gtest_discover_tests(verification_tests)