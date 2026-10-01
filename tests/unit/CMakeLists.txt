add_executable(
    unit_tests
    EulerIntegratorTest.cpp
)

target_link_libraries(
    unit_tests
    PRIVATE
        moonlander_backend
        GTest::gtest_main
    )

include(GoogleTest)

gtest_discover_tests(unit_tests)