add_executable(
    verification_tests
    VER_TRA_001_FreeTranslation.cpp
    VER_ROT_001_AxisTorque.cpp
    VER-FRM-002_MCMFtoMSC.cpp
    VER-FRM-003_ENU.cpp
    VER-FRM-004_MCI_MCMF.cpp
    VER-FRM-005_SBF_MCI.cpp
    VER-FRM-006_LVLH.cpp
    VER-FRM-007_Robustness.cpp
    VER-FRM-001_LandingPipeline.cpp
)

target_link_libraries(
    verification_tests
    PRIVATE
        moonlander_backend
        GTest::gtest_main
    )

include(GoogleTest)

gtest_discover_tests(verification_tests)