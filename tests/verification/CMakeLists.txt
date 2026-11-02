add_executable(
    verification_tests

    VER-TRA/VER_TRA_001_FreeTranslation.cpp

    VER-ROT/VER_ROT_001_AxisTorque.cpp
    VER-ROT/VER_ROT_002A_GyroscopicDerivative.cpp

    VER-MASS/VER_MASS_001A_FuelTankBoundaries.cpp

    VER-NUM/VER_NUM_001A_TranslationConvergence.cpp

    VER-SEN/VER_SEN_001A_GLoadReference.cpp

    VER-CTL/VER_CTL_001A_ControllerBasics.cpp

    VER-FRM/VER-FRM-001_LandingPipeline.cpp
    VER-FRM/VER-FRM-002_MCMFtoMSC.cpp
    VER-FRM/VER-FRM-003_ENU.cpp
    VER-FRM/VER-FRM-004_MCI_MCMF.cpp
    VER-FRM/VER-FRM-005_SBF_MCI.cpp
    VER-FRM/VER-FRM-006_LVLH.cpp
    VER-FRM/VER-FRM-007_Robustness.cpp

    VER-PROP/VER-PROP-001_PropulsionGeneration.cpp
)

target_link_libraries(
    verification_tests
    PRIVATE
        moonlander_backend
        GTest::gtest_main
    )

include(GoogleTest)

gtest_discover_tests(verification_tests)
