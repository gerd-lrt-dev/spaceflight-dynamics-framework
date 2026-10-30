#include <gtest/gtest.h>
#include <vector>

#include "Thrust.h"
#include "Thrust/EngineConfig.h"
#include "Thrust/RCSConfig.h"
#include "Thrust/FueltankStruct.h"

TEST(VER_PROP_001_PropulsionGeneration, MatchesAnalyticalSolution)
{
    // Configure Main Engine
    const EngineConfig mainEngineCfg = EngineConfig::Create(
        true,                       // activated
        0,                          // id
        "Test Main Engine",
        EngineType::MainEngine,
        0,                          // tankID
        300.0,                      // Isp [s]
        0.5,                        // timeConstant tau [s]
        8.0,                        // responseRate [Hz]
        1000.0,                     // maxThrust [N]
        {0.0, 0.0, -1.0},           // SBF thrust direction
        {0.0, 0.0, 0.0}             // engine position
        );

    std::vector<EngineConfig> mainEngines;

    mainEngines.push_back(mainEngineCfg);

    // Configure RCS Engines

    std::vector<RCSEngineConfig> RCSEngines{};

    // Configure Fuel Tanks
    const FuelTank tank{
        0,
        "Test Tank",
        "MainEngine",
        100.0,
        100.0
    };
    std::vector<FuelTank> tanks;

    tanks.push_back(tank);

    // Propulsion System initializing

    Thrust testPropulsion;

    testPropulsion.initializeEngines(mainEngines, RCSEngines, tanks);

    // Initialize test parameters
    double dt{0.05};

    double resultThrustTest1{0.0};
    double resultThrustTest2{0.0};

    // Run Simulation Loop
    for (int i = 1; i <= 1000; ++i)
    {
        testPropulsion.setTargetThrustInNewton(EngineType::MainEngine, 1000.0);

        testPropulsion.updatePropulsion(dt);

        if (i == 10)
        {
            resultThrustTest1 = testPropulsion.getCurrentThrustOfOneEngine(EngineType::MainEngine).norm();
        }

        if (i == 50)
        {
            resultThrustTest2 = testPropulsion.getCurrentThrustOfOneEngine(EngineType::MainEngine).norm();
        }

    }

    // Analytical references
    const double expectedThrustTest1 = 632.1205588285577;
    const double expectedThrustTest2 = 993.2620530009145;

    // Tolerances
    constexpr double thrustTolerance = 1e-8;

    // Verification
    EXPECT_NEAR(resultThrustTest1, expectedThrustTest1, thrustTolerance) << "Thrust magnitude of test 1 deviates from analytical references.";
    EXPECT_NEAR(resultThrustTest2, expectedThrustTest2, thrustTolerance) << "Thrust magnitude of test 2 deviates from analytical references.";
}
