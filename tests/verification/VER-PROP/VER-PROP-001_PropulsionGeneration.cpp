#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "Thrust.h"
#include "Thrust/EngineConfig.h"
#include "Thrust/RCSConfig.h"
#include "Thrust/FueltankStruct.h"

namespace
{
constexpr double kDt = 0.05;
constexpr double kMainTau = 0.5;
constexpr double kMainTargetThrust = 1000.0;

constexpr double kRCSTauOn = 0.05;
constexpr double kRCSMaxThrust = 40.0;

constexpr double kForceTolerance = 1e-8;
constexpr double kTorqueTolerance = 1e-8;
constexpr double kZeroTolerance = 1e-10;

FuelTank makeTestTank()
{
    return FuelTank{
        0,
        "Test Tank",
        "Propulsion",
        100.0,
        100.0
    };
}

RCSEngineConfig makeRCS(
    int id,
    const std::string& name,
    EngineType type,
    const std::string& axis,
    const Eigen::Vector3d& direction,
    const Eigen::Vector3d& position)
{
    return RCSEngineConfig::Create(
        true,
        id,
        name,
        type,
        axis,
        0,                  // tankID
        285.0,              // Isp [s]
        kRCSMaxThrust,      // maxThrust [N]
        0.0,                // commandDelay [s] -- isolated force/torque verification
        kRCSTauOn,          // tauOn [s]
        0.05,               // tauOff [s]
        0.0,                // minimumPulseWidth [s]
        direction,
        position
    );
}

double expectedFirstOrderThrust(
    double targetThrust,
    double elapsedTime,
    double tau)
{
    return targetThrust * (1.0 - std::exp(-elapsedTime / tau));
}

void expectVectorNear(
    const Eigen::Vector3d& actual,
    const Eigen::Vector3d& expected,
    double tolerance)
{
    EXPECT_NEAR(actual.x(), expected.x(), tolerance);
    EXPECT_NEAR(actual.y(), expected.y(), tolerance);
    EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}
} // namespace

TEST(VER_PROP_001_PropulsionGeneration, MainEngineMatchesAnalyticalMagnitudeDirectionAndZeroTorque)
{
    const EngineConfig mainEngineCfg = EngineConfig::Create(
        true,
        0,
        "Test Main Engine",
        EngineType::MainEngine,
        0,
        300.0,
        kMainTau,
        8.0,
        kMainTargetThrust,
        {0.0, 0.0, -1.0},
        {0.0, 0.0, 0.0}
    );

    std::vector<EngineConfig> mainEngines{mainEngineCfg};
    std::vector<RCSEngineConfig> rcsEngines{};
    std::vector<FuelTank> tanks{makeTestTank()};

    Thrust propulsion;
    propulsion.initializeEngines(mainEngines, rcsEngines, tanks);
    propulsion.setTargetThrustInNewton(
        EngineType::MainEngine,
        kMainTargetThrust
    );

    double thrustAfterOneTau{0.0};
    double thrustAfterFiveTau{0.0};

    for (int step = 1; step <= 50; ++step)
    {
        propulsion.updatePropulsion(kDt);

        if (step == 10)
        {
            thrustAfterOneTau =
                propulsion.getCurrentThrustOfOneEngine(EngineType::MainEngine).norm();
        }

        if (step == 50)
        {
            thrustAfterFiveTau =
                propulsion.getCurrentThrustOfOneEngine(EngineType::MainEngine).norm();
        }
    }

    const double expectedAfterOneTau =
        expectedFirstOrderThrust(kMainTargetThrust, 10.0 * kDt, kMainTau);

    const double expectedAfterFiveTau =
        expectedFirstOrderThrust(kMainTargetThrust, 50.0 * kDt, kMainTau);

    EXPECT_NEAR(
        thrustAfterOneTau,
        expectedAfterOneTau,
        kForceTolerance
    ) << "Main-engine thrust after one time constant deviates from the analytical reference.";

    EXPECT_NEAR(
        thrustAfterFiveTau,
        expectedAfterFiveTau,
        kForceTolerance
    ) << "Main-engine thrust after five time constants deviates from the analytical reference.";

    const Eigen::Vector3d expectedForce{
        0.0,
        0.0,
        -expectedAfterFiveTau
    };

    expectVectorNear(
        propulsion.getCurrentThrustOfOneEngine(EngineType::MainEngine),
        expectedForce,
        kForceTolerance
    );

    expectVectorNear(
        propulsion.getTotalTorque(),
        Eigen::Vector3d::Zero(),
        kZeroTolerance
    );
}

TEST(VER_PROP_001_PropulsionGeneration, SingleOffCenterRCSMatchesAnalyticalForceAndTorque)
{
    std::vector<EngineConfig> mainEngines{};
    std::vector<RCSEngineConfig> rcsEngines{
        makeRCS(
            1,
            "RCS Off-Center +X",
            EngineType::RCS_translation,
            "+x",
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0}
        )
    };
    std::vector<FuelTank> tanks{makeTestTank()};

    Thrust propulsion;
    propulsion.initializeEngines(mainEngines, rcsEngines, tanks);

    propulsion.setTargetThrustInPercentage(
        EngineType::RCS_translation,
        0.0,
        {1.0, 0.0, 0.0}
    );

    propulsion.updatePropulsion(kDt);

    const double expectedMagnitude =
        expectedFirstOrderThrust(kRCSMaxThrust, kDt, kRCSTauOn);

    const Eigen::Vector3d expectedForce{
        expectedMagnitude,
        0.0,
        0.0
    };

    const Eigen::Vector3d leverArm{
        0.0,
        1.0,
        0.0
    };

    const Eigen::Vector3d expectedTorque =
        leverArm.cross(expectedForce);

    expectVectorNear(
        propulsion.getCurrentThrustOfOneEngine(EngineType::RCS_translation),
        expectedForce,
        kForceTolerance
    );

    expectVectorNear(
        propulsion.getTotalTorque(),
        expectedTorque,
        kTorqueTolerance
    );
}

TEST(VER_PROP_001_PropulsionGeneration, SymmetricRCSPairProducesPureTranslation)
{
    std::vector<EngineConfig> mainEngines{};
    std::vector<RCSEngineConfig> rcsEngines{
        makeRCS(
            1,
            "RCS +X Upper",
            EngineType::RCS_translation,
            "+x",
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0}
        ),
        makeRCS(
            2,
            "RCS +X Lower",
            EngineType::RCS_translation,
            "+x",
            {1.0, 0.0, 0.0},
            {0.0, -1.0, 0.0}
        )
    };
    std::vector<FuelTank> tanks{makeTestTank()};

    Thrust propulsion;
    propulsion.initializeEngines(mainEngines, rcsEngines, tanks);

    propulsion.setTargetThrustInPercentage(
        EngineType::RCS_translation,
        0.0,
        {1.0, 0.0, 0.0}
    );

    propulsion.updatePropulsion(kDt);

    const double singleThrusterMagnitude =
        expectedFirstOrderThrust(kRCSMaxThrust, kDt, kRCSTauOn);

    const Eigen::Vector3d expectedForce{
        2.0 * singleThrusterMagnitude,
        0.0,
        0.0
    };

    expectVectorNear(
        propulsion.getCurrentThrustOfOneEngine(EngineType::RCS_translation),
        expectedForce,
        kForceTolerance
    );

    expectVectorNear(
        propulsion.getTotalTorque(),
        Eigen::Vector3d::Zero(),
        kZeroTolerance
    );
}

TEST(VER_PROP_001_PropulsionGeneration, OpposedRCSPairProducesPureTorque)
{
    std::vector<EngineConfig> mainEngines{};
    std::vector<RCSEngineConfig> rcsEngines{
        makeRCS(
            1,
            "RCS Torque Upper",
            EngineType::RCS_rotation,
            "-z",
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0}
        ),
        makeRCS(
            2,
            "RCS Torque Lower",
            EngineType::RCS_rotation,
            "-z",
            {-1.0, 0.0, 0.0},
            {0.0, -1.0, 0.0}
        )
    };
    std::vector<FuelTank> tanks{makeTestTank()};

    Thrust propulsion;
    propulsion.initializeEngines(mainEngines, rcsEngines, tanks);

    propulsion.setRCSRotationTargetThrustInPercentage(
        {0.0, 0.0, -1.0},
        Eigen::Vector3d::Zero()
    );

    propulsion.updatePropulsion(kDt);

    const double singleThrusterMagnitude =
        expectedFirstOrderThrust(kRCSMaxThrust, kDt, kRCSTauOn);

    const Eigen::Vector3d expectedTorque{
        0.0,
        0.0,
        -2.0 * singleThrusterMagnitude
    };

    expectVectorNear(
        propulsion.getCurrentThrustOfOneEngine(EngineType::RCS_rotation),
        Eigen::Vector3d::Zero(),
        kZeroTolerance
    );

    expectVectorNear(
        propulsion.getTotalTorque(),
        expectedTorque,
        kTorqueTolerance
    );
}

TEST(VER_PROP_001_PropulsionGeneration, CombinedRCSCommandsProduceExpectedForceAndTorque)
{
    std::vector<EngineConfig> mainEngines{};
    std::vector<RCSEngineConfig> rcsEngines{
        makeRCS(
            1,
            "RCS Translation +X Upper",
            EngineType::RCS_translation,
            "+x",
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0}
        ),
        makeRCS(
            2,
            "RCS Translation +X Lower",
            EngineType::RCS_translation,
            "+x",
            {1.0, 0.0, 0.0},
            {0.0, -1.0, 0.0}
        ),
        makeRCS(
            3,
            "RCS Rotation Upper",
            EngineType::RCS_rotation,
            "-z",
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0}
        ),
        makeRCS(
            4,
            "RCS Rotation Lower",
            EngineType::RCS_rotation,
            "-z",
            {-1.0, 0.0, 0.0},
            {0.0, -1.0, 0.0}
        )
    };
    std::vector<FuelTank> tanks{makeTestTank()};

    Thrust propulsion;
    propulsion.initializeEngines(mainEngines, rcsEngines, tanks);

    propulsion.setTargetThrustInPercentage(
        EngineType::RCS_translation,
        0.0,
        {1.0, 0.0, 0.0}
    );

    propulsion.setRCSRotationTargetThrustInPercentage(
        {0.0, 0.0, -1.0},
        Eigen::Vector3d::Zero()
    );

    propulsion.updatePropulsion(kDt);

    const double singleThrusterMagnitude =
        expectedFirstOrderThrust(kRCSMaxThrust, kDt, kRCSTauOn);

    const Eigen::Vector3d expectedTotalForce{
        2.0 * singleThrusterMagnitude,
        0.0,
        0.0
    };

    const Eigen::Vector3d expectedTotalTorque{
        0.0,
        0.0,
        -2.0 * singleThrusterMagnitude
    };

    expectVectorNear(
        propulsion.getCurrentThrustOfAllEngines(),
        expectedTotalForce,
        kForceTolerance
    );

    expectVectorNear(
        propulsion.getTotalTorque(),
        expectedTotalTorque,
        kTorqueTolerance
    );
}

TEST(VER_PROP_001_PropulsionGeneration, TranslationAxisSweepMatchesBodyAxisSignConvention)
{
    struct AxisCase
    {
        std::string axis;
        Eigen::Vector3d direction;
    };

    const std::array<AxisCase, 6> cases{{
        {"+x", { 1.0,  0.0,  0.0}},
        {"-x", {-1.0,  0.0,  0.0}},
        {"+y", { 0.0,  1.0,  0.0}},
        {"-y", { 0.0, -1.0,  0.0}},
        {"+z", { 0.0,  0.0,  1.0}},
        {"-z", { 0.0,  0.0, -1.0}}
    }};

    const double expectedMagnitude =
        expectedFirstOrderThrust(kRCSMaxThrust, kDt, kRCSTauOn);

    for (const auto& axisCase : cases)
    {
        SCOPED_TRACE("RCS translation axis: " + axisCase.axis);

        std::vector<EngineConfig> mainEngines{};
        std::vector<RCSEngineConfig> rcsEngines{
            makeRCS(
                1,
                "RCS Axis Sweep " + axisCase.axis,
                EngineType::RCS_translation,
                axisCase.axis,
                axisCase.direction,
                Eigen::Vector3d::Zero()
            )
        };
        std::vector<FuelTank> tanks{makeTestTank()};

        Thrust propulsion;
        propulsion.initializeEngines(mainEngines, rcsEngines, tanks);

        propulsion.setTargetThrustInPercentage(
            EngineType::RCS_translation,
            0.0,
            axisCase.direction
        );

        propulsion.updatePropulsion(kDt);

        const Eigen::Vector3d expectedForce =
            expectedMagnitude * axisCase.direction;

        expectVectorNear(
            propulsion.getCurrentThrustOfOneEngine(EngineType::RCS_translation),
            expectedForce,
            kForceTolerance
        );

        expectVectorNear(
            propulsion.getTotalTorque(),
            Eigen::Vector3d::Zero(),
            kZeroTolerance
        );
    }
}
