#include <gtest/gtest.h>

#include "physics.h"
#include "Integrators/eulerIntegrator.h"

#include <memory>

// Dummy classes

class ZeroAccelerationModel : public IPhysicsModel{
public:
    Eigen::Vector3d computeAcceleration(const Eigen::Vector3d&, const Eigen::Vector3d&, double, const Eigen::Vector3d&) const override
    {
        return Eigen::Vector3d::Zero();
    }
};

class DummyRotationalModel : public IRotationalPhysicsModel{
public:
    Eigen::Vector3d computeAngularAcceleration(const Eigen::Vector3d&, const Eigen::Matrix3d&, const Eigen::Vector3d&) const override
    {
        return Eigen::Vector3d::Zero();
    }
};

class DummySensor : public ISensor{
public:
    double computeGLoad(const Eigen::Vector3d&, const Eigen::Vector3d&, bool) const override
    {
        return 0.0;
    }
};

auto physicsModel        = std::make_shared<ZeroAccelerationModel>();

auto rotationalModel    = std::make_shared<DummyRotationalModel>();

auto integrator         = std::make_shared<EulerIntegrator>();

auto sensor             = std::make_shared<DummySensor>();

// Physics instance
physics physicsSystem(physicsModel, rotationalModel, integrator, sensor);

// Test module
TEST(VER_TRA_001_FreeTranslation, MatchesAnalyticalSolution)
{
    // Starting conditions
    // Verification frame:
    // Position and velocity are treated as inertial translational state vectors.
    // No rotating-frame or coordinate transformation effects are part of VER-TRA-001.

    Eigen::Vector3d position{1000.0, -2000.0, 3000.0};
    Eigen::Vector3d velocity{10.0, -5.0, 2.0};

    const Eigen::Vector3d thrust = Eigen::Vector3d::Zero();

    const double mass = 1000.0;
    const double dt = 0.1;
    const int steps = 100;

    // Simulation Loop
    for (int i = 0; i < steps; ++i)
    {
        const Eigen::Vector3d acceleration = physicsSystem.computeAcc(position, velocity, mass, thrust);

        position = physicsSystem.computePos(position, velocity, acceleration, dt);

        velocity = physicsSystem.computeVel(velocity, acceleration, dt);
    }

    // Analytical reference
    const Eigen::Vector3d expectedPosition{1100.0, -2050.0, 3020.0};

    const Eigen::Vector3d expectedVelocity{10.0, -5.0, 2.0};

    // Tolerances
    constexpr double positionTolerance = 1e-9;
    constexpr double velocityTolerance = 1e-12;

    // Verification
    EXPECT_NEAR(position.x(), expectedPosition.x(), positionTolerance) << "Position X deviates from analytical reference.";

    EXPECT_NEAR(position.y(), expectedPosition.y(), positionTolerance) << "Position Y deviates from analytical reference.";

    EXPECT_NEAR(position.z(), expectedPosition.z(), positionTolerance) << "Position Z deviates from analytical reference.";

    EXPECT_NEAR(velocity.x(), expectedVelocity.x(), velocityTolerance) << "Velocity X deviates from analytical reference.";

    EXPECT_NEAR(velocity.y(), expectedVelocity.y(), velocityTolerance) << "Velocity Y deviates from analytical reference.";

    EXPECT_NEAR(velocity.z(), expectedVelocity.z(), velocityTolerance) << "Velocity Z deviates from analytical reference.";

}

