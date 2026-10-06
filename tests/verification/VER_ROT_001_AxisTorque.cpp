#include <gtest/gtest.h>

#include "physics.h"
#include "Integrators/eulerIntegrator.h"

#include <memory>

// Classes

class ZeroAccelerationModel : public IPhysicsModel{
public:
    Eigen::Vector3d computeAcceleration(const Eigen::Vector3d&, const Eigen::Vector3d&, double, const Eigen::Vector3d&) const override
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

const EnvironmentConfig& cfg;

auto physicsModel        = std::make_shared<ZeroAccelerationModel>();

auto rotationalModel    = std::make_shared<RigidBodyRotationalModel>(cfg);

auto integrator         = std::make_shared<EulerIntegrator>();

auto sensor             = std::make_shared<DummySensor>();

// Physics instance
physics(physicsModel, rotationalModel, integrator, DummySensor);

// Test module
TEST(VER_ROT_001_AxisTorque, MatchesAnalyticalSolution)
{
    const Eigen::Matrix3d inertia = (Eigen::Matrix3d() <<
                                    100.0,  0.0,    0.0,
                                    0.0,    200.0,  0.0,
                                    0.0,    0.0,    300.0
                                    ).finished();

    Eigen::Vector3d angularVelocity{0.0, 0.0, 0.0};

    const Eigen::Vector3d torque{10.0, 0.0, 0.0};

    Eigen::Quaterniond attitude{
        1.0,  // w
        0.0,  // x
        0.0,  // y
        0.0   // z
    };

    const double dt = 0.1;
    const int steps = 100;

    // Simulation Loop
    for (int i = 0; i < steps; ++i)
    {
        const Eigen::Vector3d angularAcceleration = rotationalModel.computeAngAcc(angularVelocity, inertia, torque);

        angularVelocity = rotationalModel.computeAngVel(angularVelocity, angularAcceleration, dt);

        attitude        = rotationalModel.computeAttitude(attitude, angularVelocity, dt);
    }

    // Analyitcal reference
    const Eigen::Quaterniond expectedAngVelocity = {1.0, 0.0, 0.0, 0.0};

    const Eigen::Quaterniond expectedAttitude{};
    expectedAttitude.w() = -0.8011436155;
    expectedAttitude.x() = 0.5984721441;
    expectedAttitude.y() = 0.0;
    expectedAttitude.z() = 0.0;

    // Tolerances
    constexpr double angVelocitiyTolerance = 1e-12;
    constexpr double attitudeTolerance = 1e-9;

    // Verification
    EXPECT_NEAR(angularVelocity.x(), expectedAngVelocity.x(), angVelocitiyTolerance) << "Angular velocity in X deviates from analytical reference.";
    EXPECT_NEAR(angularVelocity.y(), expectedAngVelocity.y(), angVelocitiyTolerance) << "Angular velocity in Y deviates from analytical reference.";
    EXPECT_NEAR(angularVelocity.z(), expectedAngVelocity.z(), angVelocitiyTolerance) << "Angular velocity in Z deviates from analytical reference.";

    EXPECT_NEAR(attitude.w(), expectedAttitude.w(), attitudeTolerance) << "Attitude W deviates from analytical reference";
    EXPECT_NEAR(attitude.x(), expectedAttitude.x(), attitudeTolerance) << "Attitude X deviates from analytical reference";
    EXPECT_NEAR(attitude.y(), expectedAttitude.y(), attitudeTolerance) << "Attitude Y deviates from analytical reference";
    EXPECT_NEAR(attitude.z(), expectedAttitude.z(), attitudeTolerance) << "Attitude Z deviates from analytical reference";
}