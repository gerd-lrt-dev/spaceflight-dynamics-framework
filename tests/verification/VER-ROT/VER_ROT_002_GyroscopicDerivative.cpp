// SPDX-License-Identifier: MIT
// Copyright (C) 2026 Spaceflight Dynamics Framework contributors

#include <gtest/gtest.h>

#include "physics.h"
#include "Integrators/eulerIntegrator.h"
#include "Physics/rigidBodyRotationalModel.h"

#include <memory>

// Test helpers
namespace VER_ROT_002
{

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

} // namespace

// Test module
TEST(VER_ROT_002_GyroscopicDerivative, MatchesAnalyticalSolution)
{
    EnvironmentConfig cfg;

    auto physicsModel       = std::make_shared<VER_ROT_002::ZeroAccelerationModel>();

    auto rotationalModel    = std::make_shared<RigidBodyRotationalModel>(cfg);

    auto integrator         = std::make_shared<EulerIntegrator>();

    auto sensor             = std::make_shared<VER_ROT_002::DummySensor>();

    // Physics instance
    physics physicsSystem(physicsModel, rotationalModel, integrator, sensor);

    // Verification conventions:

    const Eigen::Matrix3d inertia1 = (Eigen::Matrix3d() <<
                                    100.0,  0.0,    0.0,
                                    0.0,    200.0,  0.0,
                                    0.0,    0.0,    300.0
                                    ).finished();

    const Eigen::Vector3d torque = Eigen::Vector3d::Zero();

    // Start Conditions

    Eigen::Vector3d angularVelocity_start{0.1, 0.2, 0.3};

    Eigen::Quaterniond attitude_start{
        1.0,  // w
        0.0,  // x
        0.0,  // y
        0.0   // z
    };

    // Caclulated Test Results

    Eigen::Vector3d angularAcceleration_T1{0.0, 0.0, 0.0};
    Eigen::Vector3d angularAcceleration_T2{0.0, 0.0, 0.0};

    Eigen::Vector3d angularVelocity_T2{0.0, 0.0, 0.0};

    Eigen::Quaterniond attitude_T2{0.0, 0.0, 0.0, 0.0};

    // Test procedure

    // Testcase 1 Initial gyroscopic derivatvie
    angularAcceleration_T1 = physicsSystem.computeAngAcc(angularVelocity_start, inertia1, torque);

    // Testcase 2 Time evolution of angular velocity
    const double dt     = 0.01;
    const int steps     = 100;
    angularVelocity_T2  = angularVelocity_start;
    attitude_T2         = attitude_start;

    // Simulation Loop
    for (int i = 0; i < steps; ++i)
    {
        angularAcceleration_T2  = physicsSystem.computeAngAcc(angularVelocity_T2, inertia1, torque);
        angularVelocity_T2      = physicsSystem.computeAngVel(angularVelocity_T2, angularAcceleration_T2, dt);
        attitude_T2             = physicsSystem.computeAttitude(attitude_T2, angularVelocity_T2, dt);
    }

    // Test expectations
    const Eigen::Vector3d expectedAngularAcceleration_T1{-0.06, 0.03, -0.006666666666666667};
    const Eigen::Vector3d expectedAngularVelocity_T2{
        0.0370354933652774,
        0.2205184169886715,
        0.2951675387351000
    };

    // Tolerances:
    // Angular acceleration and angular velocity use tight absolute tolerances because
    // the selected principal-axis case eliminates gyroscopic cross-coupling and has
    // an analytically exact reference apart from floating-point roundoff.
    //
    // Quaternion components use a wider tolerance because SDF propagates quaternion
    // kinematics numerically using explicit Euler integration with normalization.

    constexpr double angularAccelerationTolerance = 1e-12;
    constexpr double angularVelocityTolerance     = 2e-4;

    constexpr double quaternionComponentTolerance = 2.5e-2;
    constexpr double quaternionNormTolerance      = 1e-12;

    // Verifications

    // Test case 1
    EXPECT_NEAR(angularAcceleration_T1.x(), expectedAngularAcceleration_T1.x(), angularAccelerationTolerance) << "Angular acceleration in X deviates from analytical reference.";
    EXPECT_NEAR(angularAcceleration_T1.y(), expectedAngularAcceleration_T1.y(), angularAccelerationTolerance) << "Angular acceleration in Y deviates from analytical reference.";
    EXPECT_NEAR(angularAcceleration_T1.z(), expectedAngularAcceleration_T1.z(), angularAccelerationTolerance) << "Angular acceleration in Z deviates from analytical reference.";

    // Test Case 2
    EXPECT_NEAR(angularVelocity_T2.x(), expectedAngularVelocity_T2.x(), angularVelocityTolerance) << "Angular velocity in X deviates from analytical reference.";
    EXPECT_NEAR(angularVelocity_T2.y(), expectedAngularVelocity_T2.y(), angularVelocityTolerance) << "Angular velocity in Y deviates from analytical reference.";
    EXPECT_NEAR(angularVelocity_T2.z(), expectedAngularVelocity_T2.z(), angularVelocityTolerance) << "Angular velocity in Z deviates from analytical reference.";
}