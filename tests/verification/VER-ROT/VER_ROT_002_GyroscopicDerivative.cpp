// SPDX-License-Identifier: MIT
// Copyright (C) 2026 Spaceflight Dynamics Framework contributors

#include <gtest/gtest.h>

#include "physics.h"
#include "Integrators/eulerIntegrator.h"
#include "Physics/rigidBodyRotationalModel.h"

#include <algorithm>
#include <cmath>
#include <memory>

namespace VER_ROT_002
{

// ============================================================================
// Test models
// ============================================================================

class ZeroAccelerationModel : public IPhysicsModel
{
public:
    Eigen::Vector3d computeAcceleration(
        const Eigen::Vector3d&,
        const Eigen::Vector3d&,
        double,
        const Eigen::Vector3d&) const override
    {
        return Eigen::Vector3d::Zero();
    }
};

class DummySensor : public ISensor
{
public:
    double computeGLoad(
        const Eigen::Vector3d&,
        const Eigen::Vector3d&,
        bool) const override
    {
        return 0.0;
    }
};


// ============================================================================
// Independent physical reference functions
// ============================================================================

/**
 * @brief Calculates the rotational kinetic energy.
 *
 * @param angularVelocity Body-frame angular velocity [rad/s].
 * @param inertia Inertia tensor [kg m²].
 * @return Rotational kinetic energy [J].
 */
double computeRotationalEnergy(
    const Eigen::Vector3d& angularVelocity,
    const Eigen::Matrix3d& inertia)
{
    return 0.5 * angularVelocity.dot(inertia * angularVelocity);
}


/**
 * @brief Calculates the body-frame angular momentum.
 *
 * @param angularVelocity Body-frame angular velocity [rad/s].
 * @param inertia Inertia tensor [kg m²].
 * @return Angular momentum vector [kg m²/s].
 */
Eigen::Vector3d computeAngularMomentum(
    const Eigen::Vector3d& angularVelocity,
    const Eigen::Matrix3d& inertia)
{
    return inertia * angularVelocity;
}


// ============================================================================
// Common test fixture
// ============================================================================

/**
 * @brief Common test environment for VER-ROT-002.
 *
 * Provides the production rotational model, Forward Euler integrator,
 * and common initial conditions for torque-free gyroscopic rotation.
 *
 * GoogleTest creates a new fixture instance for every test case.
 */
class RotationalTestFixture : public ::testing::Test
{
protected:

    // ------------------------------------------------------------------------
    // Environment and production models
    // ------------------------------------------------------------------------

    EnvironmentConfig cfg;

    std::shared_ptr<ZeroAccelerationModel> physicsModel;
    std::shared_ptr<RigidBodyRotationalModel> rotationalModel;
    std::shared_ptr<EulerIntegrator> integrator;
    std::shared_ptr<DummySensor> sensor;

    std::unique_ptr<physics> physicsSystem;


    // ------------------------------------------------------------------------
    // Common reference configuration
    // ------------------------------------------------------------------------

    const Eigen::Matrix3d inertia =
        (Eigen::Matrix3d() <<
             100.0,   0.0,   0.0,
         0.0, 200.0,   0.0,
         0.0,   0.0, 300.0
         ).finished();

    const Eigen::Vector3d torque =
        Eigen::Vector3d::Zero();

    const Eigen::Vector3d initialAngularVelocity{
        0.1,
        0.2,
        0.3
    };

    const Eigen::Quaterniond initialAttitude =
        Eigen::Quaterniond::Identity();

    static constexpr double defaultTimeStep = 0.01;


    // ------------------------------------------------------------------------
    // Test initialization
    // ------------------------------------------------------------------------

    void SetUp() override
    {
        physicsModel =
            std::make_shared<ZeroAccelerationModel>();

        rotationalModel =
            std::make_shared<RigidBodyRotationalModel>(cfg);

        integrator =
            std::make_shared<EulerIntegrator>();

        sensor =
            std::make_shared<DummySensor>();

        physicsSystem =
            std::make_unique<physics>(
                physicsModel,
                rotationalModel,
                integrator,
                sensor);
    }


    // ------------------------------------------------------------------------
    // Production state propagation
    // ------------------------------------------------------------------------

    /**
     * @brief Propagates angular velocity by one integration step.
     *
     * Uses the production rotational dynamics model and integrator.
     *
     * @param angularVelocity Current angular velocity [rad/s].
     * @param dt Integration time step [s].
     * @return Propagated angular velocity [rad/s].
     */
    Eigen::Vector3d propagateAngularVelocity(
        const Eigen::Vector3d& angularVelocity,
        const double dt)
    {
        const Eigen::Vector3d angularAcceleration =
            physicsSystem->computeAngAcc(
                angularVelocity,
                inertia,
                torque);

        return physicsSystem->computeAngVel(
            angularVelocity,
            angularAcceleration,
            dt);
    }
};


// ============================================================================
// Test Case 1 - Initial Gyroscopic Angular Acceleration
// ============================================================================

TEST_F(
    RotationalTestFixture,
    InitialGyroscopicDerivativeMatchesAnalyticalSolution)
{
    // Reference data

    const Eigen::Vector3d expectedAngularAcceleration{
        -0.06,
        0.03,
        -0.006666666666666667
    };

    constexpr double tolerance = 1e-12;


    // Test procedure

    const Eigen::Vector3d angularAcceleration =
        physicsSystem->computeAngAcc(
            initialAngularVelocity,
            inertia,
            torque);


    // Verification

    ASSERT_TRUE(angularAcceleration.allFinite());

    EXPECT_NEAR(
        angularAcceleration.x(),
        expectedAngularAcceleration.x(),
        tolerance)
        << "Angular acceleration in X deviates from analytical reference.";

    EXPECT_NEAR(
        angularAcceleration.y(),
        expectedAngularAcceleration.y(),
        tolerance)
        << "Angular acceleration in Y deviates from analytical reference.";

    EXPECT_NEAR(
        angularAcceleration.z(),
        expectedAngularAcceleration.z(),
        tolerance)
        << "Angular acceleration in Z deviates from analytical reference.";
}


// ============================================================================
// Test Case 2 - Torque-Free Angular Velocity Evolution
// ============================================================================

TEST_F(
    RotationalTestFixture,
    AngularVelocityMatchesIndependentReference)
{
    // Reference data

    constexpr double dt = defaultTimeStep;
    constexpr int steps = 100;

    constexpr double tolerance = 2e-4;

    const Eigen::Vector3d expectedAngularVelocity{
        0.0370354933652774,
        0.2205184169886715,
        0.2951675387351000
    };


    // Initial state

    Eigen::Vector3d angularVelocity =
        initialAngularVelocity;


    // Test procedure

    for (int i = 0; i < steps; ++i)
    {
        angularVelocity =
            propagateAngularVelocity(angularVelocity, dt);

        ASSERT_TRUE(angularVelocity.allFinite())
            << "Non-finite angular velocity at step " << i + 1;
    }


    // Verification

    const double angularVelocityError =
        (angularVelocity - expectedAngularVelocity).norm();

    EXPECT_LT(angularVelocityError, tolerance)
        << "Angular velocity deviates from independent reference."
        << "\nCalculated: "
        << angularVelocity.transpose()
        << "\nReference:  "
        << expectedAngularVelocity.transpose()
        << "\nError:      "
        << angularVelocityError
        << "\nTolerance:  "
        << tolerance;
}


// ============================================================================
// Test Case 3 - Rotational Kinetic Energy Conservation
// ============================================================================

TEST_F(
    RotationalTestFixture,
    RotationalEnergyRemainsBounded)
{
    // Reference data

    constexpr double dt = defaultTimeStep;
    constexpr int steps = 1000;

    constexpr double expectedInitialEnergy = 18.0;
    constexpr double initialEnergyTolerance = 1e-12;

    constexpr double relativeEnergyTolerance = 3e-3;


    // Initial state

    Eigen::Vector3d angularVelocity =
        initialAngularVelocity;

    const double initialEnergy =
        computeRotationalEnergy(
            initialAngularVelocity,
            inertia);

    double maxRelativeEnergyError = 0.0;


    // Initial state verification

    ASSERT_TRUE(std::isfinite(initialEnergy));

    ASSERT_NEAR(
        initialEnergy,
        expectedInitialEnergy,
        initialEnergyTolerance);

    ASSERT_GT(initialEnergy, 0.0);


    // Test procedure

    for (int i = 0; i < steps; ++i)
    {
        angularVelocity =
            propagateAngularVelocity(angularVelocity, dt);

        ASSERT_TRUE(angularVelocity.allFinite())
            << "Non-finite angular velocity at step " << i + 1;

        const double rotationalEnergy =
            computeRotationalEnergy(
                angularVelocity,
                inertia);

        ASSERT_TRUE(std::isfinite(rotationalEnergy))
            << "Non-finite rotational energy at step " << i + 1;

        const double relativeEnergyError =
            std::abs(rotationalEnergy - initialEnergy)
            / initialEnergy;

        maxRelativeEnergyError =
            std::max(
                maxRelativeEnergyError,
                relativeEnergyError);
    }


    // Verification

    EXPECT_LT(
        maxRelativeEnergyError,
        relativeEnergyTolerance)
        << "Maximum relative rotational energy error exceeds tolerance."
        << "\nMaximum error: "
        << maxRelativeEnergyError
        << "\nTolerance:     "
        << relativeEnergyTolerance;
}


// ============================================================================
// Test Case 4 - Angular Momentum Magnitude Conservation
// ============================================================================

TEST_F(
    RotationalTestFixture,
    AngularMomentumMagnitudeRemainsBounded)
{
    GTEST_SKIP() << "Test Case 4 implementation pending.";

    // Reference data

    constexpr double dt = defaultTimeStep;
    constexpr int steps = 1000;

    const Eigen::Vector3d expectedInitialAngularMomentum{
        10.0,
        40.0,
        90.0
    };

    const double expectedInitialAngularMomentumMagnitude =
        std::sqrt(9800.0);

    // TODO: Define acceptance tolerance according to test specification.


    // Initial state

    Eigen::Vector3d angularVelocity =
        initialAngularVelocity;

    const Eigen::Vector3d initialAngularMomentum =
        computeAngularMomentum(
            initialAngularVelocity,
            inertia);

    const double initialAngularMomentumMagnitude =
        initialAngularMomentum.norm();

    double maxRelativeAngularMomentumError = 0.0;


    // TODO: Initial state verification


    // Test procedure

    for (int i = 0; i < steps; ++i)
    {
        angularVelocity =
            propagateAngularVelocity(angularVelocity, dt);

        // TODO: Calculate current angular momentum vector.

        // TODO: Calculate angular momentum magnitude.

        // TODO: Verify finite values.

        // TODO: Calculate relative angular momentum error.

        // TODO: Track maximum relative error.
    }


    // TODO: Verify maximum relative angular momentum error.
}


// ============================================================================
// Test Case 5 - Quaternion Normalization
// ============================================================================

// TODO: Implement quaternion normalization verification.


// ============================================================================
// Test Case 6 - Time Step Convergence
// ============================================================================

// TODO: Implement time step convergence verification.


} // namespace VER_ROT_002