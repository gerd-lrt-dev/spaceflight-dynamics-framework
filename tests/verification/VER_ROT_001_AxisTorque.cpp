#include <gtest/gtest.h>

#include "physics.h"
#include "Integrators/eulerIntegrator.h"
#include "Physics/rigidBodyRotationalModel.h"

#include <memory>

// Test helpers
namespace
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
TEST(VER_ROT_001_AxisTorque, MatchesAnalyticalSolution)
{
    EnvironmentConfig cfg;

    auto physicsModel        = std::make_shared<ZeroAccelerationModel>();

    auto rotationalModel    = std::make_shared<RigidBodyRotationalModel>(cfg);

    auto integrator         = std::make_shared<EulerIntegrator>();

    auto sensor             = std::make_shared<DummySensor>();

    // Physics instance
    physics physicsSystem(physicsModel, rotationalModel, integrator, sensor);

    // Verification conventions:
    // - Torque and angular velocity are expressed in the spacecraft body-fixed frame (SBF).
    // - Rotation is restricted to the positive SBF x-axis (principal axis).
    // - Initial attitude is the identity quaternion.
    // - SDF attitude quaternion represents the rotation from SBF to the inertial frame.
    // - Eigen quaternion component order is (w, x, y, z).
    // - SDF propagates attitude using the updated angular velocity omega_(n+1).

    const Eigen::Matrix3d inertia = (Eigen::Matrix3d() <<
                                    100.0,  0.0,    0.0,
                                    0.0,    200.0,  0.0,
                                    0.0,    0.0,    300.0
                                    ).finished();

    const Eigen::Vector3d torque{10.0, 0.0, 0.0};

    Eigen::Vector3d angularAcceleration{0.0, 0.0, 0.0};
    Eigen::Vector3d angularVelocity{0.0, 0.0, 0.0};
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
        angularAcceleration = physicsSystem.computeAngAcc(angularVelocity, inertia, torque);

        angularVelocity     = physicsSystem.computeAngVel(angularVelocity, angularAcceleration, dt);

        attitude            = physicsSystem.computeAttitude(attitude, angularVelocity, dt);
    }

    // Analytical reference:
    //
    // alpha_x = tau_x / I_xx
    //         = 10 / 100
    //         = 0.1 rad/s^2
    //
    // omega_x(10 s) = alpha_x * t
    //                 = 1.0 rad/s
    //
    // theta_x(10 s) = 0.5 * alpha_x * t^2
    //                 = 5.0 rad
    //
    // q_ref = [cos(theta/2), sin(theta/2), 0, 0]

    const Eigen::Vector3d       expectedAngularAcceleration{0.1, 0.0, 0.0};
    const Eigen::Vector3d       expectedAngularVelocity{1.0, 0.0, 0.0};
    const Eigen::Quaterniond    expectedAttitude{
        -0.8011436155,
        0.5984721441,
        0.0,
        0.0
    };

    // Tolerances:
    // Angular acceleration and angular velocity use tight absolute tolerances because
    // the selected principal-axis case eliminates gyroscopic cross-coupling and has
    // an analytically exact reference apart from floating-point roundoff.
    //
    // Quaternion components use a wider tolerance because SDF propagates quaternion
    // kinematics numerically using explicit Euler integration with normalization.

    constexpr double angularAccelerationTolerance = 1e-12;
    constexpr double angularVelocityTolerance     = 1e-12;

    constexpr double quaternionComponentTolerance = 2.5e-2;
    constexpr double quaternionNormTolerance      = 1e-12;

    // Verification
    EXPECT_NEAR(angularAcceleration.x(), expectedAngularAcceleration.x(), angularAccelerationTolerance) << "Angular acceleration in X deviates from analytical reference.";
    EXPECT_NEAR(angularAcceleration.y(), expectedAngularAcceleration.y(), angularAccelerationTolerance) << "Angular acceleration in Y deviates from analytical reference.";
    EXPECT_NEAR(angularAcceleration.z(), expectedAngularAcceleration.z(), angularAccelerationTolerance) << "Angular acceleration in Z deviates from analytical reference.";

    EXPECT_NEAR(angularVelocity.x(), expectedAngularVelocity.x(), angularVelocityTolerance) << "Angular velocity in X deviates from analytical reference.";
    EXPECT_NEAR(angularVelocity.y(), expectedAngularVelocity.y(), angularVelocityTolerance) << "Angular velocity in Y deviates from analytical reference.";
    EXPECT_NEAR(angularVelocity.z(), expectedAngularVelocity.z(), angularVelocityTolerance) << "Angular velocity in Z deviates from analytical reference.";

    EXPECT_NEAR(attitude.w(), expectedAttitude.w(), quaternionComponentTolerance) << "Attitude W deviates from analytical reference";
    EXPECT_NEAR(attitude.x(), expectedAttitude.x(), quaternionComponentTolerance) << "Attitude X deviates from analytical reference";
    EXPECT_NEAR(attitude.y(), expectedAttitude.y(), quaternionComponentTolerance) << "Attitude Y deviates from analytical reference";
    EXPECT_NEAR(attitude.z(), expectedAttitude.z(), quaternionComponentTolerance) << "Attitude Z deviates from analytical reference";

    EXPECT_NEAR(attitude.norm(), 1.0, quaternionNormTolerance) << "Attitude quaternion norm deviates from unity.";
}