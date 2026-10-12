#include <gtest/gtest.h>

#include "Coordinates/coordinateTransformer.h"
#include "environmentConfig.h"

#include <cmath>
#include <string>

namespace
{

constexpr double PI = 3.14159265358979323846;
constexpr double MOON_ROTATION_RATE = 2.6617e-6; // rad/s

constexpr double positionTolerance = 1e-6; // m
constexpr double velocityTolerance = 1e-9; // m/s

void expectVectorNear(
    const Eigen::Vector3d& actual,
    const Eigen::Vector3d& expected,
    double tolerance,
    const std::string& label)
{
    EXPECT_NEAR(actual.x(), expected.x(), tolerance)
        << label << " X component deviates from analytical reference.";
    EXPECT_NEAR(actual.y(), expected.y(), tolerance)
        << label << " Y component deviates from analytical reference.";
    EXPECT_NEAR(actual.z(), expected.z(), tolerance)
        << label << " Z component deviates from analytical reference.";
}

} // namespace

TEST(VER_FRM_004_MCItoMCMF, TimeZero)
{
    CoordinateTransformer transformer;

    // At t = 0 the frames are position-aligned.
    //
    // Position:
    // r_MCMF = r_MCI
    //
    // Rotating-frame velocity:
    // v_MCMF = v_MCI - omega x r_MCI
    //
    // For
    // r = {1000, 2000, 3000} m
    // v = {10, 20, 30} m/s
    //
    // omega x r = {-0.0053234, 0.0026617, 0} m/s
    //
    // v_MCMF = {10.0053234, 19.9973383, 30} m/s

    const CoordinateTransformer::State inertial{
        {1000.0, 2000.0, 3000.0},
        {10.0, 20.0, 30.0}
    };

    const auto fixed = transformer.MCItoMCMF(inertial, 0.0);

    expectVectorNear(
        fixed.position,
        {1000.0, 2000.0, 3000.0},
        positionTolerance,
        "t=0 MCI-to-MCMF position");

    expectVectorNear(
        fixed.velocity,
        {10.0053234, 19.9973383, 30.0},
        velocityTolerance,
        "t=0 MCI-to-MCMF velocity");
}

TEST(VER_FRM_004_MCItoMCMF, QuarterTurnPrincipalAxis)
{
    CoordinateTransformer transformer;
    EnvironmentConfig econfig;

    // Positive lunar rotation angle:
    // theta = omega * t = +pi/2
    //
    // With the production frame convention, a +X MCI position maps to +Y MCMF:
    //
    // r_MCI  = {R, 0, 0}
    // r_MCMF = {0, R, 0}
    //
    // For v_MCI = 0:
    //
    // omega x r = {0, omega*R, 0}
    // v_MCI - omega x r = {0, -omega*R, 0}
    //
    // Rotating by +pi/2 gives:
    // v_MCMF = {omega*R, 0, 0}
    //
    // omega*R = 4.62443758 m/s

    const double quarterTurnTime = PI / (2.0 * MOON_ROTATION_RATE);

    const CoordinateTransformer::State inertial{
        {econfig.radiusMoon, 0.0, 0.0},
        Eigen::Vector3d::Zero()
    };

    const auto fixed = transformer.MCItoMCMF(inertial, quarterTurnTime);

    expectVectorNear(
        fixed.position,
        {0.0, econfig.radiusMoon, 0.0},
        positionTolerance,
        "Quarter-turn MCI-to-MCMF position");

    expectVectorNear(
        fixed.velocity,
        {4.62443758, 0.0, 0.0},
        velocityTolerance,
        "Quarter-turn MCI-to-MCMF velocity");
}

TEST(VER_FRM_004_MCItoMCMF, HalfTurnPrincipalAxis)
{
    CoordinateTransformer transformer;
    EnvironmentConfig econfig;

    // theta = pi
    //
    // {R, 0, 0} -> {-R, 0, 0}

    const double halfTurnTime = PI / MOON_ROTATION_RATE;

    const CoordinateTransformer::State inertial{
        {econfig.radiusMoon, 0.0, 0.0},
        {0.0, MOON_ROTATION_RATE * econfig.radiusMoon, 0.0}
    };

    // The selected inertial velocity equals omega x r, therefore the
    // spacecraft is stationary in the Moon-fixed frame.
    const auto fixed = transformer.MCItoMCMF(inertial, halfTurnTime);

    expectVectorNear(
        fixed.position,
        {-econfig.radiusMoon, 0.0, 0.0},
        positionTolerance,
        "Half-turn MCI-to-MCMF position");

    expectVectorNear(
        fixed.velocity,
        Eigen::Vector3d::Zero(),
        velocityTolerance,
        "Half-turn MCI-to-MCMF velocity");
}

TEST(VER_FRM_004_MCItoMCMF, ArbitraryAngleAndState)
{
    CoordinateTransformer transformer;

    // Arbitrary non-zero time:
    //
    // t     = 123456.789 s
    // theta = omega*t = 0.32860493528129997 rad
    //
    // Initial MCI state:
    // r = {1000000, -200000, 300000} m
    // v = {100, -50, 25} m/s
    //
    // Analytical reference using
    //
    // Rz(theta) =
    // [ cos(theta) -sin(theta) 0 ]
    // [ sin(theta)  cos(theta) 0 ]
    // [     0           0      1 ]
    //
    // r_MCMF = Rz(theta) r_MCI
    // v_MCMF = Rz(theta) [v_MCI - omega x r_MCI]
    //
    // gives the frozen scalar references below.

    constexpr double t = 123456.789;

    const CoordinateTransformer::State inertial{
        {1000000.0, -200000.0, 300000.0},
        {100.0, -50.0, 25.0}
    };

    const auto fixed = transformer.MCItoMCMF(inertial, t);

    expectVectorNear(
        fixed.position,
        {1011038.068421715986915,
         133424.226443647057749,
         300000.0},
        positionTolerance,
        "Arbitrary-angle MCI-to-MCMF position");

    expectVectorNear(
        fixed.velocity,
        {111.140629801696363,
         -17.743461895804906,
         25.0},
        velocityTolerance,
        "Arbitrary-angle MCI-to-MCMF velocity");
}

TEST(VER_FRM_004_MCMFtoMCI, QuarterTurnFixedPoint)
{
    CoordinateTransformer transformer;
    EnvironmentConfig econfig;

    // theta = +pi/2
    //
    // Fixed-frame state:
    // r_MCMF = {0, R, 0}
    // v_MCMF = {0, 0, 0}
    //
    // Position inverse:
    // r_MCI = Rz(-theta) r_MCMF = {R, 0, 0}
    //
    // Velocity:
    // v_MCI = Rz(-theta) [v_MCMF + omega x r_MCMF]
    //
    // omega x r_MCMF = {-omega*R, 0, 0}
    //
    // therefore
    // v_MCI = {0, omega*R, 0}
    //       = {0, 4.62443758, 0} m/s

    const double quarterTurnTime = PI / (2.0 * MOON_ROTATION_RATE);

    const CoordinateTransformer::State fixed{
        {0.0, econfig.radiusMoon, 0.0},
        Eigen::Vector3d::Zero()
    };

    const auto inertial = transformer.MCMFtoMCI(fixed, quarterTurnTime);

    expectVectorNear(
        inertial.position,
        {econfig.radiusMoon, 0.0, 0.0},
        positionTolerance,
        "Quarter-turn MCMF-to-MCI position");

    expectVectorNear(
        inertial.velocity,
        {0.0, 4.62443758, 0.0},
        velocityTolerance,
        "Quarter-turn MCMF-to-MCI velocity");
}

TEST(VER_FRM_004_Roundtrip, MCItoMCMFtoMCI)
{
    CoordinateTransformer transformer;

    constexpr double t = 345678.9;

    const CoordinateTransformer::State original{
        {1200000.0, -450000.0, 275000.0},
        {250.0, 125.0, -30.0}
    };

    const auto fixed = transformer.MCItoMCMF(original, t);
    const auto reconstructed = transformer.MCMFtoMCI(fixed, t);

    expectVectorNear(
        reconstructed.position,
        original.position,
        positionTolerance,
        "MCI-MCMF-MCI roundtrip position");

    expectVectorNear(
        reconstructed.velocity,
        original.velocity,
        velocityTolerance,
        "MCI-MCMF-MCI roundtrip velocity");
}

TEST(VER_FRM_004_Roundtrip, MCMFtoMCItoMCMF)
{
    CoordinateTransformer transformer;

    constexpr double t = 765432.1;

    const CoordinateTransformer::State original{
        {-800000.0, 900000.0, 150000.0},
        {-75.0, 40.0, 12.0}
    };

    const auto inertial = transformer.MCMFtoMCI(original, t);
    const auto reconstructed = transformer.MCItoMCMF(inertial, t);

    expectVectorNear(
        reconstructed.position,
        original.position,
        positionTolerance,
        "MCMF-MCI-MCMF roundtrip position");

    expectVectorNear(
        reconstructed.velocity,
        original.velocity,
        velocityTolerance,
        "MCMF-MCI-MCMF roundtrip velocity");
}
