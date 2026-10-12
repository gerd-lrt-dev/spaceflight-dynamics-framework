#include <gtest/gtest.h>

#include "Coordinates/coordinateTransformer.h"
#include "environmentConfig.h"

#include <string>

namespace
{

constexpr double PI = 3.14159265358979323846;

constexpr double basisTolerance     = 1e-12;
constexpr double cartesianTolerance = 1e-6;  // m
constexpr double velocityTolerance  = 1e-12; // m/s
constexpr double nearPoleTolerance  = 1e-10;

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

void expectOrthonormalRightHanded(
    const CoordinateTransformer::ENUFrame& frame,
    double tolerance)
{
    EXPECT_NEAR(frame.east.norm(), 1.0, tolerance);
    EXPECT_NEAR(frame.north.norm(), 1.0, tolerance);
    EXPECT_NEAR(frame.up.norm(), 1.0, tolerance);

    EXPECT_NEAR(frame.east.dot(frame.north), 0.0, tolerance);
    EXPECT_NEAR(frame.east.dot(frame.up), 0.0, tolerance);
    EXPECT_NEAR(frame.north.dot(frame.up), 0.0, tolerance);

    expectVectorNear(
        frame.east.cross(frame.north),
        frame.up,
        tolerance,
        "ENU handedness E x N = U");
}

} // namespace

TEST(VER_FRM_003_ENUBasis, EquatorPrimeMeridian)
{
    CoordinateTransformer transformer;
    EnvironmentConfig econfig;

    // lat = 0 deg, lon = 0 deg:
    //
    // origin = {R_Moon, 0, 0}
    // East   = {0, 1, 0}
    // North  = {0, 0, 1}
    // Up     = {1, 0, 0}

    const CoordinateTransformer::MoonSurfaceCoordinates site{
        0.0,
        0.0,
        0.0
    };

    const auto origin = transformer.MSCtoMCMF(site);
    const auto frame = transformer.computeENUFrame(origin);

    expectVectorNear(
        frame.origin.position,
        {econfig.radiusMoon, 0.0, 0.0},
        cartesianTolerance,
        "Equator/prime-meridian ENU origin");

    expectVectorNear(frame.east, {0.0, 1.0, 0.0}, basisTolerance, "East basis");
    expectVectorNear(frame.north, {0.0, 0.0, 1.0}, basisTolerance, "North basis");
    expectVectorNear(frame.up, {1.0, 0.0, 0.0}, basisTolerance, "Up basis");

    expectOrthonormalRightHanded(frame, basisTolerance);
}

TEST(VER_FRM_003_ENUBasis, EquatorLongitude90)
{
    CoordinateTransformer transformer;
    EnvironmentConfig econfig;

    // lat = 0 deg, lon = +90 deg:
    //
    // origin = {0, R_Moon, 0}
    // East   = {-1, 0, 0}
    // North  = {0, 0, 1}
    // Up     = {0, 1, 0}

    const CoordinateTransformer::MoonSurfaceCoordinates site{
        0.0,
        PI / 2.0,
        0.0
    };

    const auto origin = transformer.MSCtoMCMF(site);
    const auto frame = transformer.computeENUFrame(origin);

    expectVectorNear(
        frame.origin.position,
        {0.0, econfig.radiusMoon, 0.0},
        cartesianTolerance,
        "Equator/+90-deg ENU origin");

    expectVectorNear(frame.east, {-1.0, 0.0, 0.0}, basisTolerance, "East basis");
    expectVectorNear(frame.north, {0.0, 0.0, 1.0}, basisTolerance, "North basis");
    expectVectorNear(frame.up, {0.0, 1.0, 0.0}, basisTolerance, "Up basis");

    expectOrthonormalRightHanded(frame, basisTolerance);
}

TEST(VER_FRM_003_ENUBasis, NonZeroLatitudeLongitude)
{
    CoordinateTransformer transformer;

    // lat = 30 deg, lon = 45 deg
    //
    // Analytical spherical ENU basis:
    //
    // East  = {-sin(lon), cos(lon), 0}
    // North = {-sin(lat)cos(lon), -sin(lat)sin(lon), cos(lat)}
    // Up    = { cos(lat)cos(lon),  cos(lat)sin(lon), sin(lat)}
    //
    // Frozen scalar references:
    //
    // East  = {-0.7071067811865475,  0.7071067811865476, 0}
    // North = {-0.3535533905932737, -0.3535533905932737, 0.8660254037844387}
    // Up    = { 0.6123724356957946,  0.6123724356957945, 0.5}

    const CoordinateTransformer::MoonSurfaceCoordinates site{
        30.0 * PI / 180.0,
        45.0 * PI / 180.0,
        0.0
    };

    const auto origin = transformer.MSCtoMCMF(site);
    const auto frame = transformer.computeENUFrame(origin);

    expectVectorNear(
        frame.east,
        {-0.7071067811865475, 0.7071067811865476, 0.0},
        basisTolerance,
        "General-case East basis");

    expectVectorNear(
        frame.north,
        {-0.3535533905932737, -0.3535533905932737, 0.8660254037844387},
        basisTolerance,
        "General-case North basis");

    expectVectorNear(
        frame.up,
        {0.6123724356957946, 0.6123724356957945, 0.5},
        basisTolerance,
        "General-case Up basis");

    expectOrthonormalRightHanded(frame, basisTolerance);
}

TEST(VER_FRM_003_ENUBasis, NearPoleFallback)
{
    CoordinateTransformer transformer;

    // Near-pole reference:
    // lat = 89.999 deg, lon = 45 deg
    //
    // The production ENU builder intentionally changes its reference axis
    // close to the poles to avoid a singular cross product. The resulting
    // basis is therefore verified against this documented fallback convention.

    const CoordinateTransformer::MoonSurfaceCoordinates site{
        89.999 * PI / 180.0,
        45.0 * PI / 180.0,
        100.0
    };

    const auto origin = transformer.MSCtoMCMF(site);
    const auto frame = transformer.computeENUFrame(origin);

    const Eigen::Vector3d expectedUp{
        1.2341341494268912e-05,
        1.2341341494268910e-05,
        0.9999999998476913
    };

    const Eigen::Vector3d expectedEast{
        0.0,
        -0.9999999999238456,
        1.2341341495208715e-05
    };

    const Eigen::Vector3d expectedNorth{
        0.9999999999238456,
        -1.5230870989059363e-10,
        -1.2341341493329110e-05
    };

    expectVectorNear(frame.up, expectedUp, nearPoleTolerance, "Near-pole Up basis");
    expectVectorNear(frame.east, expectedEast, nearPoleTolerance, "Near-pole East basis");
    expectVectorNear(frame.north, expectedNorth, nearPoleTolerance, "Near-pole North basis");

    expectOrthonormalRightHanded(frame, nearPoleTolerance);
}

TEST(VER_FRM_003_LocalTransformation, ENUtoMCMFPositionAndVelocity)
{
    CoordinateTransformer transformer;

    // Origin: lat = 30 deg, lon = 45 deg, altitude = 0 m
    //
    // origin.position =
    // {1063935.869777871, 1063935.869777871, 868700.0} m
    //
    // Use a non-zero origin velocity to verify relative velocity translation:
    // origin.velocity = {1, 2, 3} m/s
    //
    // Local ENU target:
    // position = {100, 200, 300} m
    // velocity = {4, -5, 6} m/s
    //
    // Analytical MCMF result:
    // position = {1063978.160152337, 1064119.581508575, 869023.205080757} m
    // velocity = {3.613574442394, 10.270428690758, 1.669872981078} m/s

    const CoordinateTransformer::MoonSurfaceCoordinates site{
        30.0 * PI / 180.0,
        45.0 * PI / 180.0,
        0.0
    };

    auto origin = transformer.MSCtoMCMF(site);
    origin.velocity = {1.0, 2.0, 3.0};

    const auto frame = transformer.computeENUFrame(origin);

    const CoordinateTransformer::State local{
        {100.0, 200.0, 300.0},
        {4.0, -5.0, 6.0}
    };

    const auto fixed = transformer.ENUtoMCMF(local, frame);

    expectVectorNear(
        fixed.position,
        {1063978.160152337, 1064119.581508575, 869023.205080757},
        cartesianTolerance,
        "ENU-to-MCMF position");

    expectVectorNear(
        fixed.velocity,
        {3.613574442394, 10.270428690758, 1.669872981078},
        1e-9,
        "ENU-to-MCMF velocity");
}

TEST(VER_FRM_003_LocalTransformation, MCMFtoENUPositionAndVelocity)
{
    CoordinateTransformer transformer;

    const CoordinateTransformer::MoonSurfaceCoordinates site{
        30.0 * PI / 180.0,
        45.0 * PI / 180.0,
        0.0
    };

    auto origin = transformer.MSCtoMCMF(site);
    origin.velocity = {1.0, 2.0, 3.0};

    const auto frame = transformer.computeENUFrame(origin);

    const CoordinateTransformer::State fixed{
        {1063978.160152337, 1064119.581508575, 869023.205080757},
        {3.613574442394, 10.270428690758, 1.669872981078}
    };

    const auto local = transformer.MCMFtoENU(fixed, frame);

    expectVectorNear(
        local.position,
        {100.0, 200.0, 300.0},
        cartesianTolerance,
        "MCMF-to-ENU position");

    expectVectorNear(
        local.velocity,
        {4.0, -5.0, 6.0},
        1e-9,
        "MCMF-to-ENU velocity");
}

TEST(VER_FRM_003_Roundtrip, ENUtoMCMFtoENU)
{
    CoordinateTransformer transformer;

    const CoordinateTransformer::MoonSurfaceCoordinates site{
        30.0 * PI / 180.0,
        45.0 * PI / 180.0,
        250.0
    };

    auto origin = transformer.MSCtoMCMF(site);
    origin.velocity = {-2.0, 3.0, 1.5};

    const auto frame = transformer.computeENUFrame(origin);

    const CoordinateTransformer::State original{
        {125.0, -80.0, 450.0},
        {-3.0, 7.0, 2.0}
    };

    const auto fixed = transformer.ENUtoMCMF(original, frame);
    const auto reconstructed = transformer.MCMFtoENU(fixed, frame);

    expectVectorNear(
        reconstructed.position,
        original.position,
        cartesianTolerance,
        "ENU-MCMF-ENU roundtrip position");

    expectVectorNear(
        reconstructed.velocity,
        original.velocity,
        velocityTolerance,
        "ENU-MCMF-ENU roundtrip velocity");
}

TEST(VER_FRM_003_Roundtrip, MCMFtoENUtoMCMF)
{
    CoordinateTransformer transformer;

    const CoordinateTransformer::MoonSurfaceCoordinates site{
        -20.0 * PI / 180.0,
        70.0 * PI / 180.0,
        500.0
    };

    auto origin = transformer.MSCtoMCMF(site);
    origin.velocity = {5.0, -4.0, 2.0};

    const auto frame = transformer.computeENUFrame(origin);

    const CoordinateTransformer::State original{
        origin.position + Eigen::Vector3d{250.0, -125.0, 75.0},
        {-8.0, 6.0, 4.0}
    };

    const auto local = transformer.MCMFtoENU(original, frame);
    const auto reconstructed = transformer.ENUtoMCMF(local, frame);

    expectVectorNear(
        reconstructed.position,
        original.position,
        cartesianTolerance,
        "MCMF-ENU-MCMF roundtrip position");

    expectVectorNear(
        reconstructed.velocity,
        original.velocity,
        velocityTolerance,
        "MCMF-ENU-MCMF roundtrip velocity");
}
