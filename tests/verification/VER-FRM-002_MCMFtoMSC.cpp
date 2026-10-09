#include <gtest/gtest.h>

#include "Coordinates/coordinateTransformer.h"
#include "environmentConfig.h"

#include <cmath>

namespace
{

constexpr double PI = 3.14159265358979323846;

constexpr double angularTolerance   = 1e-12; // rad
constexpr double altitudeTolerance  = 1e-6;  // m
constexpr double cartesianTolerance = 1e-6;  // m

} // namespace

TEST(VER_FRM_002_MCMFtoMSC, ReferenceGeometry)
{
    CoordinateTransformer transformer;
    EnvironmentConfig econfig;

    //=========================================================================
    // Case 1 - Equator / Prime Meridian / Surface
    //=========================================================================
    //
    // Analytical reference:
    //
    // r_MCMF = { R_Moon, 0, 0 }
    //
    // r         = sqrt(x^2 + y^2 + z^2) = R_Moon
    // latitude  = asin(z / r)            = 0 rad
    // longitude = atan2(y, x)            = 0 rad
    // altitude  = r - R_Moon             = 0 m

    CoordinateTransformer::State startCase1{
        {econfig.radiusMoon, 0.0, 0.0},
        Eigen::Vector3d::Zero()
    };

    const CoordinateTransformer::MoonSurfaceCoordinates expectedCase1{
        0.0,
        0.0,
        0.0
    };

    const auto resultCase1 = transformer.MCMFtoMSC(startCase1);

    EXPECT_NEAR(resultCase1.latitude, expectedCase1.latitude, angularTolerance)
        << "CASE1: Latitude deviates from analytical reference.";
    EXPECT_NEAR(resultCase1.longitude, expectedCase1.longitude, angularTolerance)
        << "CASE1: Longitude deviates from analytical reference.";
    EXPECT_NEAR(resultCase1.altitude, expectedCase1.altitude, altitudeTolerance)
        << "CASE1: Altitude deviates from analytical reference.";

    //=========================================================================
    // Case 2 - Equator / +90 deg Longitude
    //=========================================================================
    //
    // r_MCMF = { 0, R_Moon, 0 }
    //
    // latitude  = 0 rad
    // longitude = +pi/2 rad
    // altitude  = 0 m

    CoordinateTransformer::State startCase2{
        {0.0, econfig.radiusMoon, 0.0},
        Eigen::Vector3d::Zero()
    };

    const CoordinateTransformer::MoonSurfaceCoordinates expectedCase2{
        0.0,
        PI / 2.0,
        0.0
    };

    const auto resultCase2 = transformer.MCMFtoMSC(startCase2);

    EXPECT_NEAR(resultCase2.latitude, expectedCase2.latitude, angularTolerance)
        << "CASE2: Latitude deviates from analytical reference.";
    EXPECT_NEAR(resultCase2.longitude, expectedCase2.longitude, angularTolerance)
        << "CASE2: Longitude deviates from analytical reference.";
    EXPECT_NEAR(resultCase2.altitude, expectedCase2.altitude, altitudeTolerance)
        << "CASE2: Altitude deviates from analytical reference.";

    //=========================================================================
    // Case 3 - North Pole
    //=========================================================================
    //
    // r_MCMF = { 0, 0, R_Moon }
    //
    // latitude = +pi/2 rad
    // altitude = 0 m
    //
    // Longitude is geometrically undefined at the pole and is therefore not
    // used as an acceptance quantity.

    CoordinateTransformer::State startCase3{
        {0.0, 0.0, econfig.radiusMoon},
        Eigen::Vector3d::Zero()
    };

    const auto resultCase3 = transformer.MCMFtoMSC(startCase3);

    EXPECT_NEAR(resultCase3.latitude, PI / 2.0, angularTolerance)
        << "CASE3: Latitude deviates from analytical reference.";
    EXPECT_NEAR(resultCase3.altitude, 0.0, altitudeTolerance)
        << "CASE3: Altitude deviates from analytical reference.";

    //=========================================================================
    // Case 4 - 1000 m above Equator / Prime Meridian
    //=========================================================================
    //
    // r_MCMF = { R_Moon + 1000 m, 0, 0 }
    //
    // latitude  = 0 rad
    // longitude = 0 rad
    // altitude  = 1000 m

    CoordinateTransformer::State startCase4{
        {econfig.radiusMoon + 1000.0, 0.0, 0.0},
        Eigen::Vector3d::Zero()
    };

    const CoordinateTransformer::MoonSurfaceCoordinates expectedCase4{
        0.0,
        0.0,
        1000.0
    };

    const auto resultCase4 = transformer.MCMFtoMSC(startCase4);

    EXPECT_NEAR(resultCase4.latitude, expectedCase4.latitude, angularTolerance)
        << "CASE4: Latitude deviates from analytical reference.";
    EXPECT_NEAR(resultCase4.longitude, expectedCase4.longitude, angularTolerance)
        << "CASE4: Longitude deviates from analytical reference.";
    EXPECT_NEAR(resultCase4.altitude, expectedCase4.altitude, altitudeTolerance)
        << "CASE4: Altitude deviates from analytical reference.";

    //=========================================================================
    // Case 5 - Equator / -90 deg Longitude
    //=========================================================================
    //
    // r_MCMF = { 0, -R_Moon, 0 }
    //
    // latitude  = 0 rad
    // longitude = -pi/2 rad
    // altitude  = 0 m

    CoordinateTransformer::State startCase5{
        {0.0, -econfig.radiusMoon, 0.0},
        Eigen::Vector3d::Zero()
    };

    const CoordinateTransformer::MoonSurfaceCoordinates expectedCase5{
        0.0,
        -PI / 2.0,
        0.0
    };

    const auto resultCase5 = transformer.MCMFtoMSC(startCase5);

    EXPECT_NEAR(resultCase5.latitude, expectedCase5.latitude, angularTolerance)
        << "CASE5: Latitude deviates from analytical reference.";
    EXPECT_NEAR(resultCase5.longitude, expectedCase5.longitude, angularTolerance)
        << "CASE5: Longitude deviates from analytical reference.";
    EXPECT_NEAR(resultCase5.altitude, expectedCase5.altitude, altitudeTolerance)
        << "CASE5: Altitude deviates from analytical reference.";

    //=========================================================================
    // Case 6 - South Pole
    //=========================================================================
    //
    // r_MCMF = { 0, 0, -R_Moon }
    //
    // latitude = -pi/2 rad
    // altitude = 0 m
    //
    // Longitude is geometrically undefined at the pole.

    CoordinateTransformer::State startCase6{
        {0.0, 0.0, -econfig.radiusMoon},
        Eigen::Vector3d::Zero()
    };

    const auto resultCase6 = transformer.MCMFtoMSC(startCase6);

    EXPECT_NEAR(resultCase6.latitude, -PI / 2.0, angularTolerance)
        << "CASE6: Latitude deviates from analytical reference.";
    EXPECT_NEAR(resultCase6.altitude, 0.0, altitudeTolerance)
        << "CASE6: Altitude deviates from analytical reference.";
}

TEST(VER_FRM_002_MCMFtoMSC, LongitudeWraparound)
{
    CoordinateTransformer transformer;
    EnvironmentConfig econfig;

    //=========================================================================
    // Case 7 - Longitude Wraparound
    //=========================================================================
    //
    // MSC input:
    // latitude  = 0
    // longitude = +pi
    // altitude  = 0
    //
    // Analytical MCMF position:
    // { -R_Moon, 0, 0 }
    //
    // atan2 may return +pi or -pi for the same physical meridian, therefore
    // the reverse transformation verifies |longitude| = pi.

    const CoordinateTransformer::MoonSurfaceCoordinates startCase7{
        0.0,
        PI,
        0.0
    };

    const auto resultMCMF = transformer.MSCtoMCMF(startCase7);

    EXPECT_NEAR(resultMCMF.position.x(), -econfig.radiusMoon, cartesianTolerance)
        << "CASE7: MCMF X deviates from analytical reference.";
    EXPECT_NEAR(resultMCMF.position.y(), 0.0, cartesianTolerance)
        << "CASE7: MCMF Y deviates from analytical reference.";
    EXPECT_NEAR(resultMCMF.position.z(), 0.0, cartesianTolerance)
        << "CASE7: MCMF Z deviates from analytical reference.";
    EXPECT_NEAR(resultMCMF.velocity.norm(), 0.0, cartesianTolerance)
        << "CASE7: MSCtoMCMF must return zero Moon-fixed velocity.";

    const auto resultMSC = transformer.MCMFtoMSC(resultMCMF);

    EXPECT_NEAR(resultMSC.latitude, 0.0, angularTolerance)
        << "CASE7: Reconstructed latitude deviates from analytical reference.";
    EXPECT_NEAR(std::abs(resultMSC.longitude), PI, angularTolerance)
        << "CASE7: Longitude wraparound is not equivalent to +/-pi.";
    EXPECT_NEAR(resultMSC.altitude, 0.0, altitudeTolerance)
        << "CASE7: Reconstructed altitude deviates from analytical reference.";
}

TEST(VER_FRM_002_MCMFtoMSC, NearPoleNumericalBehavior)
{
    CoordinateTransformer transformer;

    //=========================================================================
    // Case 8 - Near-Pole Numerical Behavior
    //=========================================================================
    //
    // MSC input:
    // latitude  = 89.999 deg
    // longitude = 45 deg
    // altitude  = 100 m
    //
    // R_Moon = 1,737,400 m
    // r      = 1,737,500 m
    //
    // Analytical reference:
    //
    // x = r cos(lat) cos(lon) = 21.443080846292236 m
    // y = r cos(lat) sin(lon) = 21.443080846292236 m
    // z = r sin(lat)          = 1737499.9997353635 m

    const CoordinateTransformer::MoonSurfaceCoordinates startCase8{
        89.999 * PI / 180.0,
        45.0 * PI / 180.0,
        100.0
    };

    const Eigen::Vector3d expectedPositionCase8{
        21.443080846292236,
        21.443080846292236,
        1737499.9997353635
    };

    const auto resultMCMF = transformer.MSCtoMCMF(startCase8);

    EXPECT_NEAR(resultMCMF.position.x(), expectedPositionCase8.x(), cartesianTolerance)
        << "CASE8: Near-pole MCMF X deviates from analytical reference.";
    EXPECT_NEAR(resultMCMF.position.y(), expectedPositionCase8.y(), cartesianTolerance)
        << "CASE8: Near-pole MCMF Y deviates from analytical reference.";
    EXPECT_NEAR(resultMCMF.position.z(), expectedPositionCase8.z(), cartesianTolerance)
        << "CASE8: Near-pole MCMF Z deviates from analytical reference.";

    const auto reconstructedMSC = transformer.MCMFtoMSC(resultMCMF);

    EXPECT_NEAR(reconstructedMSC.latitude, startCase8.latitude, angularTolerance)
        << "CASE8: Near-pole latitude does not survive roundtrip.";
    EXPECT_NEAR(reconstructedMSC.longitude, startCase8.longitude, angularTolerance)
        << "CASE8: Near-pole longitude does not survive roundtrip.";
    EXPECT_NEAR(reconstructedMSC.altitude, startCase8.altitude, altitudeTolerance)
        << "CASE8: Near-pole altitude does not survive roundtrip.";
}

TEST(VER_FRM_002_MSCtoMCMF, ReferenceGeometry)
{
    CoordinateTransformer transformer;
    EnvironmentConfig econfig;

    //=========================================================================
    // Case 9 - Equator / Prime Meridian / Surface
    //=========================================================================

    const CoordinateTransformer::MoonSurfaceCoordinates startCase9{
        0.0,
        0.0,
        0.0
    };

    const auto resultCase9 = transformer.MSCtoMCMF(startCase9);

    EXPECT_NEAR(resultCase9.position.x(), econfig.radiusMoon, cartesianTolerance)
        << "CASE9: MCMF X deviates from analytical reference.";
    EXPECT_NEAR(resultCase9.position.y(), 0.0, cartesianTolerance)
        << "CASE9: MCMF Y deviates from analytical reference.";
    EXPECT_NEAR(resultCase9.position.z(), 0.0, cartesianTolerance)
        << "CASE9: MCMF Z deviates from analytical reference.";
    EXPECT_NEAR(resultCase9.velocity.norm(), 0.0, cartesianTolerance)
        << "CASE9: MSCtoMCMF must return zero Moon-fixed velocity.";

    //=========================================================================
    // Case 10 - Equator / +90 deg Longitude
    //=========================================================================

    const CoordinateTransformer::MoonSurfaceCoordinates startCase10{
        0.0,
        PI / 2.0,
        0.0
    };

    const auto resultCase10 = transformer.MSCtoMCMF(startCase10);

    EXPECT_NEAR(resultCase10.position.x(), 0.0, cartesianTolerance)
        << "CASE10: MCMF X deviates from analytical reference.";
    EXPECT_NEAR(resultCase10.position.y(), econfig.radiusMoon, cartesianTolerance)
        << "CASE10: MCMF Y deviates from analytical reference.";
    EXPECT_NEAR(resultCase10.position.z(), 0.0, cartesianTolerance)
        << "CASE10: MCMF Z deviates from analytical reference.";
    EXPECT_NEAR(resultCase10.velocity.norm(), 0.0, cartesianTolerance)
        << "CASE10: MSCtoMCMF must return zero Moon-fixed velocity.";

    //=========================================================================
    // Case 11 - General Position
    //=========================================================================
    //
    // MSC:
    // latitude  = 30 deg
    // longitude = 45 deg
    // altitude  = 500 m
    //
    // r = 1,737,900 m
    //
    // Analytical reference:
    // x = 1064242.0559957214 m
    // y = 1064242.0559957214 m
    // z = 868950.0 m

    const CoordinateTransformer::MoonSurfaceCoordinates startCase11{
        30.0 * PI / 180.0,
        45.0 * PI / 180.0,
        500.0
    };

    const Eigen::Vector3d expectedPositionCase11{
        1064242.0559957214,
        1064242.0559957214,
        868950.0
    };

    const auto resultCase11 = transformer.MSCtoMCMF(startCase11);

    EXPECT_NEAR(resultCase11.position.x(), expectedPositionCase11.x(), cartesianTolerance)
        << "CASE11: MCMF X deviates from analytical reference.";
    EXPECT_NEAR(resultCase11.position.y(), expectedPositionCase11.y(), cartesianTolerance)
        << "CASE11: MCMF Y deviates from analytical reference.";
    EXPECT_NEAR(resultCase11.position.z(), expectedPositionCase11.z(), cartesianTolerance)
        << "CASE11: MCMF Z deviates from analytical reference.";
    EXPECT_NEAR(resultCase11.velocity.norm(), 0.0, cartesianTolerance)
        << "CASE11: MSCtoMCMF must return zero Moon-fixed velocity.";
}

TEST(VER_FRM_002_Roundtrip, MCMFtoMSCtoMCMF)
{
    CoordinateTransformer transformer;

    //=========================================================================
    // Case 12 - MCMF -> MSC -> MCMF
    //=========================================================================
    //
    // Use the independently calculated non-trivial reference position from
    // Case 11 so all Cartesian components are exercised.

    const CoordinateTransformer::State original{
        {1064242.0559957214, 1064242.0559957214, 868950.0},
        Eigen::Vector3d::Zero()
    };

    const auto msc = transformer.MCMFtoMSC(original);
    const auto reconstructed = transformer.MSCtoMCMF(msc);

    EXPECT_NEAR(reconstructed.position.x(), original.position.x(), cartesianTolerance)
        << "CASE12: Roundtrip MCMF X deviates from original.";
    EXPECT_NEAR(reconstructed.position.y(), original.position.y(), cartesianTolerance)
        << "CASE12: Roundtrip MCMF Y deviates from original.";
    EXPECT_NEAR(reconstructed.position.z(), original.position.z(), cartesianTolerance)
        << "CASE12: Roundtrip MCMF Z deviates from original.";
    EXPECT_NEAR(reconstructed.velocity.norm(), 0.0, cartesianTolerance)
        << "CASE12: Reconstructed Moon-fixed velocity must remain zero.";
}

TEST(VER_FRM_002_Roundtrip, MSCtoMCMFtoMSC)
{
    CoordinateTransformer transformer;

    //=========================================================================
    // Case 13 - MSC -> MCMF -> MSC
    //=========================================================================

    const CoordinateTransformer::MoonSurfaceCoordinates original{
        30.0 * PI / 180.0,
        45.0 * PI / 180.0,
        500.0
    };

    const auto mcmf = transformer.MSCtoMCMF(original);
    const auto reconstructed = transformer.MCMFtoMSC(mcmf);

    EXPECT_NEAR(reconstructed.latitude, original.latitude, angularTolerance)
        << "CASE13: Roundtrip latitude deviates from original.";
    EXPECT_NEAR(reconstructed.longitude, original.longitude, angularTolerance)
        << "CASE13: Roundtrip longitude deviates from original.";
    EXPECT_NEAR(reconstructed.altitude, original.altitude, altitudeTolerance)
        << "CASE13: Roundtrip altitude deviates from original.";
}
