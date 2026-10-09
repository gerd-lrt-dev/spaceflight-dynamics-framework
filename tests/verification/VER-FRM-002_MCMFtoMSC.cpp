#include <gtest/gtest.h>

#include "Coordinates/coordinateTransformer.h"
#include "environmentConfig.h"

#include <memory>

TEST(VER_FRM_002_MCMFtoMSC, EquatorPrimeMeridianBasis)
{
    CoordinateTransformer transformer;
    EnvironmentConfig econfig;

    //*****************************************************************************
    //*************Case 1 - Equator / Prime Meridian / Surface*********************
    //*****************************************************************************

    // Conditions
    CoordinateTransformer::State startCase1;                            // Starting conditions
    CoordinateTransformer::MoonSurfaceCoordinates expectedResultCase1;  // Expected Result

    startCase1.position = {econfig.radiusMoon, 0.0 ,0.0};
    startCase1.velocity = Eigen::Vector3d::Zero();

    expectedResultCase1.latitude = 0.0;     // rad
    expectedResultCase1.longitude = 0.0;    // rad
    expectedResultCase1.altitude = 0.0;     // m

    // Tolerances
    constexpr double angularTolerance  = 1e-12; // rad
    constexpr double altitudeTolerance = 1e-6;  // m

    CoordinateTransformer::MoonSurfaceCoordinates resultCase1 = transformer.MCMFtoMSC(startCase1);

    // Verification
    EXPECT_NEAR(resultCase1.latitude, expectedResultCase1.latitude, angularTolerance)   << "Latitude attitude deviates from analytical reference.";
    EXPECT_NEAR(resultCase1.longitude, expectedResultCase1.longitude, angularTolerance) << "Longitude attitude deviates from analytical reference.";
    EXPECT_NEAR(resultCase1.altitude, expectedResultCase1.altitude, altitudeTolerance)  << "Altitude attitude deviates from analytical reference.";
}