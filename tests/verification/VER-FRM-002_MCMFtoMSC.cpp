#include <gtest/gtest.h>

#include "Coordinates/coordinateTransformer.h"
#include "environmentConfig.h"

#include <memory>

TEST(VER_FRM_002_MCMFtoMSC, EquatorPrimeMeridianBasis)
{
    // Backend source
    CoordinateTransformer transformer;
    EnvironmentConfig econfig;


    // Tolerances
    constexpr double angularTolerance  = 1e-12; // rad
    constexpr double altitudeTolerance = 1e-6;  // m

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


    CoordinateTransformer::MoonSurfaceCoordinates resultCase1 = transformer.MCMFtoMSC(startCase1);

    // Verification
    EXPECT_NEAR(resultCase1.latitude, expectedResultCase1.latitude, angularTolerance)   << "CASE1: Latitude attitude deviates from analytical reference.";
    EXPECT_NEAR(resultCase1.longitude, expectedResultCase1.longitude, angularTolerance) << "CASE1: Longitude attitude deviates from analytical reference.";
    EXPECT_NEAR(resultCase1.altitude, expectedResultCase1.altitude, altitudeTolerance)  << "CASE1: Altitude attitude deviates from analytical reference.";


    //*****************************************************************************
    //*************Case 2 — Equator / +90° Longitude*******************************
    //*****************************************************************************

    // Conditions
    CoordinateTransformer::State startCase2;                            // Starting conditions
    CoordinateTransformer::MoonSurfaceCoordinates expectedResultCase2;  // Expected Result

    startCase2.position = {0.0, econfig.radiusMoon, 0.0};
    startCase2.velocity = Eigen::Vector3d::Zero();

    expectedResultCase2.latitude    = 0.0;      // rad
    expectedResultCase2.longitude   = M_PI/2;   // rad
    expectedResultCase2.altitude    = 0.0;      // m

    CoordinateTransformer::MoonSurfaceCoordinates resultCase2 = transformer.MCMFtoMSC(startCase2);

    // Verification
    EXPECT_NEAR(resultCase2.latitude, expectedResultCase2.latitude, angularTolerance)   << "CASE2: Latitude attitude deviates from analytical reference.";
    EXPECT_NEAR(resultCase2.longitude, expectedResultCase2.longitude, angularTolerance) << "CASE2: Longitude attitude deviates from analytical reference.";
    EXPECT_NEAR(resultCase2.altitude, expectedResultCase2.altitude, altitudeTolerance)  << "CASE2: Altitude attitude deviates from analytical reference.";

    //*****************************************************************************
    //*************Case 3 — North Pole*********************************************
    //*****************************************************************************

    // Conditions
    CoordinateTransformer::State startCase3;                            // Starting conditions
    CoordinateTransformer::MoonSurfaceCoordinates expectedResultCase3;  // Expected Result

    startCase3.position = {0.0, 0.0, econfig.radiusMoon};
    startCase3.velocity = Eigen::Vector3d::Zero();

    expectedResultCase3.latitude    = M_PI/2;   // rad
    expectedResultCase3.longitude   = 0.0;      // not used because longitude is not mathmatical specified!
    expectedResultCase3.altitude    = 0.0;      // m

    CoordinateTransformer::MoonSurfaceCoordinates resultCase3 = transformer.MCMFtoMSC(startCase3);

    // Verification
    EXPECT_NEAR(resultCase3.latitude, expectedResultCase3.latitude, angularTolerance)   << "CASE3: Latitude attitude deviates from analytical reference.";
    EXPECT_NEAR(resultCase3.altitude, expectedResultCase3.altitude, altitudeTolerance)  << "CASE3: Altitude attitude deviates from analytical reference.";

    //*****************************************************************************
    //*************Case 4 — 1000 m above equator***********************************
    //*****************************************************************************

    // Conditions
    CoordinateTransformer::State startCase4;                            // Starting conditions
    CoordinateTransformer::MoonSurfaceCoordinates expectedResultCase4;  // Expected Result

    startCase4.position = {econfig.radiusMoon +1000, 0.0, 0.0};
    startCase4.velocity = Eigen::Vector3d::Zero();

    expectedResultCase4.latitude    = 0.0;      // rad
    expectedResultCase4.longitude   = 0.0;      // rad
    expectedResultCase4.altitude    = 1000.0;   // m

    CoordinateTransformer::MoonSurfaceCoordinates resultCase4 = transformer.MCMFtoMSC(startCase4);

    // Verification
    EXPECT_NEAR(resultCase4.latitude, expectedResultCase4.latitude, angularTolerance)   << "CASE4: Latitude attitude deviates from analytical reference.";
    EXPECT_NEAR(resultCase4.longitude, expectedResultCase4.longitude, angularTolerance)   << "CASE4: Longitude attitude deviates from analytical reference.";
    EXPECT_NEAR(resultCase4.altitude, expectedResultCase4.altitude, altitudeTolerance)  << "CASE4: Altitude attitude deviates from analytical reference.";


}