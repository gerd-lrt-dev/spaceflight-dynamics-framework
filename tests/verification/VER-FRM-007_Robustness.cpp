#include <gtest/gtest.h>

#include "Coordinates/coordinateTransformer.h"
#include "environmentConfig.h"

#include <cmath>
#include <string>

namespace
{

constexpr double PI = 3.14159265358979323846;

constexpr double positionTolerance = 1e-6;  // m
constexpr double velocityTolerance = 1e-9;  // m/s
constexpr double angularTolerance  = 1e-10; // rad
constexpr double altitudeTolerance = 1e-6;  // m
constexpr double repeatedDriftTolerance = 1e-5; // m

void expectVectorNear(
    const Eigen::Vector3d& actual,
    const Eigen::Vector3d& expected,
    double tolerance,
    const std::string& label)
{
    EXPECT_NEAR(actual.x(), expected.x(), tolerance)
        << label << " X component deviates.";
    EXPECT_NEAR(actual.y(), expected.y(), tolerance)
        << label << " Y component deviates.";
    EXPECT_NEAR(actual.z(), expected.z(), tolerance)
        << label << " Z component deviates.";
}

void expectVectorFinite(
    const Eigen::Vector3d& vector,
    const std::string& label)
{
    EXPECT_TRUE(std::isfinite(vector.x())) << label << " X is not finite.";
    EXPECT_TRUE(std::isfinite(vector.y())) << label << " Y is not finite.";
    EXPECT_TRUE(std::isfinite(vector.z())) << label << " Z is not finite.";
}

void expectStateFinite(
    const CoordinateTransformer::State& state,
    const std::string& label)
{
    expectVectorFinite(state.position, label + " position");
    expectVectorFinite(state.velocity, label + " velocity");
}

} // namespace

TEST(VER_FRM_007_Roundtrip, MCItoMCMFtoMCI)
{
    CoordinateTransformer transformer;

    constexpr double t = 456789.123;

    const CoordinateTransformer::State original{
        {1200000.0, -550000.0, 325000.0},
        {275.0, -135.0, 42.0}
    };

    const auto fixed = transformer.MCItoMCMF(original, t);
    const auto reconstructed = transformer.MCMFtoMCI(fixed, t);

    expectStateFinite(fixed, "MCI->MCMF intermediate state");
    expectStateFinite(reconstructed, "MCI->MCMF->MCI reconstructed state");

    expectVectorNear(
        reconstructed.position,
        original.position,
        positionTolerance,
        "MCI->MCMF->MCI position");

    expectVectorNear(
        reconstructed.velocity,
        original.velocity,
        velocityTolerance,
        "MCI->MCMF->MCI velocity");
}

TEST(VER_FRM_007_Roundtrip, MCMFtoMSCtoMCMF)
{
    CoordinateTransformer transformer;

    // Surface-coordinate conversion does not preserve arbitrary MCMF velocity.
    // This roundtrip therefore verifies geometry and the documented zero
    // Moon-fixed velocity returned by MSCtoMCMF.

    const CoordinateTransformer::State original{
        {1064242.0559957214, 1064242.0559957214, 868950.0},
        Eigen::Vector3d::Zero()
    };

    const auto msc = transformer.MCMFtoMSC(original);
    const auto reconstructed = transformer.MSCtoMCMF(msc);

    EXPECT_TRUE(std::isfinite(msc.latitude));
    EXPECT_TRUE(std::isfinite(msc.longitude));
    EXPECT_TRUE(std::isfinite(msc.altitude));
    expectStateFinite(reconstructed, "MCMF->MSC->MCMF reconstructed state");

    expectVectorNear(
        reconstructed.position,
        original.position,
        positionTolerance,
        "MCMF->MSC->MCMF position");

    expectVectorNear(
        reconstructed.velocity,
        Eigen::Vector3d::Zero(),
        velocityTolerance,
        "MCMF->MSC->MCMF velocity");
}

TEST(VER_FRM_007_Roundtrip, MCMFtoENUtoMCMF)
{
    CoordinateTransformer transformer;

    const CoordinateTransformer::MoonSurfaceCoordinates site{
        25.0 * PI / 180.0,
        -40.0 * PI / 180.0,
        120.0
    };

    auto origin = transformer.MSCtoMCMF(site);
    origin.velocity = {2.0, -3.0, 1.0};

    const auto frame = transformer.computeENUFrame(origin);

    const CoordinateTransformer::State original{
        origin.position + Eigen::Vector3d{300.0, -125.0, 80.0},
        {-7.0, 4.0, 2.5}
    };

    const auto local = transformer.MCMFtoENU(original, frame);
    const auto reconstructed = transformer.ENUtoMCMF(local, frame);

    expectStateFinite(local, "MCMF->ENU intermediate state");
    expectStateFinite(reconstructed, "MCMF->ENU->MCMF reconstructed state");

    expectVectorNear(
        reconstructed.position,
        original.position,
        positionTolerance,
        "MCMF->ENU->MCMF position");

    expectVectorNear(
        reconstructed.velocity,
        original.velocity,
        velocityTolerance,
        "MCMF->ENU->MCMF velocity");
}

TEST(VER_FRM_007_Roundtrip, SBFtoMCItoSBF)
{
    CoordinateTransformer transformer;

    CoordinateTransformer::SpacecraftBodyFrame body;
    body.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(
            1.1,
            Eigen::Vector3d(1.0, -2.0, 0.75).normalized()));
    body.origin.position = {5000.0, -1200.0, 800.0};
    body.origin.velocity = {12.0, -7.0, 3.5};

    const CoordinateTransformer::State original{
        {42.0, -18.0, 95.0},
        {-3.0, 6.0, 1.5}
    };

    const auto inertial = transformer.SBFtoMCI(original, body);
    const auto reconstructed = transformer.MCItoSBF(inertial, body);

    expectStateFinite(inertial, "SBF->MCI intermediate state");
    expectStateFinite(reconstructed, "SBF->MCI->SBF reconstructed state");

    expectVectorNear(
        reconstructed.position,
        original.position,
        positionTolerance,
        "SBF->MCI->SBF position");

    expectVectorNear(
        reconstructed.velocity,
        original.velocity,
        velocityTolerance,
        "SBF->MCI->SBF velocity");
}

TEST(VER_FRM_007_ChainedMissionFrames, MCItoMCMFtoENUtoMCMFtoMCI)
{
    CoordinateTransformer transformer;

    constexpr double t = 250000.0;

    const CoordinateTransformer::MoonSurfaceCoordinates site{
        -15.0 * PI / 180.0,
        65.0 * PI / 180.0,
        0.0
    };

    auto originMCMF = transformer.MSCtoMCMF(site);
    originMCMF.velocity = {0.5, -0.25, 0.1};
    const auto enuFrame = transformer.computeENUFrame(originMCMF);

    const CoordinateTransformer::State originalMCI{
        {1800000.0, 250000.0, -350000.0},
        {110.0, -65.0, 25.0}
    };

    const auto mcmf = transformer.MCItoMCMF(originalMCI, t);
    const auto enu = transformer.MCMFtoENU(mcmf, enuFrame);
    const auto mcmfReconstructed = transformer.ENUtoMCMF(enu, enuFrame);
    const auto mciReconstructed = transformer.MCMFtoMCI(mcmfReconstructed, t);

    expectStateFinite(enu, "Chained mission ENU state");
    expectStateFinite(mciReconstructed, "Chained mission reconstructed MCI state");

    expectVectorNear(
        mciReconstructed.position,
        originalMCI.position,
        positionTolerance,
        "MCI->MCMF->ENU->MCMF->MCI position");

    expectVectorNear(
        mciReconstructed.velocity,
        originalMCI.velocity,
        velocityTolerance,
        "MCI->MCMF->ENU->MCMF->MCI velocity");
}

TEST(VER_FRM_007_RepeatedCycles, MCI_MCMF_1000Cycles)
{
    CoordinateTransformer transformer;

    constexpr double t = 987654.321;
    constexpr int cycles = 1000;

    const CoordinateTransformer::State original{
        {1450000.0, -625000.0, 410000.0},
        {185.0, 92.0, -31.0}
    };

    auto state = original;

    for (int i = 0; i < cycles; ++i)
    {
        state = transformer.MCItoMCMF(state, t);
        state = transformer.MCMFtoMCI(state, t);

        ASSERT_TRUE(state.position.allFinite())
            << "MCI/MCMF repeated cycle produced non-finite position at cycle " << i;
        ASSERT_TRUE(state.velocity.allFinite())
            << "MCI/MCMF repeated cycle produced non-finite velocity at cycle " << i;
    }

    expectVectorNear(
        state.position,
        original.position,
        repeatedDriftTolerance,
        "MCI/MCMF 1000-cycle position drift");

    expectVectorNear(
        state.velocity,
        original.velocity,
        velocityTolerance,
        "MCI/MCMF 1000-cycle velocity drift");
}

TEST(VER_FRM_007_EdgeCases, HighAltitudeMSC)
{
    CoordinateTransformer transformer;

    // High-altitude reference:
    // latitude  = +12.5 deg
    // longitude = -170 deg
    // altitude  = 500 km
    //
    // Frozen analytical MCMF position:
    // {-2151179.2785040005,
    //   -379310.94714122434,
    //    484261.9922251114} m

    const CoordinateTransformer::MoonSurfaceCoordinates original{
        12.5 * PI / 180.0,
        -170.0 * PI / 180.0,
        500000.0
    };

    const auto fixed = transformer.MSCtoMCMF(original);
    const auto reconstructed = transformer.MCMFtoMSC(fixed);

    expectStateFinite(fixed, "High-altitude MCMF state");

    expectVectorNear(
        fixed.position,
        {-2151179.2785040005,
         -379310.94714122434,
          484261.9922251114},
        positionTolerance,
        "High-altitude analytical position");

    EXPECT_NEAR(reconstructed.latitude, original.latitude, angularTolerance)
        << "High-altitude latitude roundtrip deviates.";
    EXPECT_NEAR(reconstructed.longitude, original.longitude, angularTolerance)
        << "High-altitude longitude roundtrip deviates.";
    EXPECT_NEAR(reconstructed.altitude, original.altitude, altitudeTolerance)
        << "High-altitude altitude roundtrip deviates.";
}

TEST(VER_FRM_007_EdgeCases, NearZeroAltitude)
{
    CoordinateTransformer transformer;

    const CoordinateTransformer::MoonSurfaceCoordinates original{
        -33.0 * PI / 180.0,
        120.0 * PI / 180.0,
        1e-6
    };

    const auto fixed = transformer.MSCtoMCMF(original);
    const auto reconstructed = transformer.MCMFtoMSC(fixed);

    expectStateFinite(fixed, "Near-zero-altitude MCMF state");

    EXPECT_NEAR(reconstructed.latitude, original.latitude, angularTolerance)
        << "Near-zero-altitude latitude roundtrip deviates.";
    EXPECT_NEAR(reconstructed.longitude, original.longitude, angularTolerance)
        << "Near-zero-altitude longitude roundtrip deviates.";
    EXPECT_NEAR(reconstructed.altitude, original.altitude, altitudeTolerance)
        << "Near-zero-altitude altitude roundtrip deviates.";
}

TEST(VER_FRM_007_EdgeCases, NearPole)
{
    CoordinateTransformer transformer;

    const CoordinateTransformer::MoonSurfaceCoordinates original{
        89.999 * PI / 180.0,
        -135.0 * PI / 180.0,
        50.0
    };

    const auto fixed = transformer.MSCtoMCMF(original);
    const auto reconstructed = transformer.MCMFtoMSC(fixed);
    const auto enuFrame = transformer.computeENUFrame(fixed);

    expectStateFinite(fixed, "Near-pole MCMF state");
    expectVectorFinite(enuFrame.east, "Near-pole East basis");
    expectVectorFinite(enuFrame.north, "Near-pole North basis");
    expectVectorFinite(enuFrame.up, "Near-pole Up basis");

    EXPECT_NEAR(reconstructed.latitude, original.latitude, angularTolerance)
        << "Near-pole latitude roundtrip deviates.";
    EXPECT_NEAR(reconstructed.longitude, original.longitude, angularTolerance)
        << "Near-pole longitude roundtrip deviates.";
    EXPECT_NEAR(reconstructed.altitude, original.altitude, altitudeTolerance)
        << "Near-pole altitude roundtrip deviates.";
}

TEST(VER_FRM_007_EdgeCases, LongitudeWraparound)
{
    CoordinateTransformer transformer;

    const CoordinateTransformer::MoonSurfaceCoordinates positivePi{
        0.0,
        PI,
        100.0
    };

    const CoordinateTransformer::MoonSurfaceCoordinates negativePi{
        0.0,
        -PI,
        100.0
    };

    const auto fixedPositive = transformer.MSCtoMCMF(positivePi);
    const auto fixedNegative = transformer.MSCtoMCMF(negativePi);

    expectStateFinite(fixedPositive, "+pi longitude MCMF state");
    expectStateFinite(fixedNegative, "-pi longitude MCMF state");

    expectVectorNear(
        fixedPositive.position,
        fixedNegative.position,
        positionTolerance,
        "Longitude +pi/-pi physical equivalence");

    const auto reconstructed = transformer.MCMFtoMSC(fixedPositive);

    EXPECT_NEAR(std::abs(reconstructed.longitude), PI, angularTolerance)
        << "Longitude wraparound does not reconstruct to +/-pi.";
}

TEST(VER_FRM_007_Robustness, RepresentativeOutputsRemainFinite)
{
    CoordinateTransformer transformer;

    const CoordinateTransformer::State inertial{
        {1900000.0, -750000.0, 425000.0},
        {220.0, 80.0, -15.0}
    };

    const auto mcmf = transformer.MCItoMCMF(inertial, 400000.0);
    const auto msc = transformer.MCMFtoMSC(mcmf);
    const auto enuFrame = transformer.computeENUFrame(mcmf);
    const auto lvlhFrame = transformer.computeLVLHFrame(inertial);

    CoordinateTransformer::SpacecraftBodyFrame body;
    body.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(
            0.9,
            Eigen::Vector3d(0.5, 1.0, -0.25).normalized()));
    body.origin = inertial;

    const auto sbf = transformer.MCItoSBF(
        {
            inertial.position + Eigen::Vector3d{100.0, -50.0, 25.0},
            inertial.velocity + Eigen::Vector3d{1.0, 2.0, -1.0}
        },
        body);

    expectStateFinite(mcmf, "Representative MCMF state");

    EXPECT_TRUE(std::isfinite(msc.latitude));
    EXPECT_TRUE(std::isfinite(msc.longitude));
    EXPECT_TRUE(std::isfinite(msc.altitude));

    expectVectorFinite(enuFrame.east, "Representative ENU East");
    expectVectorFinite(enuFrame.north, "Representative ENU North");
    expectVectorFinite(enuFrame.up, "Representative ENU Up");

    expectVectorFinite(lvlhFrame.forward, "Representative LVLH Forward");
    expectVectorFinite(lvlhFrame.right, "Representative LVLH Right");
    expectVectorFinite(lvlhFrame.down, "Representative LVLH Down");

    expectStateFinite(sbf, "Representative SBF state");
}
