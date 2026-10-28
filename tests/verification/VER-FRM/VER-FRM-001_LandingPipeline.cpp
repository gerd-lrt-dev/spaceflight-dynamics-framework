#include <gtest/gtest.h>

#include "jsonConfigReader.h"
#include "spacecraft.h"

#include <cmath>
#include <string>

namespace
{

constexpr double R_MOON = 1737400.0;
constexpr double OMEGA_MOON = 2.6617e-6;

constexpr double positionTolerance = 1e-6;
constexpr double velocityTolerance = 1e-9;
constexpr double basisTolerance = 1e-12;
constexpr double angularTolerance = 1e-12;
constexpr double localLateralTolerance = 1e-4;

const char* referenceConfig()
{
    return R"json(
{
  "emptyMass": 1500.0,
  "fuelM": 0.0,

  "centerOfMass": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0
  },

  "Ixx": 850.0,
  "Iyy": 910.0,
  "Izz": 720.0,

  "initialState": {
    "positionFrame": "ENU",
    "velocityFrame": "ENU",

    "ENU_InitialPosition": {
      "east": 0.0,
      "north": 0.0,
      "up": 300.0
    },

    "ENU_InitialVelocity": {
      "east": 0.0,
      "north": 0.0,
      "up": -20.0
    },

    "IB_InitialOrientation": {
      "w": 1.0,
      "x": 0.0,
      "y": 0.0,
      "z": 0.0
    },

    "SBF_InitialCenterOfMass": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0
    }
  },

  "mission": {
    "landingSite": {
      "frame": "MSC",
      "latitude": 0.0,
      "longitude": 0.0,
      "altitude": 0.0
    }
  },

  "structuralIntegrity": 0.25,
  "safeVelocity": 5.0,

  "tanks": [],
  "engines": []
}
)json";
}

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

struct ReferenceSimulation
{
    customSpacecraft spacecraftConfig;
    MissionContext missionContext;
};

ReferenceSimulation parseReferenceConfiguration()
{
    const nlohmann::json json =
        nlohmann::json::parse(referenceConfig());

    return {
        jsonConfigReader::parseLander(json),
        jsonConfigReader::parseMissionContext(json)
    };
}

} // namespace

TEST(VER_FRM_001_LandingPipeline, ConfigurationParsing)
{
    // Stage 1:
    // JSON configuration -> strongly typed spacecraft and mission context.

    const auto parsed = parseReferenceConfiguration();

    EXPECT_EQ(parsed.spacecraftConfig.initialStateFrame_, InitialStateFrame::ENU);

    expectVectorNear(
        parsed.spacecraftConfig.ENU_initialState.position,
        {0.0, 0.0, 300.0},
        positionTolerance,
        "Configured ENU initial position");

    expectVectorNear(
        parsed.spacecraftConfig.ENU_initialState.velocity,
        {0.0, 0.0, -20.0},
        velocityTolerance,
        "Configured ENU initial velocity");

    EXPECT_NEAR(
        parsed.missionContext.MSC_LandingSite.latitude,
        0.0,
        angularTolerance);

    EXPECT_NEAR(
        parsed.missionContext.MSC_LandingSite.longitude,
        0.0,
        angularTolerance);

    EXPECT_NEAR(
        parsed.missionContext.MSC_LandingSite.altitude,
        0.0,
        positionTolerance);
}

TEST(VER_FRM_001_LandingPipeline, InitializationToAuthoritativeMCIState)
{
    // Stages 2-7:
    //
    // MSC landing site
    //   -> MCMF landing site
    //   -> ENU frame
    //   -> configured ENU spacecraft state
    //   -> MCMF spacecraft state
    //   -> MCI authoritative StateVector
    //   -> derived frame reconstruction
    //
    // At t0 = 0 the MCI and MCMF orientations coincide.
    //
    // Landing site:
    //   MCMF = {R_MOON, 0, 0}
    //
    // ENU basis:
    //   East  = {0, 1, 0}
    //   North = {0, 0, 1}
    //   Up    = {1, 0, 0}
    //
    // Spacecraft MCMF position:
    //   {R_MOON + 300, 0, 0}
    //
    // Local descent velocity in MCMF:
    //   {-20, 0, 0}
    //
    // MCMF -> MCI velocity at t0 additionally contains lunar surface
    // rotational velocity omega x r:
    //
    //   {0, OMEGA_MOON * (R_MOON + 300), 0}
    //
    // giving the frozen inertial reference velocity
    //   {-20, 4.62523609, 0} m/s.

    const auto parsed = parseReferenceConfiguration();
    spacecraft vehicle(parsed.spacecraftConfig, parsed.missionContext);

    const simData initial = vehicle.getFullSimulationData();

    expectVectorNear(
        initial.missionContext_.MCMF_landingSite.position,
        {R_MOON, 0.0, 0.0},
        positionTolerance,
        "Landing-site MSC->MCMF");

    expectVectorNear(
        initial.simFrameContext_.ENU_Frame.origin.position,
        {R_MOON, 0.0, 0.0},
        positionTolerance,
        "ENU frame origin");

    expectVectorNear(
        initial.simFrameContext_.ENU_Frame.east,
        {0.0, 1.0, 0.0},
        basisTolerance,
        "ENU East basis");

    expectVectorNear(
        initial.simFrameContext_.ENU_Frame.north,
        {0.0, 0.0, 1.0},
        basisTolerance,
        "ENU North basis");

    expectVectorNear(
        initial.simFrameContext_.ENU_Frame.up,
        {1.0, 0.0, 0.0},
        basisTolerance,
        "ENU Up basis");

    expectVectorNear(
        initial.statevector_.MCI_Position,
        {1737700.0, 0.0, 0.0},
        positionTolerance,
        "Authoritative initial MCI position");

    expectVectorNear(
        initial.statevector_.MCI_Velocity,
        {-20.0, 4.62523609, 0.0},
        velocityTolerance,
        "Authoritative initial MCI velocity");

    expectVectorNear(
        initial.simFrameContext_.MCI_State.position,
        initial.statevector_.MCI_Position,
        positionTolerance,
        "StateVector/FrameContext MCI position consistency");

    expectVectorNear(
        initial.simFrameContext_.MCI_State.velocity,
        initial.statevector_.MCI_Velocity,
        velocityTolerance,
        "StateVector/FrameContext MCI velocity consistency");

    expectVectorNear(
        initial.simFrameContext_.ENU_State.position,
        {0.0, 0.0, 300.0},
        positionTolerance,
        "Initial ENU roundtrip position");

    expectVectorNear(
        initial.simFrameContext_.ENU_State.velocity,
        {0.0, 0.0, -20.0},
        velocityTolerance,
        "Initial ENU roundtrip velocity");
}

TEST(VER_FRM_001_LandingPipeline, OneStepFreeFallMatchesAnalyticalReference)
{
    // Stages 8-10:
    // gravity -> one deterministic integration step -> derived frames.
    //
    // Reference state at t0:
    // r0 = {1737700, 0, 0} m
    // v0 = {-20, 4.62523609, 0} m/s
    //
    // No propulsion is configured.
    //
    // Lunar gravity:
    // a = -mu * r / |r|^3
    //
    // Frozen reference at r0:
    // ax = -1.6344355093979974 m/s^2
    //
    // Velocity after dt = 0.1 s:
    // v1 = v0 + a*dt
    //    = {-20.1634435509398, 4.62523609, 0}
    //
    // Position reference uses the intended second-order translational update:
    //
    // r1 = r0 + v0*dt + 0.5*a*dt^2
    //
    //    = {1737697.9918278225,
    //       0.462523609,
    //       0}
    //
    // This test intentionally uses the pre-step velocity in the position
    // equation. Passing an already-updated velocity into the second-order
    // integrator would count the acceleration contribution twice and must
    // therefore be detected by this end-to-end verification.

    constexpr double dt = 0.1;

    const auto parsed = parseReferenceConfiguration();
    spacecraft vehicle(parsed.spacecraftConfig, parsed.missionContext);

    vehicle.updateStep(dt);

    const simData after = vehicle.getFullSimulationData();

    const Eigen::Vector3d expectedVelocity{
        -20.1634435509398,
        4.62523609,
        0.0
    };

    const Eigen::Vector3d expectedPosition{
        1737697.9918278225,
        0.462523609,
        0.0
    };

    expectVectorNear(
        after.statevector_.MCI_Velocity,
        expectedVelocity,
        velocityTolerance,
        "One-step MCI velocity");

    expectVectorNear(
        after.statevector_.MCI_Position,
        expectedPosition,
        positionTolerance,
        "One-step MCI position");

    EXPECT_LT(after.statevector_.MCI_Position.x(), 1737700.0)
        << "Free fall must reduce radial MCI X position.";

    EXPECT_LT(after.statevector_.MCI_Velocity.x(), -20.0)
        << "Lunar gravity must increase the inward radial velocity magnitude.";

    expectVectorNear(
        after.simFrameContext_.MCI_State.position,
        after.statevector_.MCI_Position,
        positionTolerance,
        "Post-step StateVector/FrameContext MCI position");

    expectVectorNear(
        after.simFrameContext_.MCI_State.velocity,
        after.statevector_.MCI_Velocity,
        velocityTolerance,
        "Post-step StateVector/FrameContext MCI velocity");
}

TEST(VER_FRM_001_LandingPipeline, PostStepLandingSiteRelativeStateRemainsPhysical)
{
    // After one free-fall step, the spacecraft remains almost exactly above the
    // landing-site meridian. With zero thrust and zero commanded lateral motion,
    // no metre-scale East/North displacement may appear in 0.1 s.
    //
    // A very small lateral term is physically possible because the spacecraft
    // follows inertial dynamics while the surface frame rotates. The tolerance
    // therefore permits sub-millimetre numerical/kinematic effects but rejects
    // sign errors in the MCI/MCMF rotation convention.

    constexpr double dt = 0.1;

    const auto parsed = parseReferenceConfiguration();
    spacecraft vehicle(parsed.spacecraftConfig, parsed.missionContext);

    vehicle.updateStep(dt);

    const simData after = vehicle.getFullSimulationData();
    const auto& enu = after.simFrameContext_.ENU_State;

    EXPECT_NEAR(enu.position.x(), 0.0, localLateralTolerance)
        << "Unexpected East displacement after the free-fall reference step.";

    EXPECT_NEAR(enu.position.y(), 0.0, localLateralTolerance)
        << "Unexpected North displacement after the free-fall reference step.";

    EXPECT_LT(enu.position.z(), 300.0)
        << "Landing-site-relative Up position must decrease during free fall.";

    EXPECT_NEAR(enu.velocity.x(), 0.0, localLateralTolerance)
        << "Unexpected East velocity after the free-fall reference step.";

    EXPECT_NEAR(enu.velocity.y(), 0.0, localLateralTolerance)
        << "Unexpected North velocity after the free-fall reference step.";

    EXPECT_LT(enu.velocity.z(), -20.0)
        << "Landing-site-relative descent velocity must become more negative.";
}

TEST(VER_FRM_001_LandingPipeline, BackendTelemetrySnapshotIsInternallyConsistent)
{
    // Stage 12 backend boundary:
    //
    // getFullSimulationData() is the authoritative backend snapshot consumed
    // by the frontend TelemetryMapper and XML export path.
    //
    // The BUILD_FRONTEND=OFF verification target intentionally does not link
    // Qt-dependent DTO/XML code. This test therefore verifies the complete
    // backend telemetry boundary. DTO/XML serialization remains a separate
    // frontend-layer acceptance check.

    constexpr double dt = 0.1;

    const auto parsed = parseReferenceConfiguration();
    spacecraft vehicle(parsed.spacecraftConfig, parsed.missionContext);

    vehicle.updateStep(dt);

    const simData snapshot = vehicle.getFullSimulationData();

    EXPECT_NEAR(snapshot.time, dt, angularTolerance);

    expectVectorNear(
        snapshot.statevector_.MCI_Position,
        snapshot.simFrameContext_.MCI_State.position,
        positionTolerance,
        "Telemetry MCI position consistency");

    expectVectorNear(
        snapshot.statevector_.MCI_Velocity,
        snapshot.simFrameContext_.MCI_State.velocity,
        velocityTolerance,
        "Telemetry MCI velocity consistency");

    EXPECT_NEAR(
        snapshot.missionContext_.MSC_LandingSite.latitude,
        0.0,
        angularTolerance);

    EXPECT_NEAR(
        snapshot.missionContext_.MSC_LandingSite.longitude,
        0.0,
        angularTolerance);

    EXPECT_NEAR(
        snapshot.missionContext_.MSC_LandingSite.altitude,
        0.0,
        positionTolerance);

    expectVectorNear(
        snapshot.missionContext_.MCMF_landingSite.position,
        {R_MOON, 0.0, 0.0},
        positionTolerance,
        "Telemetry landing-site MCMF position");

    EXPECT_TRUE(snapshot.statevector_.MCI_Position.allFinite());
    EXPECT_TRUE(snapshot.statevector_.MCI_Velocity.allFinite());
    EXPECT_TRUE(snapshot.simFrameContext_.MCMF_State.position.allFinite());
    EXPECT_TRUE(snapshot.simFrameContext_.MCMF_State.velocity.allFinite());
    EXPECT_TRUE(snapshot.simFrameContext_.ENU_State.position.allFinite());
    EXPECT_TRUE(snapshot.simFrameContext_.ENU_State.velocity.allFinite());
    EXPECT_TRUE(snapshot.simFrameContext_.LVLH_State.position.allFinite());
    EXPECT_TRUE(snapshot.simFrameContext_.LVLH_State.velocity.allFinite());
}
