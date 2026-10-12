#include <gtest/gtest.h>

#include "Coordinates/coordinateTransformer.h"

#include <string>

namespace
{

constexpr double basisTolerance = 1e-12;
constexpr double stateTolerance = 1e-9;

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
    const CoordinateTransformer::LVLHFrame& frame)
{
    EXPECT_NEAR(frame.forward.norm(), 1.0, basisTolerance);
    EXPECT_NEAR(frame.right.norm(), 1.0, basisTolerance);
    EXPECT_NEAR(frame.down.norm(), 1.0, basisTolerance);

    EXPECT_NEAR(frame.forward.dot(frame.right), 0.0, basisTolerance);
    EXPECT_NEAR(frame.forward.dot(frame.down), 0.0, basisTolerance);
    EXPECT_NEAR(frame.right.dot(frame.down), 0.0, basisTolerance);

    // SDF LVLH convention:
    // forward x right = down
    expectVectorNear(
        frame.forward.cross(frame.right),
        frame.down,
        basisTolerance,
        "LVLH handedness forward x right = down");
}

} // namespace

TEST(VER_FRM_006_LVLH, PositionPositiveXVelocityPositiveY)
{
    CoordinateTransformer transformer;

    // Analytical reference:
    //
    // r = {+1, 0, 0}
    // v = {0, +1, 0}
    //
    // down  = -r_hat      = {-1, 0, 0}
    // h     = r x v       = {0, 0, +1}
    // right = -h_hat      = {0, 0, -1}
    // forward = right x down = {0, +1, 0}

    const CoordinateTransformer::State inertial{
        {1000.0, 0.0, 0.0},
        {0.0, 100.0, 0.0}
    };

    const auto frame = transformer.computeLVLHFrame(inertial);

    expectVectorNear(frame.forward, {0.0, 1.0, 0.0}, basisTolerance, "Forward");
    expectVectorNear(frame.right, {0.0, 0.0, -1.0}, basisTolerance, "Right");
    expectVectorNear(frame.down, {-1.0, 0.0, 0.0}, basisTolerance, "Down");

    expectVectorNear(
        frame.origin.position,
        inertial.position,
        basisTolerance,
        "LVLH origin position");

    expectVectorNear(
        frame.origin.velocity,
        inertial.velocity,
        basisTolerance,
        "LVLH origin velocity");

    expectOrthonormalRightHanded(frame);
}

TEST(VER_FRM_006_LVLH, PositionPositiveYVelocityNegativeX)
{
    CoordinateTransformer transformer;

    // Analytical reference:
    //
    // r = {0, +1, 0}
    // v = {-1, 0, 0}
    //
    // down    = {0, -1, 0}
    // h       = {0, 0, +1}
    // right   = {0, 0, -1}
    // forward = {-1, 0, 0}

    const CoordinateTransformer::State inertial{
        {0.0, 1500.0, 0.0},
        {-80.0, 0.0, 0.0}
    };

    const auto frame = transformer.computeLVLHFrame(inertial);

    expectVectorNear(frame.forward, {-1.0, 0.0, 0.0}, basisTolerance, "Forward");
    expectVectorNear(frame.right, {0.0, 0.0, -1.0}, basisTolerance, "Right");
    expectVectorNear(frame.down, {0.0, -1.0, 0.0}, basisTolerance, "Down");

    expectOrthonormalRightHanded(frame);
}

TEST(VER_FRM_006_LVLH, ArbitraryNonCollinearState)
{
    CoordinateTransformer transformer;

    // Arbitrary non-collinear reference:
    //
    // r = {2, 3, 4}
    // v = {-1, 5, 2}
    //
    // Frozen analytical basis:
    //
    // down =
    // {-0.3713906763541037,
    //  -0.5570860145311556,
    //  -0.7427813527082074}
    //
    // right =
    // { 0.6759263694193746,
    //   0.3862436396682141,
    //  -0.6276459144608478}
    //
    // forward =
    // {-0.6365473341714539,
    //   0.7351673436909748,
    //  -0.2331018406825042}

    const CoordinateTransformer::State inertial{
        {2.0, 3.0, 4.0},
        {-1.0, 5.0, 2.0}
    };

    const auto frame = transformer.computeLVLHFrame(inertial);

    expectVectorNear(
        frame.down,
        {-0.3713906763541037,
         -0.5570860145311556,
         -0.7427813527082074},
        basisTolerance,
        "Arbitrary Down");

    expectVectorNear(
        frame.right,
        {0.6759263694193746,
         0.3862436396682141,
         -0.6276459144608478},
        basisTolerance,
        "Arbitrary Right");

    expectVectorNear(
        frame.forward,
        {-0.6365473341714539,
         0.7351673436909748,
         -0.2331018406825042},
        basisTolerance,
        "Arbitrary Forward");

    expectOrthonormalRightHanded(frame);
}

TEST(VER_FRM_006_LVLH, DegenerateRadialTrajectoryFallback)
{
    CoordinateTransformer transformer;

    // Degenerate case:
    //
    // r and v are parallel -> h = r x v = 0.
    //
    // Production fallback:
    // down = {-1, 0, 0}
    // reference axis = +Z
    // right = reference x down = {0, -1, 0}
    // forward = right x down = {0, 0, -1}
    //
    // This test documents the deterministic fallback convention. It does not
    // claim that an orbital plane exists for a radial trajectory.

    const CoordinateTransformer::State inertial{
        {1000.0, 0.0, 0.0},
        {25.0, 0.0, 0.0}
    };

    const auto frame = transformer.computeLVLHFrame(inertial);

    expectVectorNear(frame.down, {-1.0, 0.0, 0.0}, basisTolerance, "Fallback Down");
    expectVectorNear(frame.right, {0.0, -1.0, 0.0}, basisTolerance, "Fallback Right");
    expectVectorNear(frame.forward, {0.0, 0.0, -1.0}, basisTolerance, "Fallback Forward");

    expectOrthonormalRightHanded(frame);
}

TEST(VER_FRM_006_LVLH, DegeneratePoleAlignedFallback)
{
    CoordinateTransformer transformer;

    // Second degenerate branch:
    //
    // r is aligned with +Z, therefore down is -Z and the default +Z fallback
    // would be parallel to down. The implementation switches to +X.
    //
    // down = {0, 0, -1}
    // right = +X x down = {0, +1, 0}
    // forward = right x down = {-1, 0, 0}

    const CoordinateTransformer::State inertial{
        {0.0, 0.0, 1000.0},
        {0.0, 0.0, -10.0}
    };

    const auto frame = transformer.computeLVLHFrame(inertial);

    expectVectorNear(frame.down, {0.0, 0.0, -1.0}, basisTolerance, "Pole fallback Down");
    expectVectorNear(frame.right, {0.0, 1.0, 0.0}, basisTolerance, "Pole fallback Right");
    expectVectorNear(frame.forward, {-1.0, 0.0, 0.0}, basisTolerance, "Pole fallback Forward");

    expectOrthonormalRightHanded(frame);
}

TEST(VER_FRM_006_Projection, LVLHtoMCIKnownLocalOffset)
{
    CoordinateTransformer transformer;

    // Base orbit:
    // r = {1000, 0, 0}
    // v = {0, 100, 0}
    //
    // LVLH:
    // forward = +Y
    // right   = -Z
    // down    = -X
    //
    // Local position {10, 20, 30}:
    // delta r_MCI = 10*forward + 20*right + 30*down
    //             = {-30, +10, -20}
    //
    // -> position = {970, 10, -20}
    //
    // Local velocity {1, 2, 3}:
    // delta v_MCI = {-3, +1, -2}
    //
    // -> velocity = {-3, 101, -2}

    const CoordinateTransformer::State origin{
        {1000.0, 0.0, 0.0},
        {0.0, 100.0, 0.0}
    };

    const auto frame = transformer.computeLVLHFrame(origin);

    const CoordinateTransformer::State local{
        {10.0, 20.0, 30.0},
        {1.0, 2.0, 3.0}
    };

    const auto inertial = transformer.LVLHtoMCI(local, frame);

    expectVectorNear(
        inertial.position,
        {970.0, 10.0, -20.0},
        stateTolerance,
        "LVLH-to-MCI position");

    expectVectorNear(
        inertial.velocity,
        {-3.0, 101.0, -2.0},
        stateTolerance,
        "LVLH-to-MCI velocity");
}

TEST(VER_FRM_006_Projection, MCItoLVLHKnownOffset)
{
    CoordinateTransformer transformer;

    const CoordinateTransformer::State origin{
        {1000.0, 0.0, 0.0},
        {0.0, 100.0, 0.0}
    };

    const auto frame = transformer.computeLVLHFrame(origin);

    const CoordinateTransformer::State inertial{
        {970.0, 10.0, -20.0},
        {-3.0, 101.0, -2.0}
    };

    const auto local = transformer.MCItoLVLH(inertial, frame);

    expectVectorNear(
        local.position,
        {10.0, 20.0, 30.0},
        stateTolerance,
        "MCI-to-LVLH position");

    expectVectorNear(
        local.velocity,
        {1.0, 2.0, 3.0},
        stateTolerance,
        "MCI-to-LVLH velocity");
}

TEST(VER_FRM_006_Roundtrip, LVLHtoMCItoLVLH)
{
    CoordinateTransformer transformer;

    const CoordinateTransformer::State origin{
        {2500.0, -1200.0, 700.0},
        {15.0, 30.0, -8.0}
    };

    const auto frame = transformer.computeLVLHFrame(origin);

    const CoordinateTransformer::State original{
        {125.0, -40.0, 60.0},
        {-2.0, 5.0, 3.0}
    };

    const auto inertial = transformer.LVLHtoMCI(original, frame);
    const auto reconstructed = transformer.MCItoLVLH(inertial, frame);

    expectVectorNear(
        reconstructed.position,
        original.position,
        stateTolerance,
        "LVLH-MCI-LVLH roundtrip position");

    expectVectorNear(
        reconstructed.velocity,
        original.velocity,
        stateTolerance,
        "LVLH-MCI-LVLH roundtrip velocity");
}
