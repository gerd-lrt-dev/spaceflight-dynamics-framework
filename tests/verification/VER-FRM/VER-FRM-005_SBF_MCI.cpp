#include <gtest/gtest.h>

#include "Coordinates/coordinateTransformer.h"

#include <cmath>
#include <string>

namespace
{

constexpr double PI = 3.14159265358979323846;
constexpr double vectorTolerance = 1e-12;
constexpr double stateTolerance  = 1e-9;

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

CoordinateTransformer::SpacecraftBodyFrame makeBodyFrame(
    const Eigen::Quaterniond& orientation)
{
    CoordinateTransformer::SpacecraftBodyFrame body;
    body.orientation = orientation;
    body.origin.position = Eigen::Vector3d::Zero();
    body.origin.velocity = Eigen::Vector3d::Zero();
    return body;
}

} // namespace

TEST(VER_FRM_005_QuaternionConvention, IdentityQuaternion)
{
    CoordinateTransformer transformer;

    // SDF convention:
    // body.orientation rotates vectors from SBF -> MCI.
    //
    // Identity quaternion:
    // q = {w=1, x=0, y=0, z=0}
    //
    // Therefore all basis vectors remain unchanged in both directions.

    const auto body = makeBodyFrame(Eigen::Quaterniond::Identity());

    expectVectorNear(
        transformer.GenSBFtoMCI({1.0, 0.0, 0.0}, body),
        {1.0, 0.0, 0.0},
        vectorTolerance,
        "Identity SBF-to-MCI X basis");

    expectVectorNear(
        transformer.GenSBFtoMCI({0.0, 1.0, 0.0}, body),
        {0.0, 1.0, 0.0},
        vectorTolerance,
        "Identity SBF-to-MCI Y basis");

    expectVectorNear(
        transformer.GenSBFtoMCI({0.0, 0.0, 1.0}, body),
        {0.0, 0.0, 1.0},
        vectorTolerance,
        "Identity SBF-to-MCI Z basis");

    expectVectorNear(
        transformer.GenMCItoSBF({1.0, 2.0, 3.0}, body),
        {1.0, 2.0, 3.0},
        vectorTolerance,
        "Identity MCI-to-SBF vector");
}

TEST(VER_FRM_005_QuaternionConvention, Positive90DegAboutX)
{
    CoordinateTransformer transformer;

    // Active right-handed +90 deg rotation about SBF X:
    //
    // X -> X
    // Y -> Z
    // Z -> -Y

    const Eigen::Quaterniond q(
        Eigen::AngleAxisd(PI / 2.0, Eigen::Vector3d::UnitX()));

    const auto body = makeBodyFrame(q);

    expectVectorNear(
        transformer.GenSBFtoMCI({1.0, 0.0, 0.0}, body),
        {1.0, 0.0, 0.0},
        vectorTolerance,
        "+90 deg X: X basis");

    expectVectorNear(
        transformer.GenSBFtoMCI({0.0, 1.0, 0.0}, body),
        {0.0, 0.0, 1.0},
        vectorTolerance,
        "+90 deg X: Y basis");

    expectVectorNear(
        transformer.GenSBFtoMCI({0.0, 0.0, 1.0}, body),
        {0.0, -1.0, 0.0},
        vectorTolerance,
        "+90 deg X: Z basis");
}

TEST(VER_FRM_005_QuaternionConvention, Positive90DegAboutY)
{
    CoordinateTransformer transformer;

    // Active right-handed +90 deg rotation about SBF Y:
    //
    // X -> -Z
    // Y -> Y
    // Z -> X

    const Eigen::Quaterniond q(
        Eigen::AngleAxisd(PI / 2.0, Eigen::Vector3d::UnitY()));

    const auto body = makeBodyFrame(q);

    expectVectorNear(
        transformer.GenSBFtoMCI({1.0, 0.0, 0.0}, body),
        {0.0, 0.0, -1.0},
        vectorTolerance,
        "+90 deg Y: X basis");

    expectVectorNear(
        transformer.GenSBFtoMCI({0.0, 1.0, 0.0}, body),
        {0.0, 1.0, 0.0},
        vectorTolerance,
        "+90 deg Y: Y basis");

    expectVectorNear(
        transformer.GenSBFtoMCI({0.0, 0.0, 1.0}, body),
        {1.0, 0.0, 0.0},
        vectorTolerance,
        "+90 deg Y: Z basis");
}

TEST(VER_FRM_005_QuaternionConvention, Positive90DegAboutZ)
{
    CoordinateTransformer transformer;

    // Active right-handed +90 deg rotation about SBF Z:
    //
    // X -> Y
    // Y -> -X
    // Z -> Z

    const Eigen::Quaterniond q(
        Eigen::AngleAxisd(PI / 2.0, Eigen::Vector3d::UnitZ()));

    const auto body = makeBodyFrame(q);

    expectVectorNear(
        transformer.GenSBFtoMCI({1.0, 0.0, 0.0}, body),
        {0.0, 1.0, 0.0},
        vectorTolerance,
        "+90 deg Z: X basis");

    expectVectorNear(
        transformer.GenSBFtoMCI({0.0, 1.0, 0.0}, body),
        {-1.0, 0.0, 0.0},
        vectorTolerance,
        "+90 deg Z: Y basis");

    expectVectorNear(
        transformer.GenSBFtoMCI({0.0, 0.0, 1.0}, body),
        {0.0, 0.0, 1.0},
        vectorTolerance,
        "+90 deg Z: Z basis");
}

TEST(VER_FRM_005_QuaternionConvention, ArbitraryNormalizedQuaternion)
{
    CoordinateTransformer transformer;

    // Arbitrary normalized quaternion:
    //
    // q = normalize({w=0.7, x=0.2, y=-0.3, z=0.6})
    //   = {0.7071067811865476,
    //      0.20203050891044216,
    //     -0.30304576336566325,
    //      0.6060915267313265}
    //
    // Corresponding SBF -> MCI rotation matrix:
    //
    // [ 0.0816326530612246  -0.979591836734694  -0.183673469387755 ]
    // [ 0.734693877551020    0.183673469387755  -0.653061224489796 ]
    // [ 0.673469387755102   -0.0816326530612245  0.734693877551020 ]

    Eigen::Quaterniond q(0.7, 0.2, -0.3, 0.6);
    q.normalize();

    const auto body = makeBodyFrame(q);

    const Eigen::Vector3d sbfVector{2.0, -3.0, 4.0};

    const Eigen::Vector3d expectedMCI{
        2.367346938775510,
        -1.693877551020408,
        4.530612244897959
    };

    expectVectorNear(
        transformer.GenSBFtoMCI(sbfVector, body),
        expectedMCI,
        vectorTolerance,
        "Arbitrary quaternion SBF-to-MCI");
}

TEST(VER_FRM_005_QuaternionConvention, QuaternionSignEquivalence)
{
    CoordinateTransformer transformer;

    // q and -q represent the same physical rotation.

    Eigen::Quaterniond q(0.7, 0.2, -0.3, 0.6);
    q.normalize();

    const Eigen::Quaterniond qNegative(
        -q.w(),
        -q.x(),
        -q.y(),
        -q.z());

    const auto bodyPositive = makeBodyFrame(q);
    const auto bodyNegative = makeBodyFrame(qNegative);

    const Eigen::Vector3d sbfVector{3.0, -5.0, 7.0};

    const auto resultPositive =
        transformer.GenSBFtoMCI(sbfVector, bodyPositive);

    const auto resultNegative =
        transformer.GenSBFtoMCI(sbfVector, bodyNegative);

    expectVectorNear(
        resultPositive,
        resultNegative,
        vectorTolerance,
        "Quaternion sign equivalence");
}

TEST(VER_FRM_005_InverseTransformation, MCItoSBFReconstructsOriginalVector)
{
    CoordinateTransformer transformer;

    const Eigen::Quaterniond q(
        Eigen::AngleAxisd(PI / 3.0, Eigen::Vector3d(1.0, 2.0, 3.0).normalized()));

    const auto body = makeBodyFrame(q);

    const Eigen::Vector3d originalSBF{7.0, -4.0, 2.0};

    const auto mci =
        transformer.GenSBFtoMCI(originalSBF, body);

    const auto reconstructed =
        transformer.GenMCItoSBF(mci, body);

    expectVectorNear(
        reconstructed,
        originalSBF,
        vectorTolerance,
        "SBF-MCI-SBF inverse vector");
}

TEST(VER_FRM_005_StateTransformation, SBFtoMCIPositionAndVelocity)
{
    CoordinateTransformer transformer;

    // Verify full state transformation including body-frame origin offsets.
    //
    // Body orientation: +90 deg about Z.
    // body.origin.position = {1000, 2000, 3000} m
    // body.origin.velocity = {10, 20, 30} m/s
    //
    // SBF position {100, 200, 300} rotates to {-200, 100, 300}
    // -> MCI position = {800, 2100, 3300}
    //
    // SBF velocity {4, 5, 6} rotates to {-5, 4, 6}
    // -> MCI velocity = {5, 24, 36}

    CoordinateTransformer::SpacecraftBodyFrame body;
    body.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(PI / 2.0, Eigen::Vector3d::UnitZ()));
    body.origin.position = {1000.0, 2000.0, 3000.0};
    body.origin.velocity = {10.0, 20.0, 30.0};

    const CoordinateTransformer::State bodyState{
        {100.0, 200.0, 300.0},
        {4.0, 5.0, 6.0}
    };

    const auto inertial =
        transformer.SBFtoMCI(bodyState, body);

    expectVectorNear(
        inertial.position,
        {800.0, 2100.0, 3300.0},
        stateTolerance,
        "SBF-to-MCI state position");

    expectVectorNear(
        inertial.velocity,
        {5.0, 24.0, 36.0},
        stateTolerance,
        "SBF-to-MCI state velocity");
}

TEST(VER_FRM_005_StateTransformation, MCItoSBFPositionAndVelocity)
{
    CoordinateTransformer transformer;

    // Inverse of the previous analytical reference case.

    CoordinateTransformer::SpacecraftBodyFrame body;
    body.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(PI / 2.0, Eigen::Vector3d::UnitZ()));
    body.origin.position = {1000.0, 2000.0, 3000.0};
    body.origin.velocity = {10.0, 20.0, 30.0};

    const CoordinateTransformer::State inertial{
        {800.0, 2100.0, 3300.0},
        {5.0, 24.0, 36.0}
    };

    const auto bodyState =
        transformer.MCItoSBF(inertial, body);

    expectVectorNear(
        bodyState.position,
        {100.0, 200.0, 300.0},
        stateTolerance,
        "MCI-to-SBF state position");

    expectVectorNear(
        bodyState.velocity,
        {4.0, 5.0, 6.0},
        stateTolerance,
        "MCI-to-SBF state velocity");
}

TEST(VER_FRM_005_Roundtrip, SBFtoMCItoSBF)
{
    CoordinateTransformer transformer;

    CoordinateTransformer::SpacecraftBodyFrame body;
    body.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(
            0.73,
            Eigen::Vector3d(1.0, -2.0, 0.5).normalized()));
    body.origin.position = {-500.0, 1200.0, 350.0};
    body.origin.velocity = {8.0, -3.0, 1.5};

    const CoordinateTransformer::State original{
        {25.0, -60.0, 110.0},
        {-4.0, 9.0, 2.0}
    };

    const auto inertial =
        transformer.SBFtoMCI(original, body);

    const auto reconstructed =
        transformer.MCItoSBF(inertial, body);

    expectVectorNear(
        reconstructed.position,
        original.position,
        stateTolerance,
        "SBF-MCI-SBF roundtrip position");

    expectVectorNear(
        reconstructed.velocity,
        original.velocity,
        stateTolerance,
        "SBF-MCI-SBF roundtrip velocity");
}
