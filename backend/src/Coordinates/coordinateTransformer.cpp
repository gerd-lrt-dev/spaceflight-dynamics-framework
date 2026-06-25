#include "Coordinates/coordinateTransformer.h"

#include <cmath>

// Default and trivial Constructor & Destructor
CoordinateTransformer::CoordinateTransformer() {}

CoordinateTransformer::~CoordinateTransformer() = default;


//=========================================================
// MCI <-> MCMF
//=========================================================

CoordinateTransformer::State CoordinateTransformer::MCItoMCMF(
    const State& inertial,
    double t) const
{
    const double theta =
        MOON_ROTATION_RATE * t;

    Eigen::Quaterniond q(
        std::cos(theta * 0.5),
        0.0,
        0.0,
        std::sin(theta * 0.5)
        );

    Eigen::Vector3d omega(
        0.0,
        0.0,
        MOON_ROTATION_RATE
        );

    State fixed;

    fixed.position = q * inertial.position;

    fixed.velocity =
        q * (inertial.velocity - omega.cross(inertial.position));

    return fixed;
}

CoordinateTransformer::State CoordinateTransformer::MCMFtoMCI(
    const State& fixed,
    double t) const
{
    const double theta =
        MOON_ROTATION_RATE * t;

    Eigen::Quaterniond q(
        std::cos(theta * 0.5),
        0.0,
        0.0,
        std::sin(theta * 0.5)
        );

    Eigen::Quaterniond qInv = q.inverse();

    Eigen::Vector3d omega(
        0.0,
        0.0,
        MOON_ROTATION_RATE
        );

    State inertial;

    inertial.position = qInv * fixed.position;

    inertial.velocity = qInv * (fixed.velocity + omega.cross(fixed.position));

    return inertial;
}


//=========================================================
// MCMF <-> MSC
//=========================================================

CoordinateTransformer::MoonSurfaceCoordinates
CoordinateTransformer::MCMFtoMSC(
    const State& fixed) const
{
    MoonSurfaceCoordinates msc;

    const Eigen::Vector3d& r = fixed.position;

    const double radius = r.norm();

    msc.latitude =
        std::asin(r.z() / radius);

    msc.longitude =
        std::atan2(r.y(), r.x());

    msc.altitude =
        radius - R_MOON;

    return msc;
}

CoordinateTransformer::State CoordinateTransformer::MSCtoMCMF(
    const MoonSurfaceCoordinates& msc) const
{
    const double r =
        R_MOON + msc.altitude;

    const double cosLat =
        std::cos(msc.latitude);

    const double sinLat =
        std::sin(msc.latitude);

    const double cosLon =
        std::cos(msc.longitude);

    const double sinLon =
        std::sin(msc.longitude);

    State fixed;

    fixed.position = Eigen::Vector3d(
        r * cosLat * cosLon,
        r * cosLat * sinLon,
        r * sinLat
        );

    // Stationary in Moon-fixed frame
    fixed.velocity = Eigen::Vector3d(
        0.0,
        0.0,
        0.0
        );

    return fixed;
}


//=========================================================
// ENU
//=========================================================

CoordinateTransformer::ENUFrame
CoordinateTransformer::computeENUFrame(
    const State& fixed) const
{
    ENUFrame frame;

    frame.origin = fixed;

    // Local up direction
    frame.up =
        fixed.position.normalized();

    // Moon rotation axis
    Eigen::Vector3d zAxis(
        0.0,
        0.0,
        1.0
        );

    // Detect pole singularity
    const double alignment =
        std::abs(frame.up.dot(zAxis));

    Eigen::Vector3d referenceAxis;

    if (alignment > ALIGNMENT_THRESHOLD)
    {
        // Near poles:
        // use global X axis instead
        referenceAxis = Eigen::Vector3d(
            1.0,
            0.0,
            0.0
            );
    }
    else
    {
        // Normal case
        referenceAxis = zAxis;
    }

    // Local east direction
    frame.east =
        referenceAxis.cross(frame.up)
            .normalized();
    // Local north direction
    frame.north =
        frame.up.cross(frame.east)
            .normalized();

    return frame;
}

CoordinateTransformer::State CoordinateTransformer::ENUtoMCMF(
    const State& targetENU,
    const ENUFrame& frame) const
{
    State fixed;

    fixed.position =
        frame.origin.position
        + frame.east  * targetENU.position.x()
        + frame.north * targetENU.position.y()
        + frame.up    * targetENU.position.z();

    fixed.velocity =
        frame.origin.velocity
        + frame.east  * targetENU.velocity.x()
        + frame.north * targetENU.velocity.y()
        + frame.up    * targetENU.velocity.z();

    return fixed;
}

CoordinateTransformer::State CoordinateTransformer::MCMFtoENU(
    const State& target,
    const ENUFrame& frame) const
{
    State enu;

    // Relative position
    Eigen::Vector3d deltaPosition =
        target.position
        - frame.origin.position;

    enu.position = Eigen::Vector3d(
        deltaPosition.dot(frame.east),
        deltaPosition.dot(frame.north),
        deltaPosition.dot(frame.up)
        );

    // Relative velocity
    Eigen::Vector3d deltaVelocity =
        target.velocity
        - frame.origin.velocity;

    enu.velocity = Eigen::Vector3d(
        deltaVelocity.dot(frame.east),
        deltaVelocity.dot(frame.north),
        deltaVelocity.dot(frame.up)
        );

    return enu;
}


//=========================================================
// LVLH
//=========================================================

CoordinateTransformer::LVLHFrame
CoordinateTransformer::computeLVLHFrame(
    const State& inertial) const
{
    LVLHFrame frame;

    frame.origin = inertial;

    Eigen::Vector3d r = inertial.position;
    Eigen::Vector3d v = inertial.velocity;

    // Local vertical (nadir)
    frame.down =
        (-r).normalized();

    // Orbital angular momentum
    Eigen::Vector3d h =
        r.cross(v);

    const double hNorm =
        h.norm();

    // Degenerate trajectory handling
    if (hNorm < ANGULAR_MOMENTUM_EPSILON)
    {
        // Fallback reference direction
        Eigen::Vector3d fallback(0.0, 0.0, 1.0);

        // Avoid parallel vectors
        if (std::abs(frame.down.dot(fallback)) > ALIGNMENT_THRESHOLD)
        {
            fallback = Eigen::Vector3d(
                1.0,
                0.0,
                0.0
                );
        }

        frame.right =
            fallback.cross(frame.down)
                .normalized();
    }
    else
    {
        frame.right =
            (-h).normalized();
    }

    frame.forward =
        frame.right.cross(frame.down).normalized();

    return frame;
}

CoordinateTransformer::State CoordinateTransformer::LVLHtoMCI(
    const State& targetLVLH,
    const LVLHFrame& frame) const
{
    State inertial;

    // Position
    inertial.position =
        frame.origin.position
        + frame.forward * targetLVLH.position.x()
        + frame.right   * targetLVLH.position.y()
        + frame.down    * targetLVLH.position.z();

    // Velocity
    inertial.velocity =
        frame.origin.velocity
        + frame.forward * targetLVLH.velocity.x()
        + frame.right   * targetLVLH.velocity.y()
        + frame.down    * targetLVLH.velocity.z();

    return inertial;
}

CoordinateTransformer::State CoordinateTransformer::MCItoLVLH(
    const State& target,
    const LVLHFrame& frame) const
{
    State lvlh;

    // Relative position
    Eigen::Vector3d deltaPosition =
        target.position
        - frame.origin.position;

    lvlh.position = Eigen::Vector3d(
        deltaPosition.dot(frame.forward),
        deltaPosition.dot(frame.right),
        deltaPosition.dot(frame.down)
        );

    // Relative velocity
    Eigen::Vector3d deltaVelocity =
        target.velocity
        - frame.origin.velocity;

    lvlh.velocity = Eigen::Vector3d(
        deltaVelocity.dot(frame.forward),
        deltaVelocity.dot(frame.right),
        deltaVelocity.dot(frame.down)
        );

    return lvlh;
}


//=========================================================
// Spacecraft Body Frame
//=========================================================

CoordinateTransformer::State CoordinateTransformer::MCItoSBF(
    const State& inertial,
    const SpacecraftBodyFrame& body) const
{
    State local;

    Eigen::Quaterniond qInv =
        body.orientation.inverse();

    local.position =
        qInv * (
            inertial.position
            - body.origin.position);

    local.velocity =
        qInv * (
            inertial.velocity
            - body.origin.velocity);

    return local;
}

CoordinateTransformer::State CoordinateTransformer::SBFtoMCI(
    const State& bodyState,
    const SpacecraftBodyFrame& body) const
{
    State inertial;

    inertial.position =
        body.origin.position
        + body.orientation * bodyState.position;

    inertial.velocity =
        body.origin.velocity
        + body.orientation * bodyState.velocity;

    return inertial;
}

Eigen::Vector3d CoordinateTransformer::GenMCItoSBF(
    const Eigen::Vector3d& MCI_vec,
    const SpacecraftBodyFrame& body) const
{
    Eigen::Quaterniond qInv =
        body.orientation.inverse();

    return qInv * (MCI_vec);
}

Eigen::Vector3d CoordinateTransformer::GenSBFtoMCI(
    const Eigen::Vector3d& SBF_vec,
    const SpacecraftBodyFrame& body) const
{
    return body.orientation * (SBF_vec);
}
