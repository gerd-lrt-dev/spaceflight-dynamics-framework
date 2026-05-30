#include "Coordinates/coordinateTransformer.h"

#include <cmath>

CoordinateTransformer::CoordinateTransformer() {}


//=========================================================
// MCI <-> MCMF
//=========================================================

CoordinateTransformer::State CoordinateTransformer::MCItoMCMF(
    const State& inertial,
    double t) const
{
    const double theta =
        MOON_ROTATION_RATE * t;

    Quaternion q(
        std::cos(theta * 0.5),
        0.0,
        0.0,
        std::sin(theta * 0.5)
        );

    Vector3 omega(
        0.0,
        0.0,
        MOON_ROTATION_RATE
        );

    State fixed;

    fixed.position =
        q.rotate(inertial.position);

    fixed.velocity =
        q.rotate(
            inertial.velocity
            - omega.cross(inertial.position)
            );

    return fixed;
}

CoordinateTransformer::State CoordinateTransformer::MCMFtoMCI(
    const State& fixed,
    double t) const
{
    const double theta =
        MOON_ROTATION_RATE * t;

    Quaternion q(
        std::cos(theta * 0.5),
        0.0,
        0.0,
        std::sin(theta * 0.5)
        );

    Quaternion qInv = q.inverse();

    Vector3 omega(
        0.0,
        0.0,
        MOON_ROTATION_RATE
        );

    State inertial;

    inertial.position =
        qInv.rotate(fixed.position);

    inertial.velocity =
        qInv.rotate(
            fixed.velocity
            + omega.cross(fixed.position)
            );

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

    const Vector3& r = fixed.position;

    const double radius = r.norm();

    msc.latitude =
        std::asin(r.z / radius);

    msc.longitude =
        std::atan2(r.y, r.x);

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

    fixed.position = Vector3(
        r * cosLat * cosLon,
        r * cosLat * sinLon,
        r * sinLat
        );

    // Stationary in Moon-fixed frame
    fixed.velocity = Vector3(
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
    Vector3 zAxis(
        0.0,
        0.0,
        1.0
        );

    // Detect pole singularity
    const double alignment =
        std::abs(frame.up.dot(zAxis));

    Vector3 referenceAxis;

    if (alignment > ALIGNMENT_THRESHOLD)
    {
        // Near poles:
        // use global X axis instead
        referenceAxis = Vector3(
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
        + frame.east  * targetENU.position.x
        + frame.north * targetENU.position.y
        + frame.up    * targetENU.position.z;

    fixed.velocity =
        frame.origin.velocity
        + frame.east  * targetENU.velocity.x
        + frame.north * targetENU.velocity.y
        + frame.up    * targetENU.velocity.z;

    return fixed;
}

CoordinateTransformer::State CoordinateTransformer::MCMFtoENU(
    const State& target,
    const ENUFrame& frame) const
{
    State enu;

    // Relative position
    Vector3 deltaPosition =
        target.position
        - frame.origin.position;

    enu.position = Vector3(
        deltaPosition.dot(frame.east),
        deltaPosition.dot(frame.north),
        deltaPosition.dot(frame.up)
        );

    // Relative velocity
    Vector3 deltaVelocity =
        target.velocity
        - frame.origin.velocity;

    enu.velocity = Vector3(
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

    Vector3 r = inertial.position;
    Vector3 v = inertial.velocity;

    // Local vertical (nadir)
    frame.down =
        (-r).normalized();

    // Orbital angular momentum
    Vector3 h =
        r.cross(v);

    const double hNorm =
        h.norm();

    // Degenerate trajectory handling
    if (hNorm < ANGULAR_MOMENTUM_EPSILON)
    {
        // Fallback reference direction
        Vector3 fallback(0.0, 0.0, 1.0);

        // Avoid parallel vectors
        if (std::abs(frame.down.dot(fallback)) > ALIGNMENT_THRESHOLD)
        {
            fallback = Vector3(
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
        + frame.forward * targetLVLH.position.x
        + frame.right   * targetLVLH.position.y
        + frame.down    * targetLVLH.position.z;

    // Velocity
    inertial.velocity =
        frame.origin.velocity
        + frame.forward * targetLVLH.velocity.x
        + frame.right   * targetLVLH.velocity.y
        + frame.down    * targetLVLH.velocity.z;

    return inertial;
}

CoordinateTransformer::State CoordinateTransformer::MCItoLVLH(
    const State& target,
    const LVLHFrame& frame) const
{
    State lvlh;

    // Relative position
    Vector3 deltaPosition =
        target.position
        - frame.origin.position;

    lvlh.position = Vector3(
        deltaPosition.dot(frame.forward),
        deltaPosition.dot(frame.right),
        deltaPosition.dot(frame.down)
        );

    // Relative velocity
    Vector3 deltaVelocity =
        target.velocity
        - frame.origin.velocity;

    lvlh.velocity = Vector3(
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

    Quaternion qInv =
        body.orientation.inverse();

    local.position =
        qInv.rotate(
            inertial.position
            - body.origin.position);

    local.velocity =
        qInv.rotate(
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
        + body.orientation.rotate(
            bodyState.position);

    inertial.velocity =
        body.origin.velocity
        + body.orientation.rotate(
            bodyState.velocity);

    return inertial;
}

Vector3 CoordinateTransformer::MCIAcctoSBFAcc(
    const Vector3& MCI_acc,
    const SpacecraftBodyFrame& body) const
{
    Quaternion qInv =
        body.orientation.inverse();

    return qInv.rotate(MCI_acc);
}

Vector3 CoordinateTransformer::SBFAcctoMCIAcc(
    const Vector3& SBF_acc,
    const SpacecraftBodyFrame& body) const
{
    return body.orientation.rotate(SBF_acc);
}