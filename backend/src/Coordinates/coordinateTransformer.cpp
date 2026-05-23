#include "Coordinates/coordinateTransformer.h"

#include <cmath>

CoordinateTransformer::CoordinateTransformer() {}


//=========================================================
// MCI <-> MCMF
//=========================================================

Vector3 CoordinateTransformer::mciToMcmf(
    const Vector3& v,
    double theta) const
{
    double c = std::cos(theta);
    double s = std::sin(theta);

    return Vector3(
        c * v.x + s * v.y,
        -s * v.x + c * v.y,
        v.z
        );
}

Vector3 CoordinateTransformer::mcmfToMci(
    const Vector3& v,
    double theta) const
{
    double c = std::cos(theta);
    double s = std::sin(theta);

    return Vector3(
        c * v.x - s * v.y,
        s * v.x + c * v.y,
        v.z
        );
}


//=========================================================
// MCMF <-> MSC
//=========================================================

CoordinateTransformer::MoonSurfaceCoordinates
CoordinateTransformer::mcmfToMSC(
    const Vector3& position) const
{
    double r = position.norm();

    MoonSurfaceCoordinates msc;

    msc.latitude =
        std::asin(position.z / r);

    msc.longitude =
        std::atan2(position.y, position.x);

    msc.altitude =
        r - R_MOON;

    return msc;
}

Vector3 CoordinateTransformer::mscToMcmf(
    const MoonSurfaceCoordinates& msc) const
{
    double r = R_MOON + msc.altitude;

    double clat = std::cos(msc.latitude);

    return Vector3(
        r * clat * std::cos(msc.longitude),
        r * clat * std::sin(msc.longitude),
        r * std::sin(msc.latitude)
        );
}


//=========================================================
// ENU
//=========================================================

CoordinateTransformer::ENUFrame
CoordinateTransformer::computeENUFrame(
    const Vector3& positionMCMF) const
{
    Vector3 up =
        positionMCMF.normalized();

    Vector3 zAxis(0.0, 0.0, 1.0);

    Vector3 east =
        zAxis.cross(up);

    if (east.norm() < 1e-8)
    {
        east = Vector3(1.0, 0.0, 0.0);
    }

    east = east.normalized();

    Vector3 north =
        up.cross(east);

    return {east, north, up};
}

Vector3 CoordinateTransformer::enuToMcmf(
    const Vector3& vENU,
    const ENUFrame& enu) const
{
    return
        enu.east  * vENU.x +
        enu.north * vENU.y +
        enu.up    * vENU.z;
}

Vector3 CoordinateTransformer::mcmfToEnu(
    const Vector3& vMCMF,
    const ENUFrame& enu) const
{
    return Vector3(
        vMCMF.dot(enu.east),
        vMCMF.dot(enu.north),
        vMCMF.dot(enu.up)
        );
}


//=========================================================
// LVLH
//=========================================================

CoordinateTransformer::LVLHFrame
CoordinateTransformer::computeLVLHFrame(
    const State& state) const
{
    Vector3 r =
        state.position.normalized();

    Vector3 v =
        state.velocity.normalized();

    Vector3 h =
        state.position.cross(
                          state.velocity).normalized();

    Vector3 down = -r;

    Vector3 forward = v;

    Vector3 right =
        forward.cross(down).normalized();

    forward =
        down.cross(right).normalized();

    return {forward, right, down};
}

Vector3 CoordinateTransformer::lvlhToMci(
    const Vector3& vLVLH,
    const LVLHFrame& lvlh) const
{
    return
        lvlh.forward * vLVLH.x +
        lvlh.right   * vLVLH.y +
        lvlh.down    * vLVLH.z;
}

Vector3 CoordinateTransformer::mciToLvlh(
    const Vector3& vMCI,
    const LVLHFrame& lvlh) const
{
    return Vector3(
        vMCI.dot(lvlh.forward),
        vMCI.dot(lvlh.right),
        vMCI.dot(lvlh.down)
        );
}


//=========================================================
// Spacecraft Body Frame
// Quaternion convention:
// q_IB transforms inertial -> body
//=========================================================

Vector3 CoordinateTransformer::mciToSbf(
    const Vector3& vMCI,
    const Quaternion& q_IB) const
{
    return q_IB.rotate(vMCI);
}

Vector3 CoordinateTransformer::sbfToMci(
    const Vector3& vBody,
    const Quaternion& q_IB) const
{
    return q_IB.inverse().rotate(vBody);
}