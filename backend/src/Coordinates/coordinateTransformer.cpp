#include "Coordinates/coordinateTransformer.h"
#include <cmath>

CoordinateTransformer::CoordinateTransformer() {}
CoordinateTransformer::~CoordinateTransformer() {}

CoordinateTransformer::MoonSurfaceCoordinates
CoordinateTransformer::mciToMSC(const State &state) const {
    double r = state.position.norm();

    MoonSurfaceCoordinates msc;
    msc.latitude = std::asin(state.position.z/r);
    msc.longitude = std::atan2(state.position.y,state.position.x);
    msc.altitude = r - R_MOON;

    return msc;
}

CoordinateTransformer::State
CoordinateTransformer::mscToMCI(const MoonSurfaceCoordinates &msc) const {
    double r = R_MOON + msc.altitude;

    double x = r * std::cos(msc.latitude) * std::cos(msc.longitude);
    double y = r * std::cos(msc.latitude) * std::sin(msc.longitude);
    double z = r * std::sin(msc.latitude);

    State state;
    state.position = Vector3(x, y, z);
    state.velocity = Vector3(0.0, 0.0, 0.0);

    return state;
}

CoordinateTransformer::LocalNavigationFrame
CoordinateTransformer::mciToLNF(const State &state) const {
    Vector3 up = state.position.normalized();

    Vector3 east(-state.position.y, state.position.x, 0.0);
    east = east.normalized();

    Vector3 north = up.cross(east);

    return {up, east, north};
}

CoordinateTransformer::State
CoordinateTransformer::lnfToMCI(const State &refState, const Vector3 &localVector, const LocalNavigationFrame &lnf) const {
    Vector3 v_mci =
        lnf.north * localVector.x +
        lnf.east * localVector.y +
        lnf.up * localVector.z;

    State result;
    result.position = refState.position;
    result.velocity = v_mci;

    return result;
}

Vector3 CoordinateTransformer::mciToSBF(const Vector3 &vMCI, const Quaternion &attitude) const {
    return attitude.rotate(vMCI);
}

Vector3 CoordinateTransformer::sbfToMCI(const Vector3 &vBody, const Quaternion &attitude) const {
    return attitude.inverse().rotate(vBody);
}