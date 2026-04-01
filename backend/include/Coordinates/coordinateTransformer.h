#pragma once
#include "vector3.h"
#include "quaternion.h"


class  CoordinateTransformer {
public:

    struct State {
        Vector3 position;
        Vector3 velocity;
    };
    struct MoonSurfaceCoordinates {
        double latitude;
        double longitude;
        double altitude;
    };
    struct LocalNavigationFrame {
        Vector3 up;
        Vector3 east;
        Vector3 north;
    };

    CoordinateTransformer();
    ~CoordinateTransformer();

    MoonSurfaceCoordinates mciToMSC(const State &state) const;
    State mscToMCI(const MoonSurfaceCoordinates &msc) const;

    LocalNavigationFrame mciToLNF(const State &state) const;
    State lnfToMCI(const State &refState, const Vector3 &localVector, const LocalNavigationFrame &lnf) const;

    Vector3 mciToSBF(const Vector3 &vMCI, const Quaternion &attitude) const;
    Vector3 sbfToMCI(const Vector3 &vBody, const Quaternion &attitude) const;

private:
    static constexpr double R_MOON = 1737400.0;

};
