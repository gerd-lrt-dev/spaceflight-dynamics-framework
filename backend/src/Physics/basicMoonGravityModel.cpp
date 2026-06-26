#include "Physics/basicMoonGravityModel.h"

//******************************************************
//************* PUBLIC *********************************
//******************************************************
Eigen::Vector3d BasicMoonGravityModel::computeAcceleration(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, double mass, const Eigen::Vector3d& thrust) const
{
    Eigen::Vector3d gravity = calcAccelerationAlignedToCenterOfMoon(pos);
    Eigen::Vector3d thrustAcc = thrust/mass;
    return gravity + thrustAcc;
}

//******************************************************
//************* PRIVATE ********************************
//******************************************************
Eigen::Vector3d BasicMoonGravityModel::calcAccelerationAlignedToCenterOfMoon(const Eigen::Vector3d& pos) const
{
    //Eigen::Vector3d dir = pos.normalized();
    //double r = pos.norm();

    Eigen::Vector3d r       = pos;
    double rNorm    = r.norm();
    double r3       = rNorm * rNorm * rNorm;

    return - r * (configData.muMoon / (r3));
}
