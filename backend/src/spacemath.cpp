#include "spacemath.h"

double spacemath::kineticEnergy(double mass, double velocity)
{
    return 0.5 * mass * velocity * velocity;
}

double spacemath::accelerationBasedOnThrust(double thrust, double mass)
{
    return thrust / mass;
}

Eigen::Vector3d spacemath::accelerationComplex(double currentThrust, double totalMass, Eigen::Vector3d directionOfThrust, Eigen::Vector3d gravityConstant)
{
    // F^_thrust = F_scalar * d^_thrust
    Eigen::Vector3d F_thrust =  directionOfThrust * currentThrust;

    // F = m * a <-> a = F / m 
    return F_thrust / totalMass + gravityConstant;
}

Eigen::Vector3d spacemath::calcAccelerationVector(double currentThrust, Eigen::Vector3d thrustDirection)
{
    return thrustDirection * currentThrust;
}

double spacemath::calcMassFlowBasedOnThrust(double currenThrust, double Isp, double earthGravity)
{
    return currenThrust / (Isp * earthGravity); 
}
