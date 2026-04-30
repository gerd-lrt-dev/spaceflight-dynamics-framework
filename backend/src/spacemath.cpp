#include "spacemath.h"

double spacemath::kineticEnergy(double mass, double velocity)
{
    return 0.5 * mass * velocity * velocity;
}

double spacemath::accelerationBasedOnThrust(double thrust, double mass)
{
    return thrust / mass;
}

Vector3 spacemath::accelerationComplex(double currentThrust, double totalMass, Vector3 directionOfThrust, Vector3 gravityConstant)
{
    // F^_thrust = F_scalar * d^_thrust
    Vector3 F_thrust =  directionOfThrust * currentThrust;

    // F = m * a <-> a = F / m 
    return F_thrust / totalMass + gravityConstant;
}

Vector3 spacemath::calcAccelerationVector(double currentThrust, Vector3 thrustDirection)
{
    return thrustDirection * currentThrust;
}

double spacemath::calcMassFlowBasedOnThrust(double currenThrust, double Isp, double earthGravity)
{
    return currenThrust / (Isp * earthGravity); 
}
