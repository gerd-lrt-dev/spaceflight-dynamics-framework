#include "physics.h"
#include <cmath>
#include <eigen3/Eigen/Dense>

// constructor - destructor ---------------------------------------

physics::~physics()
{
}

// public  ---------------------------------------------------------
Eigen::Vector3d physics::computeAcc(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, double mass, const Eigen::Vector3d& thrust) const
{
    return model_->computeAcceleration(pos, vel, mass, thrust);
}

Eigen::Vector3d physics::computeVel(const Eigen::Vector3d& vel, const Eigen::Vector3d& acc, double dt) const
{
    return integrator_->integrateVel(vel, acc, dt);
}

Eigen::Vector3d physics::computePos(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Vector3d& acc, double dt) const
{
    return integrator_->integratePos(pos, vel, acc, dt);
}

double physics::computeGLoad(const Eigen::Vector3d& totalAcceleration, const Eigen::Vector3d& gravityAcceleration, bool isLanded)
{
    return sensor_->computeGLoad(totalAcceleration, gravityAcceleration, isLanded);
}

