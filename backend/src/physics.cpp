#include "physics.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <iomanip>
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
    return integrator_->integrateFirstOrder(vel, acc, dt);
}

Eigen::Vector3d physics::computePos(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Vector3d& acc, double dt) const
{
    return integrator_->integrateSecondOrder(pos, vel, acc, dt);
}

Eigen::Vector3d physics::computeAngAcc(const Eigen::Vector3d& SBF_angularVelocity, const Eigen::Matrix3d& SBF_inertia, const Eigen::Vector3d& SBF_torque) const
{
    Eigen::Vector3d angularAcceleration = Eigen::Vector3d::Zero();
    angularAcceleration = rotModel_->computeAngularAcceleration(SBF_angularVelocity, SBF_inertia, SBF_torque);

    return angularAcceleration;
}

Eigen::Vector3d physics::computeAngVel(const Eigen::Vector3d& SBF_angularVelocity, const Eigen::Vector3d& SBF_angularAcceleration, double dt) const
{
    return integrator_->integrateFirstOrder(SBF_angularVelocity, SBF_angularAcceleration, dt);
}

Eigen::Quaterniond physics::computeAttitude(const Eigen::Quaterniond& attitude, const Eigen::Vector3d& SBF_angularVelocity, double dt) const
{
    return integrator_->integrateQuaternion(attitude, SBF_angularVelocity, dt);
}

double physics::computeGLoad(const Eigen::Vector3d& totalAcceleration, const Eigen::Vector3d& gravityAcceleration, bool isLanded)
{
    return sensor_->computeGLoad(totalAcceleration, gravityAcceleration, isLanded);
}

