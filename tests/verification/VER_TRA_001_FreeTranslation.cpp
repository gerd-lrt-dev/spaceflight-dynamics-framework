#include <gtest/gtest.h>

#include "physics.h"
#include "Integrators/eulerIntegrator.h"

#include <memory>

class ZeroAccelerationModel : public IPhysicsModel{
public:
    Eigen::Vector3d computeAcceleration(const Eigen::Vector3d&, const Eigen::Vector3d&, double, const Eigen::Vector3d&) const override
    {
        return Eigen::Vector3d::Zero();
    }
};

class DummyRotationalModel : public IRotationalPhysicsModel{
public:
    Eigen::Vector3d computeAngularAcceleration(const Eigen::Vector3d&, const Eigen::Matrix3d&, const Eigen::Vector3d&) const override
    {
        return Eigen::Vector3d::Zero();
    }
};

class DummySensor : public ISensor{
public:
    double computeGLoad(const Eigen::Vector3d&, const Eigen::Vector3d&, bool) const override
    {
        return 0.0;
    }
};

auto physicModel        = std::make_shared<ZeroAccelerationModel>();

auto rotationalModel    = std::make_shared<DummyRotationalModel>();

auto integrator         = std::make_shared<EulerIntegrator>();

auto sensor             = std::make_shared<DummySensor>();

physics physicsSystem(IPhysicsModel, rotationalModel, integrator, sensor);

