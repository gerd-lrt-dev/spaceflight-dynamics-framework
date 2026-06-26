#include "Integrators/eulerIntegrator.h"

Eigen::Vector3d EulerIntegrator::integrateVel(const Eigen::Vector3d& vel, const Eigen::Vector3d& acc, double dt) const
{
    return vel + acc * dt;
}

Eigen::Vector3d EulerIntegrator::integratePos(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Vector3d& acc, double dt) const
{
    return pos + vel * dt + acc * 0.5 * dt * dt;
}
