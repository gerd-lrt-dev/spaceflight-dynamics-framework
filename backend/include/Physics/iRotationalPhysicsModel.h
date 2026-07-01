#ifndef IROTATIONALPHYSICSMODEL_H
#define IROTATIONALPHYSICSMODEL_H

class IRotationalPhysicsModel {
public:
    virtual ~IRotationalPhysicsModel() = default;

    virtual Eigen::Vector3d computeAngularAcceleration(const Eigen::Vector3d& angularVelocity_B, const Eigen::Matrix3d& inertia_B, const Eigen::Vector3d& torque_B) const = 0;
};

#endif // IROTATIONALPHYSICSMODEL_H
