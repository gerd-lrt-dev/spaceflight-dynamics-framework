#ifndef QUATERNION_H
#define QUATERNION_H

#include "vector3.h"


/**
 * @class Quaternion
 * @brief Mathematical representation of a 3D rotation using unit quaternions.
 *
 * This class represents an orientation quaternion used to describe the
 * rotation between an inertial reference frame and a body-fixed frame.
 *
 * The quaternion is enforced to be a unit quaternion (‖q‖ = 1) at all times.
 * This invariant is critical because only unit quaternions represent pure
 * rotations without scaling. Any deviation from unit norm would lead to
 * non-orthogonal rotation matrices and unphysical behavior.
 *
 * The class is deliberately kept free of any physical or numerical integration
 * logic. It only encapsulates mathematical properties and guarantees internal
 * consistency. Time integration and dynamics are handled externally.
 */
class Quaternion
{
public:
    /**
     * @brief Default constructor creating the identity quaternion.
     *
     * The identity quaternion represents zero rotation:
     * q = (1, 0, 0, 0)
     */
    Quaternion();

    /**
     * @brief Constructs a quaternion from its components and normalizes it.
     *
     * The provided components do not need to form a unit quaternion.
     * The constructor enforces the unit-norm invariant internally.
     *
     * @param q0 Scalar component
     * @param q1 First vector component
     * @param q2 Second vector component
     * @param q3 Third vector component
     */
    Quaternion(double q0, double q1, double q2, double q3);

    /// @return Scalar component of the quaternion
    double getQ0() const;

    /// @return First vector component of the quaternion
    double getQ1() const;

    /// @return Second vector component of the quaternion
    double getQ2() const;

    /// @return Third vector component of the quaternion
    double getQ3() const;

    /**
     * @brief Computes the Euclidean norm of a quaternion.
     *
     * This function is used internally to enforce the unit-norm invariant.
     *
     * @param q0 Scalar component
     * @param q1 First vector component
     * @param q2 Second vector component
     * @param q3 Third vector component
     * @return Quaternion norm
     */
    double norm(double q0, double q1, double q2, double q3) const;

    /**
     * @brief Normalizes a single quaternion component.
     *
     * This helper function divides a quaternion component by the quaternion norm.
     *
     */
    void normalize();

    Vector3 rotate(const Vector3 &vector) const;

    Quaternion inverse() const;

    Quaternion operator*(const Quaternion &other) const;
private:
    /// Quaternion scalar component
    double q0_;

    /// Quaternion vector components
    double q1_;
    double q2_;
    double q3_;
};

#endif // QUATERNION_H
