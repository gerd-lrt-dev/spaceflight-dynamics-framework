#include "quaternion.h"
#include <cmath>

// ========================================
// Public constructors
// ========================================

Quaternion::Quaternion()
    : q0_(1.0), q1_(0.0), q2_(0.0), q3_(0.0)
{
}

Quaternion::Quaternion(
    double q0,
    double q1,
    double q2,
    double q3)
    : Quaternion(q0, q1, q2, q3, true)
{
}

// ========================================
// Private/internal constructor
// ========================================

Quaternion::Quaternion(
    double q0,
    double q1,
    double q2,
    double q3,
    bool normalizeQuaternion)
    : q0_(q0),
    q1_(q1),
    q2_(q2),
    q3_(q3)
{
    if (normalizeQuaternion)
    {
        normalize();
    }
}

double Quaternion::getQ0() const
{
    return q0_;
}

double Quaternion::getQ1() const
{
    return q1_;
}

double Quaternion::getQ2() const
{
    return q2_;
}

double Quaternion::getQ3() const
{
    return q3_;
}

//private
double Quaternion::norm(double q0, double q1, double q2, double q3) const
{
    return sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
}

void Quaternion::normalize()
{
    double qN = norm(q0_, q1_, q2_, q3_);

    if (qN == 0.0)
    {
        q0_ = 1.0;
        q1_ = 0.0;
        q2_ = 0.0;
        q3_ = 0.0;
        return;
    }

    q0_ /= qN;
    q1_ /= qN;
    q2_ /= qN;
    q3_ /= qN;
}

Vector3 Quaternion::rotate(const Vector3 &vector) const
{
    Quaternion vq(
        0.0,
        vector.x,
        vector.y,
        vector.z,
        false
        );

    Quaternion result =
        (*this) * vq * inverse();

    return Vector3(
        result.q1_,
        result.q2_,
        result.q3_
        );
}

Quaternion Quaternion::inverse() const
{
    return Quaternion(
        q0_,
        -q1_,
        -q2_,
        -q3_,
        false
        );
}

Quaternion Quaternion::operator*(const Quaternion &other) const
{
    return Quaternion(
        q0_ * other.q0_
            - q1_ * other.q1_
            - q2_ * other.q2_
            - q3_ * other.q3_,

        q0_ * other.q1_
            + q1_ * other.q0_
            + q2_ * other.q3_
            - q3_ * other.q2_,

        q0_ * other.q2_
            - q1_ * other.q3_
            + q2_ * other.q0_
            + q3_ * other.q1_,

        q0_ * other.q3_
            + q1_ * other.q2_
            - q2_ * other.q1_
            + q3_ * other.q0_,

        false
        );
}
