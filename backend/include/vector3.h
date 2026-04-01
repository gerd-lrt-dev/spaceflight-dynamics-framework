#ifndef VECTOR3
#define VECTOR3

#include <cmath>

struct Vector3 {
    double x, y, z;

    // Basic vector-vector operations 
    Vector3 operator+(const Vector3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    Vector3 operator-(const Vector3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    // Basic vector-scalar operations
    Vector3 operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    Vector3 operator/(double scalar) const {
        return {x / scalar, y / scalar, z / scalar};
    }

    //negation
    Vector3 operator-() const {
        return { -x, -y, -z };
    }

    // Sum
    Vector3& operator+=(const Vector3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    // Further vector operations
    double norm() const {
        return std::sqrt(x*x + y*y + z*z);
    }

    Vector3 normalized() const {
        double n = norm();
        return n == 0 ? *this : *this * (1.0 / n);
    }

    Vector3 cross(const Vector3 &other) const {
        double x = this->y * other.z - this->z * other.y;
        double y = this->z * other.x - this->x * other.z;
        double z = this->x * other.y - this->y * other.x;

        return {x, y, z};
    }
};


#endif
