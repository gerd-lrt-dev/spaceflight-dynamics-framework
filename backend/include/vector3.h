#ifndef VECTOR3_H
#define VECTOR3_H

#include <cmath>

/**
 * @brief Three-dimensional vector representation.
 *
 * This structure provides basic mathematical operations for 3D vectors and is
 * used throughout the simulation framework to represent physical quantities
 * such as position, velocity, acceleration, and force.
 *
 * All operations follow standard Euclidean vector algebra.
 */
struct Vector3
{
    double x{0.0}; ///< X-component
    double y{0.0}; ///< Y-component
    double z{0.0}; ///< Z-component

    static constexpr double EPSILON = 1e-12; ///< Numerical tolerance used for small-magnitude checks

    // -------------------------------------------------------------------------
    // Vector-vector operations
    // -------------------------------------------------------------------------

    /**
     * @brief Vector addition.
     *
     * @param other Vector to add
     * @return Resulting vector
     */
    Vector3 operator+(const Vector3& other) const
    {
        return {x + other.x, y + other.y, z + other.z};
    }

    /**
     * @brief Vector subtraction.
     *
     * @param other Vector to subtract
     * @return Resulting vector
     */
    Vector3 operator-(const Vector3& other) const
    {
        return {x - other.x, y - other.y, z - other.z};
    }

    /**
     * @brief In-place vector addition.
     *
     * @param other Vector to add
     * @return Reference to this vector
     */
    Vector3& operator+=(const Vector3& other)
    {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    /**
     * @brief Unary negation.
     *
     * @return Vector with inverted sign
     */
    Vector3 operator-() const
    {
        return {-x, -y, -z};
    }

    // -------------------------------------------------------------------------
    // Vector-scalar operations
    // -------------------------------------------------------------------------

    /**
     * @brief Scalar multiplication.
     *
     * @param scalar Scalar value
     * @return Scaled vector
     */
    Vector3 operator*(double scalar) const
    {
        return {x * scalar, y * scalar, z * scalar};
    }

    /**
     * @brief Scalar division.
     *
     * @param scalar Scalar value
     * @return Scaled vector
     */
    Vector3 operator/(double scalar) const
    {
        return {x / scalar, y / scalar, z / scalar};
    }

    // -------------------------------------------------------------------------
    // Vector operations
    // -------------------------------------------------------------------------

    /**
     * @brief Computes the squared Euclidean norm.
     *
     * \f[
     * \|\mathbf{v}\|^2 = x^2 + y^2 + z^2
     * \f]
     *
     * This function avoids the square-root operation required by norm()
     * and is therefore useful in performance-sensitive calculations.
     *
     * @return Squared vector magnitude
     */
    double normSquared() const
    {
        return x * x + y * y + z * z;
    }

    /**
     * @brief Computes the Euclidean norm (magnitude).
     *
     * \f[
     * \|\mathbf{v}\| =
     * \sqrt{x^2 + y^2 + z^2}
     * \f]
     *
     * @return Vector magnitude
     */
    double norm() const
    {
        return std::sqrt(normSquared());
    }

    /**
     * @brief Returns a normalized unit-length vector.
     *
     * \f[
     * \hat{\mathbf{v}} =
     * \frac{\mathbf{v}}{\|\mathbf{v}\|}
     * \f]
     *
     * If the vector magnitude is smaller than EPSILON,
     * the zero vector is returned.
     *
     * @return Normalized vector
     */
    Vector3 normalized() const
    {
        double n = norm();

        if (n < EPSILON)
        {
            return {};
        }

        return *this / n;
    }

    /**
     * @brief Normalizes the vector in-place.
     *
     * If the vector magnitude is smaller than EPSILON,
     * the vector remains unchanged.
     */
    void normalize()
    {
        double n = norm();

        if (n < EPSILON)
        {
            return;
        }

        x /= n;
        y /= n;
        z /= n;
    }

    /**
     * @brief Computes the dot product (scalar product) with another vector.
     *
     * \f[
     * \mathbf{a} \cdot \mathbf{b} = a_x b_x + a_y b_y + a_z b_z
     * \f]
     *
     * The dot product represents the projection of one vector onto another.
     *
     * @param other Second vector
     * @return Scalar result
     */
    double dot(const Vector3& other) const
    {
        return x * other.x + y * other.y + z * other.z;
    }

    /**
     * @brief Computes the cross product with another vector.
     *
     * \f[
     * \mathbf{a} \times \mathbf{b} =
     * \begin{bmatrix}
     * a_y b_z - a_z b_y \\
     * a_z b_x - a_x b_z \\
     * a_x b_y - a_y b_x
     * \end{bmatrix}
     * \f]
     *
     * The cross product results in a vector perpendicular to both input vectors.
     *
     * @param other Second vector
     * @return Resulting vector
     */
    Vector3 cross(const Vector3 &other) const {
        double x = this->y * other.z - this->z * other.y;
        double y = this->z * other.x - this->x * other.z;
        double z = this->x * other.y - this->y * other.x;

        return {x, y, z};
    }
};

#endif // VECTOR3
