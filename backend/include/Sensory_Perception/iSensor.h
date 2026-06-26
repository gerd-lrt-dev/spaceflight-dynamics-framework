#pragma once

#include <eigen3/Eigen/Dense>

/**
 * @class ISensor
 * @brief Interface for spacecraft sensor and telemetry models.
 *
 * ISensor defines the contract for all sensor implementations that derive
 * measurable values from the current physical state of the spacecraft.
 * Sensors do not modify the simulation state; they only observe and
 * transform physical quantities into instrument or telemetry outputs.
 *
 * Typical implementations may include accelerometers, altimeters,
 * gyroscopes, fuel gauges, or other cockpit instruments. Sensor models
 * may optionally introduce noise, bias, filtering or delay to simulate
 * real-world measurement behavior.
 */
class ISensor {
public:

    /**
     * @brief Virtual destructor to ensure proper cleanup of derived sensors.
     */
    virtual ~ISensor() = default;

    /**
     * @brief Computes the experienced g-load from acceleration vectors.
     *
     * The g-load represents the proper acceleration measured by an onboard
     * accelerometer and excludes gravitational acceleration. The result is
     * normalized to Earth gravity (g₀ ≈ 9.80665 m/s²).
     *
     * @param totalAcceleration   Total acceleration acting on the spacecraft.
     * @param gravityAcceleration Gravitational acceleration vector.
     * @return Scalar g-load value in multiples of Earth gravity.
     */
    virtual double computeGLoad(const Eigen::Vector3d& totalAcceleration, const Eigen::Vector3d& gravityAcceleration, bool isLanded) const = 0;
};
