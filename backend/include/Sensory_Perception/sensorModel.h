#pragma once

#include "Sensory_Perception/iSensor.h"
#include "environmentConfig.h"

#include <eigen3/Eigen/Dense>

/**
 * @class SensorModel
 * @brief Abstract base class for spacecraft sensor model implementations.
 *
 * SensorModel extends the generic ISensor interface and represents a
 * configurable collection of onboard measurement algorithms. Implementations
 * derive observable quantities from the physical simulation state without
 * modifying it.
 *
 * A SensorModel may aggregate multiple virtual instruments such as
 * accelerometers, altimeters, gyroscopes or fuel gauges. Concrete subclasses
 * can introduce effects like noise, bias, filtering or delay to emulate
 * realistic hardware behavior.
 *
 * This layer forms the perception boundary between the physical simulation
 * (ground truth) and the spacecraft’s instrumentation or telemetry output.
 */
class SensorModel : public ISensor {
public:

    /**
     * @brief Constructs the model with environment configuration data.
     *
     * @param cfg Reference to global environment configuration containing
     *            physical constants such as gravitational parameters.
     */
    SensorModel(const EnvironmentConfig& cfg) : configData(cfg) {}

    /**
     * @brief Computes the experienced g-load from acceleration vectors.
     *
     * The g-load corresponds to the proper acceleration measured by an
     * onboard accelerometer and excludes gravitational acceleration.
     * The value is typically normalized to Earth gravity (g₀).
     *
     * @param totalAcceleration   Total acceleration acting on the spacecraft.
     * @param gravityAcceleration Gravitational acceleration vector.
     * @return Scalar g-load value in multiples of Earth gravity.
     */
    double computeGLoad(const Eigen::Vector3d& totalAcceleration, const Eigen::Vector3d& gravityAcceleration, bool isLanded) const override;

private:
    /**
     * @brief Reference to environment configuration containing constants.
     */
    const EnvironmentConfig& configData;
};
