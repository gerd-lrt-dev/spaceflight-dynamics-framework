#ifndef ENVIRONMENTCONFIG_H
#define ENVIRONMENTCONFIG_H

#include "vector3.h"

/**
 * @struct EnvironmentConfig
 * @brief Holds environmental and planetary constants for the lunar lander simulation.
 *
 * This structure provides all relevant physical constants and simulation parameters
 * required to model the lunar environment. All spatial parameters are defined
 * relative to a Moon-centered reference frame.
 *
 * Constants include:
 *   - moonGravity              : Surface gravity on the Moon [m/s²]
 *   - earthGravity             : Standard Earth gravity [m/s²] (for reference)
 *   - radiusMoon               : Average lunar radius [m]
 *   - gravitationalFactorMoon  : Standard gravitational parameter μ = g_surface * r_moon² [m³/s²]
 *
 * Vector constants like moonGravityVec are defined in the Moon-centered
 * inertial frame and point in the positive Z direction away from the Moon's center.
 *
 * Simulation parameters:
 *   - maxTimeStep           : Maximum simulation timestep in milliseconds
 *
 * @note All positions and distances in the simulation are measured from the
 *       Moon's center, ensuring consistency with physics calculations in the
 *       physics class.
 */


struct EnvironmentConfig {
    // Constants
    const double moonGravity      = 1.635;     ///< [m/s²], Moon gravity
    const double earthGravity     = 9.81;      ///< [m/s^2], earth gravity

    // Planetary informations
    const double radiusMoon       = 1.7374e6;   ///< [m] Middle moon radius 

    // Multiplied factors
    const double muMoon = moonGravity * radiusMoon * radiusMoon; ///< [m³/s²], Moon gravitational factor

    // Vector constants
    const Vector3 moonGravityVec  = {0, 0, -1.635};  ///< [m/s²], Moon gravity

    // Simulation parameters
    int maxTimeStep         = 16;      ///< [ms], maximum simulation step (~60 Hz)
};



#endif
