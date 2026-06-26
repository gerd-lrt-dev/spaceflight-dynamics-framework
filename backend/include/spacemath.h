#ifndef SPACEMATH_H
#define SPACEMATH_H

#include <eigen3/Eigen/Dense>


/**
 * @class spacemath
 * @brief Utility class for space-related physics calculations.
 *
 * This class provides static methods to compute common physical quantities
 * used in spacecraft simulations, such as kinetic energy, potential energy,
 * or other physics-based calculations.
 *
 * All functions are static and can be called without instantiating the class.
 */
class spacemath
{
public:
    /**
     * @brief Constructor (trivial, no data members)
     */
    spacemath() = default;

    /**
     * @brief Destructor (trivial)
     */
    ~spacemath() = default;

    /**
     * @brief Computes kinetic energy.
     * 
     * Calculates the kinetic energy of a body given its mass and velocity:
     * \f$ KE = \frac{1}{2} m v^2 \f$
     *
     * @param mass      ///< [kg] Mass of the object 
     * @param velocity  ///< [m/s] Velocity of the object 
     * @return          ///< [J] Kinetic energy 
     */
    static double kineticEnergy(double mass, double velocity);

    /**
     * @brief Calculates acceleration based on thrust
     * @param thrust    ///< [N] Engine thrust spacecraft
     * @param mass      ///< [kg] Total mass of spacecraft
     * 
     * a = F{thrust} / m{total}
     */
    static double accelerationBasedOnThrust(double thrust, double mass);

    /**
     * @brief Calculates acceleration based on thrust, mass and specific impulse
     * @param currentThrust     ///< [N] Current thrust of Spacecraft
     * @param totalMass         ///< [kg] Total mass of Spacecraft
     * @param directionOfThrust ///< [-] Vektor, in which direction the thrust is running
     * @param gravityConstant   ///< [m/s²] Constant of Gravity
     * @return                  ///< [m/s²] Acceleration of Spacecraft due to thrust, mass and thrust 
     * 
     */
    static Eigen::Vector3d accelerationComplex(double currentThrust, double totalMass, Eigen::Vector3d directionOfThrust, Eigen::Vector3d gravityConstant);

    /**
     * @brief Calcualtes acceleration in relation with thrust direction
     * @param currentThrust     ///< [N] Engine thrust spacecraft
     * @param thrustDirection   ///< [-] Vector with direction of thrust. Static coordinate system is spacecraft.
     * @return                  ///< [m/s²] Return acceleration vector
     */
    static Eigen::Vector3d calcAccelerationVector(double currentThrust, Eigen::Vector3d thrustDirection);

    /**
     * @brief Calculates Mass flow based on thrust
     * @param currentThrust ///< [N] Thrust from the last time step
     * @param Isp           ///< [s] Specific impulse
     * 
     * g0 is always the acceleration due to gravity, regardless of whether you are 
     * on the moon or Mars. This is defined historically and physically because 
     * specific impulse is measured according to this standard.
     * 
     * \dot(m) = F / (I_{Sp} \cdot g_0)
     */
    static double calcMassFlowBasedOnThrust(double currenThrust, double Isp, double earthGravity);

    
};

#endif
