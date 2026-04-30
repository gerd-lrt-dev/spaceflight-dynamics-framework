#ifndef FUELSTATESTRUCT_H
#define FUELSTATESTRUCT_H

/**
 * @brief Fuel-related state variables for the engine.
 *
 * Tracks the mass of the remaining fuel and instantaneous consumption rate.
 * All values evolve over simulation time.
 */
struct FuelState
{
    double massStart;
    /**<
     * @brief Fuel mass at the previous simulation timestep.
     * @unit kilograms [kg]
     * Used to compute per-step consumption rates.
     */

    double massCurrent;
    /**<
     * @brief Current remaining fuel mass.
     * @unit kilograms [kg]
     * Updated each timestep based on thrust and Isp.
     */

    double consumptionRate;
    /**<
     * @brief Instantaneous propellant mass flow rate.
     * @unit kilograms per second [kg/s]
     * Calculated from thrust and specific impulse:
     *  \f$ \dot{m} = \frac{T}{I_{sp} g_0} \f$
     */
};

#endif // FUELSTATESTRUCT_H
