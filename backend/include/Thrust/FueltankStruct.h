#ifndef FUELTANKSTRUCT_H
#define FUELTANKSTRUCT_H

#include <string>

/**
 * @brief Represents a propellant storage tank in the spacecraft.
 *
 * A FuelTank stores propellant mass that can be consumed by one or
 * multiple engines during the simulation.
 *
 * Tanks are managed by the propulsion orchestrator (e.g. Thrust system)
 * and referenced by engines via a tank identifier.
 *
 * Responsibilities:
 * - Store propellant mass
 * - Provide propellant to engines
 * - Track remaining propellant
 *
 * The tank itself does not compute thrust or engine behaviour.
 * It only acts as a mass reservoir.
 *
 * Example usage:
 * @code
 * FuelTank mainTank;
 * mainTank.id = 0;
 * mainTank.name = "MainTank";
 * mainTank.capacity = 500.0;
 * mainTank.mass = 500.0;
 * @endcode
 */
struct FuelTank
{
    int id;
    /**<
     * @brief Unique identifier of the fuel tank.
     * @unit none
     *
     * Used by engines to reference which tank supplies propellant.
     */

    std::string name;
    /**<
     * @brief Human-readable tank name.
     *
     * Example:
     * - "Main Tank"
     * - "RCS Tank"
     */

    std::string role;
    /**<
     * @brief role
     * Which engine is mainly connected to the tank
     */

    double capacity;
    /**<
     * @brief Maximum propellant mass the tank can hold.
     * @unit kilograms [kg]
     */

    double mass;
    /**<
     * @brief Current propellant mass stored in the tank.
     * @unit kilograms [kg]
     *
     * This value decreases during simulation when engines consume fuel.
     */

    /**
     * @brief Removes propellant from the tank.
     *
     * This function decreases the current fuel mass based on engine
     * consumption. The tank mass will never drop below zero.
     *
     * @param amount Propellant mass to remove [kg]
     */
    void consume(double amount)
    {
        if(amount <= 0.0)
            return;

        mass -= amount;

        if(mass < 0.0)
            mass = 0.0;
    }

    /**
     * @brief Returns remaining propellant fraction.
     *
     * Useful for UI displays or telemetry.
     *
     * @return Remaining fuel ratio [0..1]
     */
    double fillRatio() const
    {
        if(capacity <= 0.0)
            return 0.0;

        return mass / capacity;
    }

    /**
     * @brief Checks if the tank is empty.
     *
     * @return true if no propellant remains.
     */
    bool empty() const
    {
        return mass <= 0.0;
    }
};

#endif // FUELTANKSTRUCT_H
