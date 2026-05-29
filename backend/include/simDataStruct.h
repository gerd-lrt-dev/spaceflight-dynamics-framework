#ifndef SIMDATASTRUCT_H
#define SIMDATASTRUCT_H

#include "stateVectorStruct.h"
#include "spacecraftStateStruct.h"
#include "Thrust/FueltankStruct.h"
#include "Thrust/ME_thrustState.h"
#include "Thrust/RCS_ThrustState.h"

#include <string>
#include <vector>

/**
 * @brief Aggregated simulation telemetry and runtime state data exchanged
 * between backend and frontend.
 *
 * This structure represents the primary simulation data container returned by
 * the simulation backend during runtime execution.
 *
 * It contains:
 * - spacecraft navigation and kinematic data
 * - spacecraft integrity and operational state
 * - propulsion telemetry
 * - fuel system telemetry
 * - force and acceleration data
 * - backend debug output
 *
 * The structure is used for:
 * - real-time cockpit visualization
 * - backend/frontend communication
 * - telemetry export (e.g. XML/CSV)
 * - debugging and validation
 * - future research-oriented data analysis workflows
 */
struct simData
{
    // Navigation data
    StateVector statevector_;

    // Spacecraft Integrity
    bool spacecraftIntegrity;
    SpacecraftState spacecraftState_;

    // Console output
    std::string output;

    // MainEngine data
    ME_ThrustState ME_ThrustState_;

    // RCSEngine data
    std::vector<RCS_ThrustState> RCS_ThrustState_;

    // Fuel data
    std::vector<FuelTank> tanks;

    /**
     * @brief Total remaining fuel mass of all installed tanks.
     *
     * Represents the summed fuel mass across the complete propulsion system.
     *
     * Unit: kg
     */
    double fuelMass;

    /**
     * @brief Total fuel mass flow rate.
     *
     * Represents the combined instantaneous fuel consumption of all active
     * propulsion elements.
     *
     * Unit: kg/s
     */
    double fuelFlow;

    // Forces
    /**
     * @brief Experienced spacecraft g-load.
     *
     * Represents the proper acceleration acting on the spacecraft structure
     * and crew.
     *
     * Unit: g
     */
    double GLoad;
};

#endif // SIMDATASTRUCT_H
