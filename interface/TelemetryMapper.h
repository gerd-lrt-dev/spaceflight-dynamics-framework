#ifndef TELEMETRYMAPPER_H
#define TELEMETRYMAPPER_H

#include "TelemetryDTO.h"

#include <Eigen/Dense>

/**
 * @file TelemetryMapper.h
 * @brief Translation layer between backend simulation data and frontend telemetry DTOs.
 */

class simcontrol;

/**
 * @class telemetryMapper
 * @brief Converts backend simulation data into frontend telemetry objects.
 *
 * This class forms the architectural boundary between the simulation backend
 * and the Qt frontend.
 *
 * It translates the backend-specific simulation data structures (`simData`)
 * into the frontend-oriented `Telemetry` data transfer object (DTO).
 *
 * The mapper intentionally performs only data transformation and type
 * conversion. It contains no simulation logic or presentation logic.
 *
 * Responsibilities:
 * - Decouple frontend from backend implementation details
 * - Translate backend types into frontend DTOs
 * - Convert backend string types to Qt types where required
 * - Provide a stable interface for UI telemetry updates
 */
class TelemetryMapper
{
public:
    /**
     * @brief Constructs the telemetry mapper.
     *
     * Creates the backend simulation controller and prepares the
     * telemetry translation layer.
     */
    TelemetryMapper();

    /**
     * @brief Destroys the telemetry mapper.
     */
    ~TelemetryMapper();

    /**
     * @brief Initializes the backend simulation.
     *
     * Parses the supplied JSON configuration and initializes the
     * simulation backend before the first simulation step.
     *
     * @param jsonConfig JSON spacecraft and mission configuration.
     */
    void initialize(const std::string& jsonConfig);

    /**
     * @brief Generates a frontend telemetry snapshot.
     *
     * Converts the previously supplied backend simulation state into a
     * frontend `Telemetry` DTO suitable for Qt signal/slot communication.
     *
     * @return Frontend telemetry snapshot.
     */
    Telemetry getQTTelemetryData() const;

    /**
     * @brief Executes one complete simulation step.
     *
     * Advances the simulation by one discrete timestep and updates the internal
     * simulation state. The resulting simulation snapshot can subsequently be
     * retrieved via getSimulationData().
     *
     * @param dt Discrete simulation timestep [s].
     */
    void runStepSimulation(const double dt) const;

    /**
     * @brief Sets reset Boolean in backend to true
    */
    void setReset();

    /**
     * @brief Forwards a user control command to the simulation backend.
     *
     * Passes a frontend-generated control command through the telemetry interface
     * to the backend simulation. The backend remains responsible for command
     * arbitration and for separating user input from autopilot commands.
     *
     * @param userCmd Control command provided by the frontend.
     */
    void transferUserCommandtoBackend(const Telemetry::ControlCommand& userCmd);

private:
    std::unique_ptr<simcontrol> backend_;
};

#endif // TELEMETRYMAPPER_H
