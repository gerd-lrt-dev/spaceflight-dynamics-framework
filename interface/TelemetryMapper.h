#ifndef TELEMETRYMAPPER_H
#define TELEMETRYMAPPER_H

#include "TelemetryDTO.h"
#include "include/simDataStruct.h"

#include <Eigen/Dense>

/**
 * @file TelemetryMapper.h
 * @brief Translation layer between backend simulation data and frontend telemetry DTOs.
 */

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
     * @brief Stores the backend simulation snapshot to be translated.
     *
     * @param simdata_ Current backend simulation state.
     */
    void setBackendData(simData simdata_);

    /**
     * @brief Generates a frontend telemetry snapshot.
     *
     * Converts the previously supplied backend simulation state into a
     * frontend `Telemetry` DTO suitable for Qt signal/slot communication.
     *
     * @return Frontend telemetry snapshot.
     */
    Telemetry getQTTelemetryData();

private:

    /// Latest backend simulation snapshot.
    simData backendData;
};

#endif // TELEMETRYMAPPER_H
