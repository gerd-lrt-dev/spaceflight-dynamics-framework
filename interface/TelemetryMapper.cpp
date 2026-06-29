#include "TelemetryMapper.h"

namespace
{
QString toQString(const std::string& value)
{
    return QString::fromStdString(value);
}

QString spacecraftStateToQString(SpacecraftState state)
{
    switch (state)
    {
    case SpacecraftState::Operational:
        return "Operational";
    case SpacecraftState::Landed:
        return "Landed";
    case SpacecraftState::Crashed:
        return "Crashed";
    case SpacecraftState::Destroyed:
        return "Destroyed";
    default:
        return "Unknown";
    }
}
}

void TelemetryMapper::setBackendData(simData simdata_)
{
    backendData = simdata_;
}

Telemetry TelemetryMapper::getQTTelemetryData()
{
    Telemetry FE;

    // -------------------------------------------------------------------------
    // Navigation
    // -------------------------------------------------------------------------

    FE.navigation.MCI_position =
        backendData.statevector_.MCI_Position;

    FE.navigation.MCI_velocity =
        backendData.statevector_.MCI_Velocity;

    FE.navigation.IB_Orientation =
        backendData.statevector_.IB_Orientation;

    FE.navigation.SBF_AngularVelocity =
        backendData.statevector_.SBF_AngularVelocity;

    // -------------------------------------------------------------------------
    // Hull integrity
    // -------------------------------------------------------------------------

    FE.hullIntegrity.spacecraftIntegrity =
        backendData.spacecraftIntegrity;

    FE.hullIntegrity.spacecraftState =
        spacecraftStateToQString(backendData.spacecraftState_);

    // -------------------------------------------------------------------------
    // Console
    // -------------------------------------------------------------------------

    FE.console.output =
        toQString(backendData.output);

    // -------------------------------------------------------------------------
    // Propulsion: Main engine
    // -------------------------------------------------------------------------

    FE.propulsionSystems.mainEngine.T_target =
        backendData.ME_ThrustState_.target;

    FE.propulsionSystems.mainEngine.T_current =
        backendData.ME_ThrustState_.current;

    FE.propulsionSystems.mainEngine.T_targetPercentage =
        backendData.ME_ThrustState_.targetPercentage;

    FE.propulsionSystems.mainEngine.SBF_direction =
        backendData.ME_ThrustState_.SBF_direction;

    // -------------------------------------------------------------------------
    // Propulsion: RCS engines
    // -------------------------------------------------------------------------

    FE.propulsionSystems.RCSEngines.clear();
    FE.propulsionSystems.RCSEngines.reserve(
        static_cast<int>(backendData.RCS_ThrustState_.size())
        );

    for (const RCS_ThrustState& backendRCS : backendData.RCS_ThrustState_)
    {
        Telemetry::PropulsionSystems::RCSThrust frontendRCS;

        frontendRCS.engineID =
            backendRCS.engineID;

        frontendRCS.engineName =
            toQString(backendRCS.engineName);

        frontendRCS.axis =
            toQString(backendRCS.axis);

        frontendRCS.SBF_direction =
            backendRCS.SBF_direction;

        frontendRCS.T_current =
            backendRCS.currentThrust;

        frontendRCS.T_target =
            backendRCS.targetThrust;

        frontendRCS.thrustState =
            backendRCS.thrustState;

        FE.propulsionSystems.RCSEngines.append(frontendRCS);
    }

    // -------------------------------------------------------------------------
    // Propulsion: Fuel tanks
    // -------------------------------------------------------------------------

    FE.propulsionSystems.fuelTanks.clear();
    FE.propulsionSystems.fuelTanks.reserve(
        static_cast<int>(backendData.tanks.size())
        );

    for (const FuelTank& backendTank : backendData.tanks)
    {
        Telemetry::PropulsionSystems::Tank frontendTank;

        frontendTank.ID =
            backendTank.id;

        frontendTank.name =
            toQString(backendTank.name);

        frontendTank.role =
            toQString(backendTank.role);

        frontendTank.capacity =
            backendTank.capacity;

        frontendTank.mass =
            backendTank.mass;

        frontendTank.fillLevel =
            backendTank.fillRatio();

        FE.propulsionSystems.fuelTanks.append(frontendTank);
    }

    // -------------------------------------------------------------------------
    // Sensors
    // -------------------------------------------------------------------------

    FE.sensor.GLoad =
        backendData.GLoad;

    return FE;
}
