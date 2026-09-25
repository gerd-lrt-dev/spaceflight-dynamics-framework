#include "TelemetryMapper.h"
#include "simcontrol.h"

TelemetryMapper::TelemetryMapper() : backend_(std::make_unique<simcontrol>(0.0))
{

};

TelemetryMapper::~TelemetryMapper() = default;

namespace
{
    QString toQString(const std::string& value)
    {
        return QString::fromStdString(value);
    }

    std::string toStdString(const QString& value)
    {
        return value.toStdString();
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

void TelemetryMapper::initialize(const std::string& jsonConfig)
{
    backend_->initialize(jsonConfig);
}

Telemetry TelemetryMapper::getQTTelemetryData() const
{
    // Backend data
    simData backendData = backend_->getSimulationData();

    // Frontend data
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
    // Mission Context
    // -------------------------------------------------------------------------

    FE.missionContext.MSC_LandingSite.latitude =
        backendData.missionContext_.MSC_LandingSite.latitude;

    FE.missionContext.MSC_LandingSite.longitude =
        backendData.missionContext_.MSC_LandingSite.longitude;

    FE.missionContext.MSC_LandingSite.altitude =
        backendData.missionContext_.MSC_LandingSite.altitude;

    FE.missionContext.MCMF_landingSite.position =
        backendData.missionContext_.MCMF_landingSite.position;

    FE.missionContext.MCI_landingSite.position =
        backendData.missionContext_.MCI_landingSite.position;

    FE.missionContext.ENU_frame.origin.position =
        backendData.missionContext_.ENU_landingSite.origin.position;

    FE.missionContext.ENU_frame.east =
        backendData.missionContext_.ENU_landingSite.east;

    FE.missionContext.ENU_frame.north =
        backendData.missionContext_.ENU_landingSite.north;

    FE.missionContext.ENU_frame.up =
        backendData.missionContext_.ENU_landingSite.up;

    // -------------------------------------------------------------------------
    // Simulation Frame Context
    // -------------------------------------------------------------------------

    FE.frameContext.MCI_State.position =
        backendData.simFrameContext_.MCI_State.position;

    FE.frameContext.MCI_State.velocity =
        backendData.simFrameContext_.MCI_State.velocity;

    FE.frameContext.MCMF_State.position =
        backendData.simFrameContext_.MCMF_State.position;

    FE.frameContext.MCMF_State.velocity =
        backendData.simFrameContext_.MCMF_State.velocity;

    FE.frameContext.MSC_State.latitude =
        backendData.simFrameContext_.MSC_State.latitude;

    FE.frameContext.MSC_State.longitude =
        backendData.simFrameContext_.MSC_State.longitude;

    FE.frameContext.MSC_State.altitude =
        backendData.simFrameContext_.MSC_State.altitude;

    FE.frameContext.ENU_State.position =
        backendData.simFrameContext_.ENU_State.position;

    FE.frameContext.ENU_State.velocity =
        backendData.simFrameContext_.ENU_State.velocity;

    FE.frameContext.LVLH_State.position =
        backendData.simFrameContext_.LVLH_State.position;

    FE.frameContext.LVLH_State.velocity =
        backendData.simFrameContext_.LVLH_State.velocity;

    FE.frameContext.LVLH_frame.origin.position =
        backendData.simFrameContext_.LVLH_Frame.origin.position;

    FE.frameContext.LVLH_frame.origin.velocity =
        backendData.simFrameContext_.LVLH_Frame.origin.velocity;

    FE.frameContext.LVLH_frame.forward =
        backendData.simFrameContext_.LVLH_Frame.forward;

    FE.frameContext.LVLH_frame.right =
        backendData.simFrameContext_.LVLH_Frame.right;

    FE.frameContext.LVLH_frame.down =
        backendData.simFrameContext_.LVLH_Frame.down;

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

void TelemetryMapper::runStepSimulation(const double dt) const
{
    backend_->runSimulation(dt);
}

void TelemetryMapper::setReset()
{
    backend_->setResetBoolean();
}

void TelemetryMapper::transferUserCommandtoBackend(const FlightCommandDTO& userCmd)
{
    ControlCommand backendCmd;

    backendCmd.thrustInPercentage =
        userCmd.mainEngine;

    backendCmd.autopilotActive =
        userCmd.autopilotActive;

    backendCmd.mainEngine =
        userCmd.mainEngine;

    backendCmd.rotation =
        userCmd.rotation;

    backendCmd.translation =
        userCmd.translation;

    backendCmd.stabilize =
        userCmd.stabilize;

    backendCmd.killRotation =
        userCmd.killRotation;

    backend_->receiveCommandFromFrontEnd(backendCmd);
}
