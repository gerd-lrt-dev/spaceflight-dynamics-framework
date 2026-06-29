#include "simulationworker.h"

#include <QString>
#include <eigen3/Eigen/Dense>

SimulationWorker::SimulationWorker(QObject *parent)
    : QObject(parent)
{
    // Build timer event
    simulationTimer = new QTimer(this);
    simulationTimer->setInterval(50); // 20 Hz

    // connect timer with worker function
    connect(simulationTimer, &QTimer::timeout, this, &SimulationWorker::stepSimulation);
}

void SimulationWorker::start()
{
    try {
        telemetryMapper_.initialize(jsonConfig);
    }
    catch (const std::exception& e)
    {
        qCritical() << "Simulation start failed: " << e.what();
        emit simulationError(QString::fromStdString(e.what()));
    }

    running = true;
    simulationTimer->start();

    qDebug("[simulationworker]-start-: Simulation started successfully");
}

void SimulationWorker::pause()
{
    running = false;
    simulationTimer->stop();
}

void SimulationWorker::stop()
{
    running = false;
    simulationTimer->stop();
    currentTime = 0.0;

    emit stateUpdated(currentTime,
                      {0.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0},
                      0.0,
                      "Operational",
                      {0.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0},
                      QVector<Telemetry::PropulsionSystems::RCSThrust>{},
                      QVector<Telemetry::PropulsionSystems::Tank>{},
                      0.0,
                      0.0,
                      "");

    telemetryMapper_.setReset();
}

void SimulationWorker::receiveJsonConfig(const QString &json)
{
    this->jsonConfig = json.toStdString();

    qDebug() << "SimulationWorker received JSON config, size:"
             << jsonConfig.size();
}

void SimulationWorker::setFlightCommand(FlightCommand cmd)
{
    QMutexLocker locker(&mutex);

    collectControlCommands(cmd);
}

void SimulationWorker::setAutopilotFlag(bool active)
{
    QMutexLocker locker(&mutex); // protect access
    collectAutopilotCommand(active);
}

void SimulationWorker::stepSimulation()
{
    double dt = 0.00;   ///< Initialized discrete timestep

    // Return if not running
    if(!running)
        return;

    // times
    // TODO: Should be change by timer event
    dt = 0.05; // TODO: should specified in json as well
    currentTime += dt;

    // Update backend via interface
    telemetryMapper_.runStepSimulation(dt);

    // Get backend data via interface
    telemetry_ = telemetryMapper_.getQTTelemetryData();

    // Withdraw user input due to thrust
    sendControlCommands();

    // signals
    emit stateUpdated(currentTime,
                      telemetry_.navigation.MCI_position, //spacecraftData.statevector_.MCI_Position,
                      telemetry_.navigation.MCI_velocity, //spacecraftData.statevector_.MCI_Velocity,
                      telemetry_.sensor.GLoad, //spacecraftData.GLoad,
                      telemetry_.hullIntegrity.spacecraftState, //spacecraftData.spacecraftState_,
                      telemetry_.propulsionSystems.mainEngine.SBF_direction * telemetry_.propulsionSystems.mainEngine.T_current, //spacecraftData.ME_ThrustState_.SBF_direction * spacecraftData.ME_ThrustState_.current,
                      telemetry_.propulsionSystems.mainEngine.SBF_direction * telemetry_.propulsionSystems.mainEngine.T_target, //spacecraftData.ME_ThrustState_.SBF_direction * spacecraftData.ME_ThrustState_.target,
                      telemetry_.propulsionSystems.mainEngine.SBF_direction * telemetry_.propulsionSystems.mainEngine.T_targetPercentage, //spacecraftData.ME_ThrustState_.SBF_direction * spacecraftData.ME_ThrustState_.targetPercentage,
                      telemetry_.propulsionSystems.RCSEngines, //RCSTelemetryVec,
                      telemetry_.propulsionSystems.fuelTanks, //fuelTanksQVec,
                      0.0, //spacecraftData.fuelMass,
                      0.0, //spacecraftData.fuelFlow,
                      telemetry_.console.output //consoleOutput
                      );
}

void SimulationWorker::collectControlCommands(const FlightCommand &cmd, const double &thrustInPercentage, const double &thrustInNewton)
{
    collectedCmdData = cmd;
}

void SimulationWorker::collectAutopilotCommand(bool autopilotActive)
{
    collectedCmdData.autopilotActive      = autopilotActive;
}


void SimulationWorker::sendControlCommands()
{
    telemetryMapper_.transferUserCommandtoBackend(collectedCmdData); //->receiveCommandFromFrontEnd(FEControlCommands_);
}


