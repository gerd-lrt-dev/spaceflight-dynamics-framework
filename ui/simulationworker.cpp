#include "simulationworker.h"

#include <QString>
#include <Eigen/Dense>

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

    emit stateUpdated(Telemetry{});

    telemetryMapper_.setReset();
}

void SimulationWorker::receiveJsonConfig(const QString &json)
{
    this->jsonConfig = json.toStdString();

    qDebug() << "SimulationWorker received JSON config, size:"
             << jsonConfig.size();
}

void SimulationWorker::setFlightCommand(FlightCommandDTO cmd)
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
    // Return if not running
    if(!running)
        return;

    // TODO: Should be change by timer event
    double dt = 0.05;   ///< Fixed discrete timestep

    // times
    currentTime += dt;

    // Withdraw user input due to thrust
    sendControlCommands();

    // Update backend via interface
    telemetryMapper_.runStepSimulation(dt);

    // Get backend data via interface
    telemetry_ = telemetryMapper_.getQTTelemetryData();

    // signals
    emit stateUpdated(telemetry_);
}

void SimulationWorker::collectControlCommands(const FlightCommandDTO &cmd, const double &thrustInPercentage, const double &thrustInNewton)
{
    collectedCmdData = cmd;
}

void SimulationWorker::collectAutopilotCommand(bool autopilotActive)
{
    collectedCmdData.autopilotActive      = autopilotActive;
}


void SimulationWorker::sendControlCommands()
{
    telemetryMapper_.transferUserCommandtoBackend(collectedCmdData);
}


