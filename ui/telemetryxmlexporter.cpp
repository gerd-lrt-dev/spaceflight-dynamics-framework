#include "telemetryxmlexporter.h"

#include <QFile>
#include <QIODevice>
#include <QXmlStreamWriter>

bool TelemetryXmlExporter::exportTelemetry(const QVector<Telemetry>& telemetryHistory,
                                           const QString& filePath,
                                           QString* errorMessage) const
{
    if (telemetryHistory.isEmpty())
    {
        if (errorMessage)
            *errorMessage = "Telemetry history is empty.";

        return false;
    }

    if (filePath.trimmed().isEmpty())
    {
        if (errorMessage)
            *errorMessage = "No destination file path was provided.";

        return false;
    }

    QFile file(filePath);

    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
    {
        if (errorMessage)
            *errorMessage = QString("Unable to open XML export file: %1").arg(file.errorString());

        return false;
    }

    QXmlStreamWriter writer(&file);
    writer.setAutoFormatting(true);
    writer.setAutoFormattingIndent(4);

    writer.writeStartDocument();
    writer.writeStartElement("TelemetryExport");
    writer.writeAttribute("schemaVersion", "1.0");
    writer.writeAttribute("sampleCount", QString::number(telemetryHistory.size()));

    for (qsizetype index = 0; index < telemetryHistory.size(); ++index)
    {
        writeTelemetrySnapshot(writer, telemetryHistory.at(index), index);
    }

    writer.writeEndElement();
    writer.writeEndDocument();

    if (writer.hasError())
    {
        if (errorMessage)
            *errorMessage = "An XML serialization error occurred while writing telemetry data.";

        file.close();
        return false;
    }

    file.close();

    if (file.error() != QFileDevice::NoError)
    {
        if (errorMessage)
            *errorMessage = QString("Unable to finalize XML export file: %1").arg(file.errorString());

        return false;
    }

    if (errorMessage)
        errorMessage->clear();

    return true;
}

void TelemetryXmlExporter::writeTelemetrySnapshot(QXmlStreamWriter& writer,
                                                  const Telemetry& telemetry,
                                                  qsizetype index) const
{
    writer.writeStartElement("TelemetrySnapshot");
    writer.writeAttribute("index", QString::number(index));

    writeDouble(writer, "time", telemetry.time);
    writeNavigation(writer, telemetry.navigation);
    writeMissionContext(writer, telemetry.missionContext);
    writeFrameContext(writer, telemetry.frameContext);
    writeHullIntegrity(writer, telemetry.hullIntegrity);
    writeAutomation(writer, telemetry.automation);
    writeConsole(writer, telemetry.console);
    writePropulsionSystems(writer, telemetry.propulsionSystems);
    writeSensor(writer, telemetry.sensor);

    writer.writeEndElement();
}

void TelemetryXmlExporter::writeNavigation(QXmlStreamWriter& writer,
                                           const Telemetry::Navigation& navigation) const
{
    writer.writeStartElement("navigation");

    writeVector3d(writer, "MCI_position", navigation.MCI_position);
    writeVector3d(writer, "MCI_velocity", navigation.MCI_velocity);
    writeQuaternion(writer, "IB_Orientation", navigation.IB_Orientation);
    writeVector3d(writer, "SBF_AngularVelocity", navigation.SBF_AngularVelocity);

    writer.writeEndElement();
}

void TelemetryXmlExporter::writeMissionContext(
    QXmlStreamWriter& writer,
    const Telemetry::MissionContext& missionContext) const
{
    writer.writeStartElement("missionContext");

    writeSurfaceCoordinates(writer,
                            "MSC_LandingSite",
                            missionContext.MSC_LandingSite);
    writePositionState(writer,
                       "MCMF_landingSite",
                       missionContext.MCMF_landingSite);
    writePositionState(writer,
                       "MCI_landingSite",
                       missionContext.MCI_landingSite);

    writer.writeEndElement();
}

void TelemetryXmlExporter::writeFrameContext(QXmlStreamWriter& writer,
                                             const Telemetry::FrameContext& frameContext) const
{
    writer.writeStartElement("frameContext");

    writeFrameState(writer, "MCI_State", frameContext.MCI_State);
    writeFrameState(writer, "MCMF_State", frameContext.MCMF_State);
    writeFrameState(writer, "ENU_State", frameContext.ENU_State);
    writeFrameState(writer, "LVLH_State", frameContext.LVLH_State);
    writeSurfaceCoordinates(writer, "MSC_State", frameContext.MSC_State);

    writer.writeEndElement();
}

void TelemetryXmlExporter::writeHullIntegrity(
    QXmlStreamWriter& writer,
    const Telemetry::HullIntegrity& hullIntegrity) const
{
    writer.writeStartElement("hullIntegrity");

    writeDouble(writer, "spacecraftIntegrity", hullIntegrity.spacecraftIntegrity);
    writer.writeTextElement("spacecraftState", hullIntegrity.spacecraftState);

    writer.writeEndElement();
}

void TelemetryXmlExporter::writeAutomation(
    QXmlStreamWriter& writer,
    const Telemetry::Automation& automation) const
{
    writer.writeStartElement("automation");
    writer.writeTextElement("killRotationActive",
                            automation.killRotationActive ? "true" : "false");
    writer.writeTextElement("stabilizeActive",
                            automation.stabilizeActive ? "true" : "false");
    writer.writeEndElement();
}

void TelemetryXmlExporter::writeConsole(QXmlStreamWriter& writer,
                                        const Telemetry::Console& console) const
{
    writer.writeStartElement("console");
    writer.writeTextElement("output", console.output);
    writer.writeEndElement();
}

void TelemetryXmlExporter::writePropulsionSystems(
    QXmlStreamWriter& writer,
    const Telemetry::PropulsionSystems& propulsionSystems) const
{
    writer.writeStartElement("propulsionSystems");

    writer.writeStartElement("mainEngine");
    writeDouble(writer, "T_target", propulsionSystems.mainEngine.T_target);
    writeDouble(writer, "T_current", propulsionSystems.mainEngine.T_current);
    writeDouble(writer,
                "T_targetPercentage",
                propulsionSystems.mainEngine.T_targetPercentage);
    writeDouble(writer, "massflow", propulsionSystems.mainEngine.massflow);
    writeVector3d(writer,
                  "SBF_direction",
                  propulsionSystems.mainEngine.SBF_direction);
    writer.writeEndElement();

    writer.writeStartElement("RCSEngines");
    writer.writeAttribute("count", QString::number(propulsionSystems.RCSEngines.size()));

    for (const Telemetry::PropulsionSystems::RCSThrust& engine : propulsionSystems.RCSEngines)
    {
        writer.writeStartElement("RCSEngine");

        writeInt(writer, "engineID", engine.engineID);
        writer.writeTextElement("engineName", engine.engineName);
        writeVector3d(writer, "SBF_direction", engine.SBF_direction);
        writeDouble(writer, "T_current", engine.T_current);
        writeDouble(writer, "T_target", engine.T_target);
        writeDouble(writer, "maxThrust", engine.maxThrust);
        writeDouble(writer, "massflow", engine.massflow);

        writer.writeEndElement();
    }

    writer.writeEndElement();

    writer.writeStartElement("fuelTanks");
    writer.writeAttribute("count", QString::number(propulsionSystems.fuelTanks.size()));

    for (const Telemetry::PropulsionSystems::Tank& tank : propulsionSystems.fuelTanks)
    {
        writer.writeStartElement("Tank");

        writeInt(writer, "ID", tank.ID);
        writer.writeTextElement("name", tank.name);
        writer.writeTextElement("role", tank.role);
        writeDouble(writer, "capacity", tank.capacity);
        writeDouble(writer, "mass", tank.mass);
        writeDouble(writer, "fillLevel", tank.fillLevel);

        writer.writeEndElement();
    }

    writer.writeEndElement();
    writer.writeEndElement();
}

void TelemetryXmlExporter::writeSensor(QXmlStreamWriter& writer,
                                       const Telemetry::Sensor& sensor) const
{
    writer.writeStartElement("sensor");
    writeDouble(writer, "GLoad", sensor.GLoad);
    writer.writeEndElement();
}

void TelemetryXmlExporter::writeSurfaceCoordinates(
    QXmlStreamWriter& writer,
    const QString& elementName,
    const Telemetry::MissionContext::SurfaceCoordinates& coordinates) const
{
    writer.writeStartElement(elementName);

    writeDouble(writer, "latitude", coordinates.latitude);
    writeDouble(writer, "longitude", coordinates.longitude);
    writeDouble(writer, "altitude", coordinates.altitude);

    writer.writeEndElement();
}

void TelemetryXmlExporter::writePositionState(
    QXmlStreamWriter& writer,
    const QString& elementName,
    const Telemetry::MissionContext::PositionState& state) const
{
    writer.writeStartElement(elementName);
    writeVector3d(writer, "position", state.position);
    writer.writeEndElement();
}

void TelemetryXmlExporter::writeFrameState(
    QXmlStreamWriter& writer,
    const QString& elementName,
    const Telemetry::FrameContext::State& state) const
{
    writer.writeStartElement(elementName);

    writeVector3d(writer, "position", state.position);
    writeVector3d(writer, "velocity", state.velocity);

    writer.writeEndElement();
}

void TelemetryXmlExporter::writeVector3d(QXmlStreamWriter& writer,
                                         const QString& elementName,
                                         const Eigen::Vector3d& vector) const
{
    writer.writeStartElement(elementName);

    writeDouble(writer, "x", vector.x());
    writeDouble(writer, "y", vector.y());
    writeDouble(writer, "z", vector.z());

    writer.writeEndElement();
}

void TelemetryXmlExporter::writeQuaternion(QXmlStreamWriter& writer,
                                           const QString& elementName,
                                           const Eigen::Quaterniond& quaternion) const
{
    writer.writeStartElement(elementName);

    writeDouble(writer, "w", quaternion.w());
    writeDouble(writer, "x", quaternion.x());
    writeDouble(writer, "y", quaternion.y());
    writeDouble(writer, "z", quaternion.z());

    writer.writeEndElement();
}

void TelemetryXmlExporter::writeDouble(QXmlStreamWriter& writer,
                                       const QString& elementName,
                                       double value) const
{
    writer.writeTextElement(elementName, QString::number(value, 'g', 17));
}

void TelemetryXmlExporter::writeInt(QXmlStreamWriter& writer,
                                    const QString& elementName,
                                    int value) const
{
    writer.writeTextElement(elementName, QString::number(value));
}
