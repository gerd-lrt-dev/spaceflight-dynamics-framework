#ifndef TELEMETRYXMLEXPORTER_H
#define TELEMETRYXMLEXPORTER_H

#include <QString>
#include <QVector>

#include "TelemetryDTO.h"

class QXmlStreamWriter;

/**
 * @class TelemetryXmlExporter
 * @brief Serializes recorded frontend telemetry snapshots to an XML file.
 *
 * The exporter operates exclusively on the frontend-facing Telemetry DTO and
 * therefore remains independent from backend simulation models. A complete
 * telemetry history is written as a sequence of snapshots while preserving
 * the functional grouping defined by TelemetryDTO.
 *
 * The class performs serialization only. Collection of telemetry data,
 * lifecycle handling and user interaction remain responsibilities of the
 * calling frontend components.
 */
class TelemetryXmlExporter
{
public:
    /**
     * @brief Exports a complete telemetry history to an XML file.
     *
     * Every Telemetry snapshot and all nested DTO fields are serialized.
     * Existing files at @p filePath are replaced.
     *
     * @param telemetryHistory Recorded telemetry snapshots in chronological order.
     * @param filePath Destination path of the XML file.
     * @param errorMessage Optional output parameter receiving a human-readable
     *        error description when the export fails.
     * @return True if the XML file was written successfully, otherwise false.
     */
    bool exportTelemetry(const QVector<Telemetry>& telemetryHistory,
                         const QString& filePath,
                         QString* errorMessage = nullptr) const;

private:
    void writeTelemetrySnapshot(QXmlStreamWriter& writer,
                                const Telemetry& telemetry,
                                qsizetype index) const;

    void writeNavigation(QXmlStreamWriter& writer,
                         const Telemetry::Navigation& navigation) const;

    void writeMissionContext(QXmlStreamWriter& writer,
                             const Telemetry::MissionContext& missionContext) const;

    void writeFrameContext(QXmlStreamWriter& writer,
                           const Telemetry::FrameContext& frameContext) const;

    void writeHullIntegrity(QXmlStreamWriter& writer,
                            const Telemetry::HullIntegrity& hullIntegrity) const;

    void writeAutomation(QXmlStreamWriter& writer,
                         const Telemetry::Automation& automation) const;

    void writeConsole(QXmlStreamWriter& writer,
                      const Telemetry::Console& console) const;

    void writePropulsionSystems(QXmlStreamWriter& writer,
                                const Telemetry::PropulsionSystems& propulsionSystems) const;

    void writeSensor(QXmlStreamWriter& writer,
                     const Telemetry::Sensor& sensor) const;

    void writeSurfaceCoordinates(
        QXmlStreamWriter& writer,
        const QString& elementName,
        const Telemetry::MissionContext::SurfaceCoordinates& coordinates) const;

    void writePositionState(
        QXmlStreamWriter& writer,
        const QString& elementName,
        const Telemetry::MissionContext::PositionState& state) const;

    void writeFrameState(QXmlStreamWriter& writer,
                         const QString& elementName,
                         const Telemetry::FrameContext::State& state) const;

    void writeVector3d(QXmlStreamWriter& writer,
                       const QString& elementName,
                       const Eigen::Vector3d& vector) const;

    void writeQuaternion(QXmlStreamWriter& writer,
                         const QString& elementName,
                         const Eigen::Quaterniond& quaternion) const;

    void writeDouble(QXmlStreamWriter& writer,
                     const QString& elementName,
                     double value) const;

    void writeInt(QXmlStreamWriter& writer,
                  const QString& elementName,
                  int value) const;
};

#endif // TELEMETRYXMLEXPORTER_H
