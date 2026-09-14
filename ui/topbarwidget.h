#pragma once

#include <QWidget>

class QPushButton;

class TopBarWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TopBarWidget(QWidget *parent = nullptr);

    /**
     * @brief Enables or disables telemetry export availability.
     * @param enabled True if collected simulation telemetry may be exported.
     */
    void setExportEnabled(bool enabled);

signals:
    void homeRequested();
    void selectSpacecraftRequested();
    void startSimulationRequested();
    void exportTelemetryRequested();
    void controlsHelpRequested();
    void coordinateHelpRequested();
    void settingsRequested();

private:
    QPushButton *homeButton = nullptr;
    QPushButton *selectSpacecraftButton = nullptr;
    QPushButton *startSimulationButton = nullptr;
    QPushButton *exportTelemetryButton = nullptr;
    QPushButton *controlsHelpButton = nullptr;
    QPushButton *coordinateHelpButton = nullptr;
    QPushButton *settingsButton = nullptr;
};
