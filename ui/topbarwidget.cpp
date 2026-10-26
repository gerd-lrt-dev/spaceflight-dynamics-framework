#include "topbarwidget.h"
#include "uibuilder.h"

#include <QHBoxLayout>
#include <QPushButton>
#include <QSizePolicy>

TopBarWidget::TopBarWidget(QWidget *parent)
    : QWidget(parent)
{
    UIBuilder uiBuilder;

    auto *layout = new QHBoxLayout(this);
    layout->setContentsMargins(6, 4, 6, 4);
    layout->setSpacing(4);

    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    setMinimumHeight(40);

    setStyleSheet(
        "TopBarWidget { "
        "background-color: #07111F; "
        "border-bottom: 1px solid #223A5E; "
        "}"
        );

    homeButton = uiBuilder.createTopBarButton("Home", this);
    selectSpacecraftButton = uiBuilder.createTopBarButton("Select Spacecraft", this);
    startSimulationButton = uiBuilder.createTopBarButton("Start Simulation", this);
    exportTelemetryButton = uiBuilder.createTopBarButton("Export Telemetry", this);
    exportTelemetryButton->setEnabled(false);

    controlsHelpButton = uiBuilder.createTopBarButton("Controls", this);
    coordinateHelpButton = uiBuilder.createTopBarButton("Coordinate Frames", this);
    settingsButton = uiBuilder.createTopBarButton("Settings", this);

    // Keep the main navigation and utility buttons together. Flexible button
    // widths allow the layout to adapt to narrower windows without clipping.
    layout->addWidget(homeButton);
    layout->addWidget(selectSpacecraftButton);
    layout->addWidget(startSimulationButton);
    layout->addWidget(exportTelemetryButton);
    layout->addStretch(1);
    layout->addWidget(controlsHelpButton);
    layout->addWidget(coordinateHelpButton);
    layout->addWidget(settingsButton);

    connect(homeButton, &QPushButton::clicked,
            this, &TopBarWidget::homeRequested);
    connect(selectSpacecraftButton, &QPushButton::clicked,
            this, &TopBarWidget::selectSpacecraftRequested);
    connect(startSimulationButton, &QPushButton::clicked,
            this, &TopBarWidget::startSimulationRequested);
    connect(exportTelemetryButton, &QPushButton::clicked,
            this, &TopBarWidget::exportTelemetryRequested);
    connect(controlsHelpButton, &QPushButton::clicked,
            this, &TopBarWidget::controlsHelpRequested);
    connect(coordinateHelpButton, &QPushButton::clicked,
            this, &TopBarWidget::coordinateHelpRequested);
    connect(settingsButton, &QPushButton::clicked,
            this, &TopBarWidget::settingsRequested);
}

void TopBarWidget::setExportEnabled(bool enabled)
{
    exportTelemetryButton->setEnabled(enabled);
}