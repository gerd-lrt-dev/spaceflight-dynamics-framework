#include "homepage.h"
#include "uibuilder.h"

#include <QPixmap>

Homepage::Homepage(QWidget *parent)
    : QWidget(parent)
{
    setupUI();
    setupConnections();
}

void Homepage::setupUI()
{
    UIBuilder uiBuilder;

    vHomepageLayout = new QVBoxLayout(this);

    vHomepageLayout->setContentsMargins(40, 30, 40, 30);
    vHomepageLayout->setSpacing(0);

    // =====================================================
    // Logo
    // =====================================================

    QLabel *logoLabel = new QLabel(this);

    QPixmap logoPixmap(":/new/logo/Resources/Images/newlogo.png");

    logoPixmap = logoPixmap.scaled(
        600,
        600,
        Qt::KeepAspectRatio,
        Qt::SmoothTransformation);

    logoLabel->setPixmap(logoPixmap);
    logoLabel->setAlignment(Qt::AlignCenter);

    // =====================================================
    // Framework Description
    // =====================================================

    QLabel *description = new QLabel(
        "High Fidelity Spaceflight Simulation\n"
        "and Research Environment",
        this);

    description->setAlignment(Qt::AlignCenter);

    description->setStyleSheet(
        "font-size: 18px;"
        "font-weight: 500;"
        "color: #AFC2D6;"
        "letter-spacing: 0.5px;"
        "line-height: 28px;"
        );

    // =====================================================
    // Release Information
    // =====================================================

    QLabel *releaseInfo = new QLabel(
        "Development Build\n"
        "Current Milestone: M1 - Full 6DOF Simulation",
        this);

    releaseInfo->setAlignment(Qt::AlignCenter);

    releaseInfo->setStyleSheet(
        "font-size: 14px;"
        "color: #7F93AA;"
        "padding-top: 10px;"
        "line-height: 22px;"
        );

    // =====================================================
    // Navigation Buttons
    // =====================================================

    selectSpacecraftButton =
        uiBuilder.createButton("Select Spacecraft", this);

    startSimulationButton =
        uiBuilder.createButton("Start Simulation", this);

    // =====================================================
    // Copyright
    // =====================================================

    QLabel *copyright =
        uiBuilder.createCopyright(
            "© 2026 gerd-lrt-dev | Spaceflight Dynamics Framework",
            this);

    // =====================================================
    // Layout Assembly
    // =====================================================

    vHomepageLayout->addStretch();

    vHomepageLayout->addWidget(logoLabel);

    vHomepageLayout->addSpacing(10);

    vHomepageLayout->addWidget(description);

    vHomepageLayout->addSpacing(20);

    vHomepageLayout->addWidget(releaseInfo);

    vHomepageLayout->addSpacing(40);

    vHomepageLayout->addWidget(
        selectSpacecraftButton,
        0,
        Qt::AlignHCenter);

    vHomepageLayout->addSpacing(15);

    vHomepageLayout->addWidget(
        startSimulationButton,
        0,
        Qt::AlignHCenter);

    vHomepageLayout->addStretch();

    vHomepageLayout->addWidget(copyright);

    vHomepageLayout->setAlignment(Qt::AlignCenter);
}

void Homepage::setupConnections()
{
    connect(selectSpacecraftButton,
            &QPushButton::clicked,
            this,
            &Homepage::selectSpacecraftRequested);

    connect(startSimulationButton,
            &QPushButton::clicked,
            this,
            &Homepage::startSimulationRequested);
}
