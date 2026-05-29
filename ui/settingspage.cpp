#include "settingspage.h"

#include <QLabel>
#include <QVBoxLayout>

SettingsPage::SettingsPage(QWidget *parent)
    : QWidget(parent)
{
    setupUI();
}

void SettingsPage::setupUI()
{
    setStyleSheet(
        "QWidget { "
        "background-color: #07111F; "
        "color: #E6EEF8; "
        "}"

        "QLabel#TitleLabel { "
        "font-size: 32px; "
        "font-weight: 700; "
        "color: #E6EEF8; "
        "}"

        "QLabel#SubtitleLabel { "
        "font-size: 16px; "
        "color: #38BDF8; "
        "font-weight: 600; "
        "}"

        "QLabel#InfoLabel { "
        "font-size: 15px; "
        "color: #A8BACD; "
        "line-height: 22px; "
        "}"

        "QLabel#VersionLabel { "
        "font-size: 13px; "
        "color: #7F93AA; "
        "}"
        );

    auto *layout = new QVBoxLayout(this);

    layout->setContentsMargins(50, 50, 50, 50);
    layout->setSpacing(24);

    QLabel *title =
        new QLabel("SETTINGS");

    title->setObjectName("TitleLabel");

    QLabel *subtitle =
        new QLabel("SDF Research Release");

    subtitle->setObjectName("SubtitleLabel");

    QLabel *info =
        new QLabel(
            "The settings subsystem is currently under construction "
            "and planned for the upcoming v0.2 release cycle.\n\n"

            "Future planned functionality includes:\n"
            "- Input configuration\n"
            "- Simulation presets\n"
            "- Telemetry customization\n"
            "- UI theme adjustments\n"
            "- ROS2 communication settings\n"
            "- Runtime simulation parameters\n"
            "- Data logging configuration");

    info->setWordWrap(true);
    info->setObjectName("InfoLabel");

    QLabel *version =
        new QLabel(
            "Planned availability: v0.2 - SDF Research Release");

    version->setObjectName("VersionLabel");

    layout->addWidget(title);
    layout->addWidget(subtitle);
    layout->addSpacing(10);
    layout->addWidget(info);
    layout->addStretch();
    layout->addWidget(version);
}
