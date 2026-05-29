#include "controlshelppage.h"

#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>

ControlsHelpPage::ControlsHelpPage(QWidget *parent)
    : QWidget(parent)
{
    setupUI();
}

void ControlsHelpPage::setupUI()
{
    setStyleSheet(
        "QWidget { "
        "background-color: #07111F; "
        "color: #E6EEF8; "
        "}"

        "QFrame#SectionFrame { "
        "background-color: #0E1A2B; "
        "border: 1px solid #223A5E; "
        "border-radius: 10px; "
        "}"

        "QLabel#TitleLabel { "
        "font-size: 32px; "
        "font-weight: 700; "
        "color: #E6EEF8; "
        "padding-bottom: 12px; "
        "}"

        "QLabel#SectionTitle { "
        "font-size: 18px; "
        "font-weight: bold; "
        "color: #38BDF8; "
        "padding-bottom: 6px; "
        "}"

        "QLabel#KeyLabel { "
        "background-color: #13233A; "
        "border: 1px solid rgba(220,230,240,0.18); "
        "border-radius: 6px; "
        "padding: 6px 12px; "
        "font-family: Consolas; "
        "font-size: 14px; "
        "font-weight: bold; "
        "color: #E6EEF8; "
        "min-width: 80px; "
        "}"

        "QLabel#DescLabel { "
        "font-size: 14px; "
        "color: #C7D3E0; "
        "padding-left: 10px; "
        "}"
        );

    mainLayout = new QVBoxLayout(this);

    mainLayout->setContentsMargins(40, 30, 40, 30);
    mainLayout->setSpacing(24);

    QLabel *title = new QLabel("CONTROL REFERENCE");
    title->setObjectName("TitleLabel");

    mainLayout->addWidget(title);

    // =====================================================
    // MAIN ENGINE
    // =====================================================

    QFrame *mainEngineFrame = new QFrame();
    mainEngineFrame->setObjectName("SectionFrame");

    QVBoxLayout *mainEngineLayout =
        new QVBoxLayout(mainEngineFrame);

    mainEngineLayout->addWidget(
        createSectionTitle("MAIN ENGINE"));

    mainEngineLayout->addWidget(
        createControlRow("↑", "Increase main engine thrust"));

    mainEngineLayout->addWidget(
        createControlRow("↓", "Decrease main engine thrust"));

    mainLayout->addWidget(mainEngineFrame);

    // =====================================================
    // RCS TRANSLATION
    // =====================================================

    QFrame *rcsFrame = new QFrame();
    rcsFrame->setObjectName("SectionFrame");

    QVBoxLayout *rcsLayout =
        new QVBoxLayout(rcsFrame);

    rcsLayout->addWidget(
        createSectionTitle("RCS TRANSLATION"));

    rcsLayout->addWidget(
        createControlRow("W", "+Y translation"));

    rcsLayout->addWidget(
        createControlRow("S", "-Y translation"));

    rcsLayout->addWidget(
        createControlRow("D", "+X translation"));

    rcsLayout->addWidget(
        createControlRow("A", "-X translation"));

    rcsLayout->addWidget(
        createControlRow("E", "+Z translation"));

    rcsLayout->addWidget(
        createControlRow("Q", "-Z translation"));

    mainLayout->addWidget(rcsFrame);

    // =====================================================
    // SIMULATION
    // =====================================================

    QFrame *simFrame = new QFrame();
    simFrame->setObjectName("SectionFrame");

    QVBoxLayout *simLayout =
        new QVBoxLayout(simFrame);

    simLayout->addWidget(
        createSectionTitle("SIMULATION"));

    simLayout->addWidget(
        createControlRow("START", "Start simulation"));

    simLayout->addWidget(
        createControlRow("PAUSE", "Pause simulation"));

    simLayout->addWidget(
        createControlRow("STOP", "Stop / reset simulation"));

    simLayout->addWidget(
        createControlRow("AUTOPILOT", "Toggle autopilot"));

    mainLayout->addWidget(simFrame);

    // =====================================================
    // INFO
    // =====================================================

    QLabel *info =
        new QLabel(
            "The current control system is keyboard-first and "
            "prepared for future controller integration.");

    info->setWordWrap(true);

    info->setStyleSheet(
        "color: #8FA4BA;"
        "font-size: 13px;"
        "padding-top: 10px;"
        );

    mainLayout->addWidget(info);

    mainLayout->addStretch();
}

QWidget* ControlsHelpPage::createControlRow(
    const QString& key,
    const QString& description)
{
    QWidget *row = new QWidget();

    QHBoxLayout *layout = new QHBoxLayout(row);

    layout->setContentsMargins(6, 4, 6, 4);

    QLabel *keyLabel = new QLabel(key);
    keyLabel->setObjectName("KeyLabel");
    keyLabel->setAlignment(Qt::AlignCenter);

    QLabel *descLabel = new QLabel(description);
    descLabel->setObjectName("DescLabel");

    layout->addWidget(keyLabel, 0);
    layout->addWidget(descLabel, 1);

    return row;
}

QWidget* ControlsHelpPage::createSectionTitle(
    const QString& title)
{
    QLabel *label = new QLabel(title);
    label->setObjectName("SectionTitle");

    return label;
}
