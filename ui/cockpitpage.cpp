#include "cockpitpage.h"
#include "Thrust/FueltankStruct.h"

#include <QGridLayout>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QString>
#include <QtMath>
#include <QThread>

// ------------------------------------------------
// Constructor
// ------------------------------------------------
cockpitPage::cockpitPage(QWidget *parent)
    : QWidget(parent),
    lcdTime(nullptr),
    MCI_lcdPosX(nullptr),
    MCI_lcdPosY(nullptr),
    MCI_lcdPosZ(nullptr),
    LNF_lcdLat(nullptr),
    LNF_lcdLon(nullptr),
    LNF_lcdRot(nullptr),
    LNF_lcdVelX(nullptr),
    LNF_lcdVelY(nullptr),
    LNF_lcdVelZ(nullptr),
    LNF_lcdRoll(nullptr),
    LNF_lcdPitch(nullptr),
    LNF_lcdYaw(nullptr),
    LNF_totalVel(nullptr),
    LNF_lcdThrust_BX(nullptr),
    LNF_lcdThrust_BY(nullptr),
    LNF_lcdThrust_BZ(nullptr),
    LNF_lcdTargetThrust_BX(nullptr),
    LNF_lcdTargetThrust_BY(nullptr),
    LNF_lcdTargetThrust_BZ(nullptr),
    lcdGLoad(nullptr),
    lcdFuelMass(nullptr),
    lcdFuelFlow(nullptr)
{
    initializeQTObjects();
    setupUI();

    // Prepare for keyboard input
    initializeControlInput();

    setupConnections();
}

// ------------------------------------------------
// Helper
// ------------------------------------------------
static void configureLCD(QLCDNumber* lcd, int digits)
{
    lcd->setDigitCount(digits);
    lcd->setSegmentStyle(QLCDNumber::Flat);
    lcd->setSmallDecimalPoint(false);
}

QWidget* cockpitPage::createLcdField(const QString& title, QLCDNumber* lcd, int digits)
{
    QWidget *field = new QWidget();
    QVBoxLayout *layout = new QVBoxLayout(field);

    layout->setContentsMargins(2, 2, 2, 2);
    layout->setSpacing(2);

    QLabel *label = new QLabel(title);
    label->setAlignment(Qt::AlignCenter);
    label->setStyleSheet("color: #AFC7DF; font-size: 10px;");

    lcd = new QLCDNumber();
    configureLCD(lcd, digits);

    layout->addWidget(label);
    layout->addWidget(lcd);

    return field;
}

// ------------------------------------------------
// UI Setup
// ------------------------------------------------
void cockpitPage::setupUI()
{
    qDebug() << "setupUI called on:" << this;
    QGridLayout *mainLayout = new QGridLayout(this);
    mainLayout->setSpacing(12);
    mainLayout->setContentsMargins(10, 10, 10, 10);

    mainLayout->setRowStretch(0, 1);
    mainLayout->setRowStretch(1, 1);

    mainLayout->setColumnStretch(0, 1);
    mainLayout->setColumnStretch(1, 1);
    mainLayout->setColumnStretch(2, 1);

    setStyleSheet(
        "QWidget { background-color: #0E1624; color: #D6E1F0; }"
        "QGroupBox { border: 1px solid #2F4A72; border-radius: 8px; margin-top: 22px; padding-top: 18px; font-weight: bold; }"
        "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; left: 12px; top: 6px; padding: 0 6px; color: #4FC3F7; }"
        "QLCDNumber { background-color: #000000; color: #4FC3F7; border: 2px solid #2F4A72; border-radius: 4px; min-height: 26px; }"
        "QSlider::groove:horizontal { height: 6px; background: #2F4A72; }"
        "QSlider::handle:horizontal { width: 14px; background: #4FC3F7; margin: -4px 0; border-radius: 3px; }"
        );

    // ================= NAV =================
    QGroupBox *navBox = setupNavBox();

    // ================= ENGINE =================
    QGroupBox *engineBox = setupEngineBox();

    // ================= FUEL =================
    QGroupBox *fuelBox = setupFuelBox();

    // ================= STATUS =================
    QGroupBox *statusBox = setupStatusBox();

    // ================= LANDING VIEW =================
    QGroupBox *landingBox = setupLandingBox();

    // ================= GRID ASSEMBLY =================
    mainLayout->addWidget(navBox,     0, 0);
    mainLayout->addWidget(fuelBox,    1, 0);
    mainLayout->addWidget(landingBox, 0, 1, 2, 1);
    mainLayout->addWidget(engineBox,  0, 2);
    mainLayout->addWidget(statusBox,  1, 2);
}

void cockpitPage::initializeQTObjects()
{
    // *********** LCD OBJECTS *************

    // =====================================================
    // Navigation Instruments
    // =====================================================

    lcdTime = new QLCDNumber();

    // Moon Centered Inertial (physical truth)
    MCI_lcdPosX = new QLCDNumber();
    MCI_lcdPosY = new QLCDNumber();
    MCI_lcdPosZ = new QLCDNumber();

    // Local Navigation Frame
    LNF_lcdLat = new QLCDNumber();
    LNF_lcdLon = new QLCDNumber();
    LNF_lcdRot = new QLCDNumber();

    LNF_lcdVelX = new QLCDNumber();
    LNF_lcdVelY = new QLCDNumber();
    LNF_lcdVelZ = new QLCDNumber();

    LNF_lcdRoll = new QLCDNumber();
    LNF_lcdPitch = new QLCDNumber();
    LNF_lcdYaw = new QLCDNumber();

    LNF_totalVel = new QLCDNumber();

    // =====================================================
    // Engine Instruments
    // =====================================================

    LNF_lcdThrust_BX = new QLCDNumber();
    LNF_lcdThrust_BY = new QLCDNumber();
    LNF_lcdThrust_BZ = new QLCDNumber();

    LNF_lcdTargetThrust_BX = new QLCDNumber();
    LNF_lcdTargetThrust_BY = new QLCDNumber();
    LNF_lcdTargetThrust_BZ = new QLCDNumber();

    lcdGLoad = new QLCDNumber();

    // =====================================================
    // Fuel Instruments
    // =====================================================

    lcdFuelMass = new QLCDNumber();
    lcdFuelFlow = new QLCDNumber();
}

void cockpitPage::initializeControlInput()
{
    m_inputMapper = new inputmapper(thrustSlider, lblThrustCmd, this);
    setFocusPolicy(Qt::StrongFocus);
    setFocus();
}

// ================= NAV =================
QGroupBox *cockpitPage::setupNavBox()
{
    QGroupBox *navBox = new QGroupBox("NAV");
    QGridLayout *navLayout = new QGridLayout(navBox);

    QVector<QLCDNumber*> timePanel;
    QVector<QLCDNumber*> absolutePosPanel;
    QVector<QLCDNumber*> absoluteRotPanel;
    QVector<QLCDNumber*> absoluteTransVelPanel;
    QVector<QLCDNumber*> absoluteAngVelPanel;

    timePanel.push_back(lcdTime);

    absolutePosPanel.push_back(MCI_lcdPosX);
    absolutePosPanel.push_back(MCI_lcdPosY);
    absolutePosPanel.push_back(MCI_lcdPosZ);

    absoluteRotPanel.push_back(LNF_lcdLat);
    absoluteRotPanel.push_back(LNF_lcdLon);
    absoluteRotPanel.push_back(LNF_lcdRot);

    absoluteTransVelPanel.push_back(LNF_lcdVelX);
    absoluteTransVelPanel.push_back(LNF_lcdVelY);
    absoluteTransVelPanel.push_back(LNF_lcdVelZ);

    absoluteAngVelPanel.push_back(LNF_lcdRoll);
    absoluteAngVelPanel.push_back(LNF_lcdPitch);
    absoluteAngVelPanel.push_back(LNF_lcdYaw);

    QWidget *timeDetailBox = uibuilder.setupDetailBox(timePanel, {"TIME [s]"}, "SIMULATION TIME DATA", 1);
    QWidget *absPosDetailBox = uibuilder.setupDetailBox(absolutePosPanel, {"X [m]", "Y [m]", "Z [m]"}, "MCI_POSITION", 3);
    QWidget *absRotDetailBox = uibuilder.setupDetailBox(absoluteRotPanel, {"LAT [°]", "LON [°]", "ROT [°]"}, "LNF_ROTATION", 3);
    QWidget *absTransVelDetailBox = uibuilder.setupDetailBox(absoluteTransVelPanel, {"VX [m/s]", "VY [m/s]", "VZ [m/s]"}, "LNF_VELOCITY", 3);
    QWidget *absAngVelDetailBox = uibuilder.setupDetailBox(absoluteAngVelPanel, {"ROLL [°/s]", "PITCH [°/s]", "YAW [°/s]"}, "LNF_ANGULAR VEL", 3);

    navLayout->addWidget(timeDetailBox, 0, 0, 1, 2);
    navLayout->addWidget(absPosDetailBox, 1, 0);
    navLayout->addWidget(absRotDetailBox, 1, 1);
    navLayout->addWidget(absTransVelDetailBox, 2, 0);
    navLayout->addWidget(absAngVelDetailBox, 2, 1);

    navLayout->setColumnStretch(0, 1);
    navLayout->setColumnStretch(1, 1);

    navLayout->setRowStretch(0, 1);
    navLayout->setRowStretch(1, 1);
    navLayout->setRowStretch(2, 1);

    return navBox;
}

// ================= ENGINE =================
QGroupBox *cockpitPage::setupEngineBox()
{
    QGroupBox *engineBox = new QGroupBox("ENGINE");
    QGridLayout *engineLayout = new QGridLayout(engineBox);

    QVector<QLCDNumber*> currentThrustPanel;
    QVector<QLCDNumber*> targetThrustPanel;

    currentThrustPanel.push_back(LNF_lcdThrust_BX);
    currentThrustPanel.push_back(LNF_lcdThrust_BY);
    currentThrustPanel.push_back(LNF_lcdThrust_BZ);
    targetThrustPanel.push_back(LNF_lcdTargetThrust_BX);
    targetThrustPanel.push_back(LNF_lcdTargetThrust_BY);
    targetThrustPanel.push_back(LNF_lcdTargetThrust_BZ);

    QWidget *currentThrustDetailBox = uibuilder.setupDetailBox(currentThrustPanel, {"Thrust [N] X:", "Thrust [N] Y:", "Thrust [N] Z:"}, "LNF_CURRENT THRUST", 3);
    QWidget *targetThrustDetailBox  = uibuilder.setupDetailBox(targetThrustPanel, {"Thrust [N] X:", "Thrust [N] Y:", "Thrust [N] Z:"}, "LNF_TARGET THRUST", 3);
    QWidget *GLoadDetailBox         = uibuilder.setupDetailBox({lcdGLoad}, {"GLoad [m/s²]"}, "ACCELERATION", 1);

    engineLayout->addWidget(currentThrustDetailBox, 0, 0);
    engineLayout->addWidget(targetThrustDetailBox, 0, 1);
    engineLayout->addWidget(GLoadDetailBox, 1, 0, 1, 2);

    engineLayout->setColumnStretch(0, 1);
    engineLayout->setColumnStretch(1, 1);

    engineLayout->setRowStretch(0, 1);
    engineLayout->setRowStretch(1, 1);

    return engineBox;
}

// ================= FUEL =================
QGroupBox *cockpitPage::setupFuelBox()
{
    QGroupBox *fuelBox = new QGroupBox("FUEL");
    QVBoxLayout *fuelMainLayout = new QVBoxLayout(fuelBox);

    // -------------------------
    // General fuel metrics
    // -------------------------
    QGridLayout *fuelMetricsLayout = new QGridLayout();

    lcdFuelMass = new QLCDNumber();
    lcdFuelFlow = new QLCDNumber();

    configureLCD(lcdFuelMass, 8);
    configureLCD(lcdFuelFlow, 8);

    fuelMetricsLayout->addWidget(new QLabel("Total Fuel Mass [kg]"), 0, 0);
    fuelMetricsLayout->addWidget(lcdFuelMass,                       0, 1);

    fuelMetricsLayout->addWidget(new QLabel("Fuel Flow [kg/s]"),    1, 0);
    fuelMetricsLayout->addWidget(lcdFuelFlow,                       1, 1);

    fuelMainLayout->addLayout(fuelMetricsLayout);

    // -------------------------
    // Dynamic tank section
    // -------------------------
    QLabel *tankSectionTitle = new QLabel("TANK STATUS");
    tankSectionTitle->setStyleSheet("color: #4FC3F7; font-weight: bold;");
    fuelMainLayout->addWidget(tankSectionTitle);

    fuelTankContainer = new QWidget();
    fuelTankLayout = new QVBoxLayout(fuelTankContainer);
    fuelTankLayout->setContentsMargins(0, 0, 0, 0);
    fuelTankLayout->setSpacing(8);

    QScrollArea *scrollArea = new QScrollArea();
    scrollArea->setWidgetResizable(true);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setWidget(fuelTankContainer);
    scrollArea->setMinimumHeight(180);

    fuelMainLayout->addWidget(scrollArea);

    return fuelBox;
}

// ================= STATUS =================
QGroupBox *cockpitPage::setupStatusBox()
{
    QGroupBox *statusBox = new QGroupBox("STATUS");
    QVBoxLayout *statusLayout = new QVBoxLayout(statusBox);

    // --- HULL ---
    lblHullStatus = new QLabel("HULL: OK");
    lblHullStatus->setStyleSheet("color: lime; font-weight: bold;");
    statusLayout->addWidget(lblHullStatus);

    // --- Autopilot ---
    lblAutopilotStatus = new QLabel("AUTOPILOT: OFF");
    lblAutopilotStatus->setStyleSheet("color: gray; font-weight: bold;");
    statusLayout->addWidget(lblAutopilotStatus);

    // --- Autopilot blink timer ---
    autopilotBlinkTimer = new QTimer(this);
    autopilotBlinkTimer->setInterval(500); // ms
    autopilotBlinkOn = false;

    // --- Controller output ---
    lblControllerOutput = new QLabel("Controller Output: OK");
    lblControllerOutput->setStyleSheet("Color: white; font-weight: bold;");
    statusLayout->addWidget(lblControllerOutput);

    statusLayout->addStretch();

    return statusBox;
}

// ================= LANDING VIEW =================
QGroupBox *cockpitPage::setupLandingBox()
{
    QGroupBox *landingBox = new QGroupBox("LANDING VIEW");
    QVBoxLayout *landingLayout = new QVBoxLayout(landingBox);

    landingView = new LandingView(this);
    landingView->setMinimumSize(240, 180);
    landingLayout->addWidget(landingView, 1);

    // === Thrust Control Console ===
    QGroupBox *thrustBox = new QGroupBox("CONTROL UNIT");
    QVBoxLayout *thrustLayout = new QVBoxLayout(thrustBox);

    thrustSlider = new QSlider(Qt::Horizontal);
    thrustSlider->setRange(0, 100);
    thrustSlider->setValue(0);

    lblThrustCmd = new QLabel("Commanded Thrust: 0 %");
    lblThrustCmd->setAlignment(Qt::AlignCenter);

    thrustLayout->addWidget(thrustSlider);
    thrustLayout->addWidget(lblThrustCmd);

    landingLayout->addWidget(thrustBox);

    // === Simulation Controls ===
    QHBoxLayout *simControlLayout = new QHBoxLayout();
    btnSimStart = new QPushButton("START");
    btnSimPause = new QPushButton("PAUSE");
    btnSimStop  = new QPushButton("STOP");

    simControlLayout->addWidget(btnSimStart);
    simControlLayout->addWidget(btnSimPause);
    simControlLayout->addWidget(btnSimStop);

    // === Autopilot Toggle ===
    btnAutopilot = new QPushButton("AUTOPILOT OFF");
    btnAutopilot->setCheckable(true);
    btnAutopilot->setStyleSheet(
        "QPushButton { background-color: #333; color: #AAA; font-weight: bold; padding: 6px; }"
        "QPushButton:checked { background-color: #00BCD4; color: black; }"
        );

    thrustLayout->addWidget(btnAutopilot);


    landingLayout->addLayout(simControlLayout);

    return landingBox;
}

// ------------------------------------------------
// Connections
// ------------------------------------------------
void cockpitPage::setupConnections()
{
    connect(btnSimStart, &QPushButton::clicked, this, &cockpitPage::startRequested);    ///< Emits signal
    connect(btnSimPause, &QPushButton::clicked, this, &cockpitPage::pauseRequested);    ///< Emits signal
    connect(btnSimStop,  &QPushButton::clicked, this, &cockpitPage::onStopClicked);     ///< Combined with private slot

    connect(thrustSlider, &QSlider::valueChanged, this, [this](int value)
    {
        lblThrustCmd->setText(QString("Commanded Thrust: %1 %").arg(value));
        collectedCmd.mainEngine = static_cast<float>(value) / 100.0;
        sendFlightCmd();
    });

    connect(btnAutopilot, &QPushButton::clicked, this, &cockpitPage::onAutopilotClicked);

    connect(autopilotBlinkTimer, &QTimer::timeout, this, &cockpitPage::onAutopilotBlinkTimeout);

    connect(m_inputMapper, &inputmapper::RCS_cmdRequested, this, [this](FlightCommand cmd)
    {
        collectedCmd.translation = cmd.translation;
        collectedCmd.rotation    = cmd.rotation;
        collectedCmd.stabilize   = cmd.stabilize;
        collectedCmd.killRotation = cmd.killRotation;
        sendFlightCmd();
    });
}

// ------------------------------------------------
// Update Interface
// ------------------------------------------------
void cockpitPage::updateTime(double t)
{
    if (!lcdTime)
    {
        qDebug() << "lcdTime is nullptr!";
        return;
    }
    lcdTime->display(QString::number(t, 'f', 2));
}
void cockpitPage::updatePosition(Vector3 pos)
{
    MCI_lcdPosX->display(QString::number(pos.x, 'f', 1));
    MCI_lcdPosY->display(QString::number(pos.y, 'f', 1));
    MCI_lcdPosZ->display(QString::number(pos.z, 'f', 1));
}

void cockpitPage::updateRotation(Vector3 rot)
{
    // TODO: Build own data type for rotational parameters
    LNF_lcdLat->display(QString::number(rot.x, 'f', 1));
    LNF_lcdLon->display(QString::number(rot.y, 'f', 1));
    LNF_lcdRot->display(QString::number(rot.z, 'f', 1));
}

void cockpitPage::updateVelocity(Vector3 v)
{
    LNF_lcdVelX->display(QString::number(v.x, 'f', 1));
    LNF_lcdVelY->display(QString::number(v.y, 'f', 1));
    LNF_lcdVelZ->display(QString::number(v.z, 'f', 1));
}

void cockpitPage::updateAngularVelocity(Vector3 angV)
{
    // TODO: Build own data type for rotational parameters
    LNF_lcdRoll->display(QString::number(angV.x, 'f', 1));
    LNF_lcdPitch->display(QString::number(angV.y, 'f', 1));
    LNF_lcdYaw->display(QString::number(angV.z, 'f', 1));
}

void cockpitPage::rebuildFuelTankPanel(const QVector<FuelTank>& tanks)
{
    if (!fuelTankLayout)
        return;

    QLayoutItem *item;
    while ((item = fuelTankLayout->takeAt(0)) != nullptr)
    {
        if (item->widget())
            delete item->widget();
        delete item;
    }

    lcdTankMasses.clear();
    barTankFillLevels.clear();
    lblTankNames.clear();
    lblTankRoles.clear();

    for (const FuelTank& tank : tanks)
    {
        QGroupBox *tankBox = new QGroupBox();
        QVBoxLayout *tankBoxLayout = new QVBoxLayout(tankBox);
        tankBoxLayout->setContentsMargins(8, 8, 8, 8);
        tankBoxLayout->setSpacing(4);

        // Header
        QLabel *lblName = new QLabel(QString::fromStdString(tank.name));
        lblName->setStyleSheet("color: #D6E1F0; font-weight: bold;");

        QLabel *lblRole = new QLabel(QString("Role: %1").arg(QString::fromStdString(tank.role)));
        lblRole->setStyleSheet("color: #AFC7DF; font-size: 10px;");

        tankBoxLayout->addWidget(lblName);
        tankBoxLayout->addWidget(lblRole);

        // Metrics row
        QGridLayout *metricsLayout = new QGridLayout();

        QLCDNumber *lcdMass = new QLCDNumber();
        configureLCD(lcdMass, 8);
        lcdMass->display(QString::number(tank.mass, 'f', 1));

        QProgressBar *barFill = new QProgressBar();
        barFill->setRange(0, 100);
        barFill->setValue(static_cast<int>(tank.fillRatio() * 100.0));
        barFill->setTextVisible(true);
        barFill->setFormat("%p%");

        barFill->setStyleSheet(
            "QProgressBar {"
            "  border: 1px solid #2F4A72;"
            "  border-radius: 4px;"
            "  background-color: #09111B;"
            "  color: #D6E1F0;"
            "  text-align: center;"
            "}"
            "QProgressBar::chunk {"
            "  background-color: #4FC3F7;"
            "  border-radius: 3px;"
            "}"
            );

        QLabel *lblMass = new QLabel("Remaining [kg]");
        QLabel *lblFill = new QLabel(
            QString("Fill Level [%]  (%1 / %2 kg)")
                .arg(tank.mass, 0, 'f', 1)
                .arg(tank.capacity, 0, 'f', 1)
            );

        lblMass->setStyleSheet("color: #AFC7DF;");
        lblFill->setStyleSheet("color: #AFC7DF;");

        metricsLayout->addWidget(lblMass, 0, 0);
        metricsLayout->addWidget(lblFill, 0, 1);

        metricsLayout->addWidget(lcdMass, 1, 0);
        metricsLayout->addWidget(barFill, 1, 1);

        metricsLayout->setColumnStretch(0, 1);
        metricsLayout->setColumnStretch(1, 2);

        tankBoxLayout->addLayout(metricsLayout);

        fuelTankLayout->addWidget(tankBox);

        lcdTankMasses.push_back(lcdMass);
        barTankFillLevels.push_back(barFill);
        lblTankNames.push_back(lblName);
        lblTankRoles.push_back(lblRole);
    }

    fuelTankLayout->addStretch();
}

void cockpitPage::updateFuelTanks(const QVector<FuelTank>& tanks)
{
    if (tanks.size() != lcdTankMasses.size())
    {
        rebuildFuelTankPanel(tanks);
    }

    const int count = qMin(tanks.size(), lcdTankMasses.size());

    for (int i = 0; i < count; ++i)
    {
        const FuelTank& tank = tanks[i];

        lcdTankMasses[i]->display(QString::number(tank.mass, 'f', 1));

        int fillPercent = static_cast<int>(qBound(0.0, tank.fillRatio() * 100.0, 100.0));
        barTankFillLevels[i]->setValue(fillPercent);

        lblTankNames[i]->setText(QString::fromStdString(tank.name));
        lblTankRoles[i]->setText(QString("Role: %1").arg(QString::fromStdString(tank.role)));

        // Optional: leere Tanks rot markieren
        if (tank.empty())
        {
            barTankFillLevels[i]->setStyleSheet(
                "QProgressBar {"
                "  border: 1px solid #7A2B2B;"
                "  border-radius: 4px;"
                "  background-color: #09111B;"
                "  color: #D6E1F0;"
                "  text-align: center;"
                "}"
                "QProgressBar::chunk {"
                "  background-color: #E53935;"
                "  border-radius: 3px;"
                "}"
                );
        }
        else
        {
            barTankFillLevels[i]->setStyleSheet(
                "QProgressBar {"
                "  border: 1px solid #2F4A72;"
                "  border-radius: 4px;"
                "  background-color: #09111B;"
                "  color: #D6E1F0;"
                "  text-align: center;"
                "}"
                "QProgressBar::chunk {"
                "  background-color: #4FC3F7;"
                "  border-radius: 3px;"
                "}"
                );
        }
    }
}

void cockpitPage::updateAcceleration(double a)
{
    lcdGLoad->display(QString::number(a, 'f', 2));
}

void cockpitPage::updateThrust(Vector3 t)
{
    LNF_lcdThrust_BX->display(QString::number(t.x, 'f', 1));
    LNF_lcdThrust_BY->display(QString::number(t.y, 'f', 1));
    LNF_lcdThrust_BZ->display(QString::number(t.z, 'f', 1));
}
void cockpitPage::updateTargetThrust(Vector3 t)
{
    LNF_lcdTargetThrust_BX->display(QString::number(t.x, 'f', 1));
    LNF_lcdTargetThrust_BY->display(QString::number(t.y, 'f', 1));
    LNF_lcdTargetThrust_BZ->display(QString::number(t.z, 'f', 1));
}

void cockpitPage::updateFuelMass(double f)
{
    if (!lcdFuelMass)
    {
        qDebug() << "lcdFuelMass is nullptr!";
        return;
    }

    lcdFuelMass->display(QString::number(f, 'f', 1));
}

void cockpitPage::updateFuelFlow(double f)
{
    if (!lcdFuelFlow)
    {
        qDebug() << "lcdFuelFlow is nulltpr!";
        return;
    }

    lcdFuelFlow->display(QString::number(f, 'f', 2));
}

void cockpitPage::updateHullStatus(SpacecraftState spacecraftState_)
{
    if (spacecraftState_ == SpacecraftState::Operational)
    {
        lblHullStatus->setText("Operational");
        lblHullStatus->setStyleSheet("color: lime; font-weight: bold;");
    }
    else if (spacecraftState_ == SpacecraftState::Landed)
    {
        lblHullStatus->setText("Landed");
        lblHullStatus->setStyleSheet("color: green; font-weight: bold;");
    }
    else if (spacecraftState_ == SpacecraftState::Crashed)
    {
        lblHullStatus->setText("Crashed");
        lblHullStatus->setStyleSheet("color: red; font-weight: bold;");
    }
    else if (spacecraftState_ == SpacecraftState::Destroyed)
    {
        lblHullStatus->setText("Destroyed");
        lblHullStatus->setStyleSheet("color: red; font-weight: bold;");
    }
}

void cockpitPage::updateAutopilotStatus(bool active)
{
    if (active)
    {
        lblAutopilotStatus->setText("AUTOPILOT: ACTIVATED");
        lblAutopilotStatus->setStyleSheet("color: cyan; font-weight: bold;");
    }
    else
    {
        lblAutopilotStatus->setText("AUTOPILOT: OFF");
        lblAutopilotStatus->setStyleSheet("color: gray; font-weight: bold;");
    }
}

void cockpitPage::keyPressEvent(QKeyEvent* event)
{
    m_inputMapper->handleKeyPress(event);
    QWidget::keyPressEvent(event);
}

void cockpitPage::keyReleaseEvent(QKeyEvent* event)
{
    m_inputMapper->handleKeyRelease(event);
    QWidget::keyReleaseEvent(event);
}

void cockpitPage::sendFlightCmd()
{
    emit flightCmdRequested(collectedCmd);
}

// ------------------------------------------------
// Slots
// ------------------------------------------------
void cockpitPage::onStateUpdated(double time,
                                 const Vector3& pos,
                                 const Vector3& vel,
                                 const double& GLoad,
                                 SpacecraftState spacecraftState_,
                                 Vector3 thrust,
                                 Vector3 targetThrust,
                                 Vector3 thrustInPercentage,
                                 QVector<FuelTank> tanks,
                                 double fuelMass,
                                 double fuelFlow,
                                 QString consoleOutput_)
{
    updateTime(time);
    updatePosition(pos);
    updateRotation({0.0, 0.0, 0.0});
    updateVelocity(vel);
    updateAngularVelocity({0.0, 0.0, 0.0});
    updateAcceleration(qRound(GLoad * 100.0) / 100.0);
    updateThrust({qRound(thrust.x * 10.0) / 10.0, qRound(thrust.y * 10.0) / 10.0, qRound(-thrust.z * 10.0) / 10.0}); // TODO: Elimnate minus when coordinate transformation class is ready
    updateTargetThrust({qRound(targetThrust.x * 10.0) / 10.0, qRound(targetThrust.y * 10.0) / 10.0, qRound(-targetThrust.z * 10.0) / 10.0}); // TODO: Elimnate minus when coordinate transformation class is ready

    updateFuelTanks(tanks);
    updateFuelMass(qRound(fuelMass * 10.0) / 10.0);
    updateFuelFlow(qRound(fuelFlow * 100.0) / 100.0);
    updateHullStatus(spacecraftState_);

    landingView->setPositionENU(pos);
    landingView->setVelocityENU(vel);
    landingView->setYawDeg(0.0);          // DUMMY later from Quaternion/Euler
    landingView->setTargetENU({0,0,0});   // DUMMY
    landingView->setThrust(-thrustInPercentage.z);
    landingView->setRCSActive(thrust);
    landingView->setHullIntact(spacecraftState_);

    (autopilotActive) ? consoleOutput(consoleOutput_) : consoleOutput("No controlling active");

}

void cockpitPage::onStopClicked()
{
    auto reply = QMessageBox::question(
        this,
        "Stop Simulation",
        "If you continue, the simulation will be reset.\nDo you want to proceed?",
        QMessageBox::Yes | QMessageBox::No
        );

    if (reply == QMessageBox::Yes) {
        emit stopConfirmed();
    }
}

void cockpitPage::onAutopilotClicked()
{
    autopilotActive = btnAutopilot->isChecked();

    if (autopilotActive)
    {
        btnAutopilot->setText("AUTOPILOT ON");
        autopilotBlinkTimer->start();
    }
    else
    {
        btnAutopilot->setText("AUTOPILOT OFF");
        autopilotBlinkTimer->stop();
        lblAutopilotStatus->setStyleSheet("color: gray; font-weight: bold;");
    }

    updateAutopilotStatus(autopilotActive);
    emit autopilotToggled(autopilotActive);
}

void cockpitPage::onAutopilotBlinkTimeout()
{
    if (!autopilotActive)
    {
        autopilotBlinkTimer->stop();
        lblAutopilotStatus->setStyleSheet("color: gray; font-weight: bold;");
        return;
    }

    autopilotBlinkOn = !autopilotBlinkOn;

    if (autopilotBlinkOn)
        lblAutopilotStatus->setStyleSheet("color: cyan; font-weight: bold;");
    else
        lblAutopilotStatus->setStyleSheet("color: black; font-weight: bold;");
}

void cockpitPage::consoleOutput(const QString& output)
{
    lblControllerOutput->setText(output);
}
