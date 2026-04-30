#include "homepage.h"
#include "uibuilder.h"

Homepage::Homepage(QWidget *parent) :
    QMainWindow(parent)
{
    setupStackedWidget();
    setupThread();
    setupConnections();

    bool jsonLoaded = configManager_.loadConfig(":/configs/Resources/configs/lander.json");
    qDebug() << "[homepage] constructor: Json config file successfully loaded";
}

Homepage::~Homepage()
{
    simulationThread->quit();
    simulationThread->wait();
}

void Homepage::setupStackedWidget()
{
    // Build central QWidget
    centralWidget = new QWidget(this);

    // Build vertical box layout
    QVBoxLayout *layout = new QVBoxLayout(centralWidget);

    // Build stackedWidget for pages
    stackedWidget = new QStackedWidget(this);

    // Create pages
    QWidget *homepage = createHomePage(stackedWidget);
    cockpit = new cockpitPage(this);

    spacecraftSelectionPage_ = new SpacecraftSelectionPage(configManager_, this);

    // Add pages to stacked widget
    stackedWidget->addWidget(homepage);
    stackedWidget->addWidget(cockpit);
    stackedWidget->addWidget(spacecraftSelectionPage_);

    // Add stackedwidget to Layout
    layout->addWidget(stackedWidget);

    // Set central page
    setCentralWidget(centralWidget);
}

QWidget* Homepage::createHomePage(QStackedWidget *stackedWidget)
{
    //Create Widget
    QWidget *homepage = new QWidget(this);

    // Build instance for ui building class
    UIBuilder uiBuilder;

    // Initialize Layout
    vHomepageLayout = new QVBoxLayout(homepage);

    // Add title
    QLabel *title = uiBuilder.createPageTitle("Lunar Lander Simulation", this);

    // Add logo
    QLabel *logoLabel   = new QLabel(this);
    QPixmap logoPixmap(":/new/logo/Resources/Images/logo.png");
    logoPixmap = logoPixmap.scaled(200, 200, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    logoLabel->setPixmap(logoPixmap);
    logoLabel->setAlignment(Qt::AlignCenter);

    // Add copryright
    QLabel *copyright = uiBuilder.createCopyright("© gerd-lrt-dev", this);

    // Add Buttons
    selectSpacecraftButton = uiBuilder.createButton("Select Spacecraft", this);
    startSimulationButton = uiBuilder.createButton("Start Simulation", this);

    // Add widgets to Layout
    vHomepageLayout->addSpacing(30);
    vHomepageLayout->addWidget(title);
    vHomepageLayout->addSpacing(15);
    vHomepageLayout->addWidget(logoLabel);
    vHomepageLayout->addSpacing(15);
    vHomepageLayout->addWidget(copyright);
    vHomepageLayout->addSpacing(15);
    vHomepageLayout->addWidget(selectSpacecraftButton, 0, Qt::AlignHCenter);
    vHomepageLayout->addSpacing(15);
    vHomepageLayout->addWidget(startSimulationButton, 0, Qt::AlignHCenter);
    vHomepageLayout->setAlignment(Qt::AlignTop);
    vHomepageLayout->setAlignment(Qt::AlignCenter);

    return homepage;
}

void Homepage::setupConnections()
{
    connect(startSimulationButton, &QPushButton::clicked, this, [this]
            {
                handleJsonConfig(jsonConfigStr);
                stackedWidget->setCurrentWidget(cockpit);
            });

    // Connect thrust input from user with backend

    connect(cockpit, &cockpitPage::flightCmdRequested, simulationWorker, &SimulationWorker::setFlightCommand);

    connect(cockpit, &cockpitPage::autopilotToggled, simulationWorker, &SimulationWorker::setAutopilotFlag);

    connect(selectSpacecraftButton, &QPushButton::clicked, this, [this]
            {
                stackedWidget->setCurrentWidget(spacecraftSelectionPage_);
            });

    connect(spacecraftSelectionPage_, &SpacecraftSelectionPage::backRequested, this, [this]
            {
                stackedWidget->setCurrentIndex(0);
            });

    connect(spacecraftSelectionPage_, &SpacecraftSelectionPage::spacecraftSelected, this, [this](QString spacecraft)
            {
                selectedJsonConfigStr = spacecraft;
                handleJsonConfig(jsonConfigStr);
                stackedWidget->setCurrentWidget(cockpit);
            });
    connect(this, &Homepage::sendJsonToSpacecraftSelectPage, spacecraftSelectionPage_, &SpacecraftSelectionPage::receiveJsonConfigStr);
}

void Homepage::setupThread()
{
    // Instance thread
    simulationThread = new QThread(this);

    // Instance class
    simulationWorker = new SimulationWorker();

    simulationWorker->moveToThread(simulationThread);

    // Build connections
    connect(simulationThread, &QThread::finished, simulationWorker, &QObject::deleteLater);

    connect(this, &QObject::destroyed, simulationThread, &QThread::quit);

    connect(cockpit, &cockpitPage::startRequested,
            simulationWorker, &SimulationWorker::start);

    connect(cockpit, &cockpitPage::pauseRequested,
            simulationWorker, &SimulationWorker::pause);

    connect(cockpit, &cockpitPage::stopConfirmed,
            simulationWorker, &SimulationWorker::stop);

    connect(this, &Homepage::sendJsonToWorker, simulationWorker,
            &SimulationWorker::receiveJsonConfig, Qt::QueuedConnection);

    connect(simulationWorker, &SimulationWorker::stateUpdated,
            cockpit, &cockpitPage::onStateUpdated);


    // Start simulation
    simulationThread->start();
}

void Homepage::handleJsonConfig(const QString &jsonConfigStr_)
{
// Catch failure when config file is empty
    if (selectedJsonConfigStr.isEmpty())
    {
        QString jsonConfigDefault = configManager_.defaultSpacecraftJson();
        emit sendJsonToWorker(jsonConfigDefault);
    }
    else
    {
        emit sendJsonToWorker(selectedJsonConfigStr);
    }
}

