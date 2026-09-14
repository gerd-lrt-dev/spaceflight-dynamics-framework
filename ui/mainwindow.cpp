#include "mainwindow.h"

#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setupUI();
    setupThread();
    setupConnections();

    const bool jsonLoaded =
        configManager_.loadConfig(":/configs/Resources/configs/lander.json");

    qDebug() << "[MainWindow] JSON config loaded:" << jsonLoaded;
}

MainWindow::~MainWindow()
{
    if (simulationThread)
    {
        simulationThread->quit();
        simulationThread->wait();
    }
}

void MainWindow::setupUI()
{
    QWidget *centralWidget = new QWidget(this);

    // Full Backgorund Style
    centralWidget->setStyleSheet(
        "background-color: #07111F;"
        "color: #E6EEF8;"
        );

    vLayout = new QVBoxLayout(centralWidget);
    vLayout->setContentsMargins(0, 0, 0, 0);
    vLayout->setSpacing(0);

    // Widgets
    topBar          = new TopBarWidget(this);
    stackedWidget   = new QStackedWidget(this);

    // Pages
    homepage            = new Homepage(this);
    selectionPage       = new SpacecraftSelectionPage(configManager_, this);
    cockpit             = new cockpitPage(this);
    controlsHelpPage    = new ControlsHelpPage(this);
    coordinateHelpPage  = new CoordinateHelpPage(this);
    settingsPage        = new SettingsPage(this);

    stackedWidget->addWidget(homepage);
    stackedWidget->addWidget(selectionPage);
    stackedWidget->addWidget(cockpit);
    stackedWidget->addWidget(controlsHelpPage);
    stackedWidget->addWidget(coordinateHelpPage);
    stackedWidget->addWidget(settingsPage);

    vLayout->addWidget(topBar);
    vLayout->addWidget(stackedWidget, 1);

    setCentralWidget(centralWidget);
}

void MainWindow::setupThread()
{
    simulationThread = new QThread(this);
    simulationWorker = new SimulationWorker();

    simulationWorker->moveToThread(simulationThread);

    connect(simulationThread,
            &QThread::finished,
            simulationWorker,
            &QObject::deleteLater);

    simulationThread->start();
}

void MainWindow::setupConnections()
{
    // =====================================================
    // TopBar Navigation
    // =====================================================

    connect(topBar,
            &TopBarWidget::homeRequested,
            this,
            [this]()
            {
                stackedWidget->setCurrentWidget(homepage);
            });

    connect(topBar,
            &TopBarWidget::selectSpacecraftRequested,
            this,
            [this]()
            {
                stackedWidget->setCurrentWidget(selectionPage);
            });

    connect(topBar,
            &TopBarWidget::startSimulationRequested,
            this,
            [this]()
            {
                sendCurrentConfigToWorker();
                stackedWidget->setCurrentWidget(cockpit);
            });

    connect(topBar,
            &TopBarWidget::exportTelemetryRequested,
            this,
            [this]()
            {
                QString filePath = QFileDialog::getSaveFileName(
                    this,
                    "Export Telemetry as XML",
                    "telemetry.xml",
                    "XML files (*.xml)");

                if (filePath.isEmpty())
                {
                    return;
                }

                if (!filePath.endsWith(".xml", Qt::CaseInsensitive))
                {
                    filePath += ".xml";
                }

                QMetaObject::invokeMethod(simulationWorker,
                                          "exportTelemetryToXml",
                                          Qt::QueuedConnection,
                                          Q_ARG(QString, filePath));
            });

    connect(topBar,
            &TopBarWidget::controlsHelpRequested,
            this,
            [this]()
            {
                stackedWidget->setCurrentWidget(controlsHelpPage);
            });

    connect(topBar,
            &TopBarWidget::coordinateHelpRequested,
            this,
            [this]()
            {
                stackedWidget->setCurrentWidget(coordinateHelpPage);
            });

    connect(topBar,
            &TopBarWidget::settingsRequested,
            this,
            [this]()
            {
                stackedWidget->setCurrentWidget(settingsPage);
            });

    // =====================================================
    // Homepage Navigation
    // =====================================================

    connect(homepage,
            &Homepage::selectSpacecraftRequested,
            this,
            [this]()
            {
                stackedWidget->setCurrentWidget(selectionPage);
            });

    connect(homepage,
            &Homepage::startSimulationRequested,
            this,
            [this]()
            {
                sendCurrentConfigToWorker();
                stackedWidget->setCurrentWidget(cockpit);
            });

    // =====================================================
    // Spacecraft Selection
    // =====================================================

    connect(selectionPage,
            &SpacecraftSelectionPage::backRequested,
            this,
            [this]()
            {
                stackedWidget->setCurrentWidget(homepage);
            });

    connect(selectionPage,
            &SpacecraftSelectionPage::spacecraftSelected,
            this,
            [this](const QString& spacecraft)
            {
                selectedJsonConfigStr = spacecraft;
                sendCurrentConfigToWorker();
                stackedWidget->setCurrentWidget(cockpit);
            });

    // =====================================================
    // Cockpit → SimulationWorker
    // =====================================================

    connect(cockpit,
            &cockpitPage::flightCmdRequested,
            simulationWorker,
            &SimulationWorker::setFlightCommand);

    connect(cockpit,
            &cockpitPage::autopilotToggled,
            simulationWorker,
            &SimulationWorker::setAutopilotFlag);

    connect(cockpit,
            &cockpitPage::startRequested,
            simulationWorker,
            &SimulationWorker::start);

    connect(cockpit,
            &cockpitPage::pauseRequested,
            simulationWorker,
            &SimulationWorker::pause);

    connect(cockpit,
            &cockpitPage::pauseRequested,
            this,
            [this]()
            {
                topBar->setExportEnabled(true);
            });

    connect(cockpit,
            &cockpitPage::stopConfirmed,
            simulationWorker,
            &SimulationWorker::stop);

    connect(cockpit,
            &cockpitPage::stopConfirmed,
            this,
            [this]()
            {
                topBar->setExportEnabled(true);
            });

    // =====================================================
    // MainWindow → SimulationWorker
    // =====================================================

    connect(this,
            &QObject::destroyed,
            simulationThread,
            &QThread::quit);

    // =====================================================
    // SimulationWorker → Frontend
    // =====================================================

    connect(simulationWorker,
            &SimulationWorker::stateUpdated,
            cockpit,
            &cockpitPage::onStateUpdated);

    connect(simulationWorker,
            &SimulationWorker::simulationStarted,
            this,
            [this]()
            {
                topBar->setExportEnabled(false);
            });

    connect(simulationWorker,
            &SimulationWorker::telemetryExportSucceeded,
            this,
            [this](const QString &filePath)
            {
                QMessageBox::information(
                    this,
                    "Telemetry Export",
                    QString("Telemetry data was exported successfully to:\n%1").arg(filePath));
            });

    connect(simulationWorker,
            &SimulationWorker::telemetryExportFailed,
            this,
            [this](const QString &errorMessage)
            {
                QMessageBox::critical(
                    this,
                    "Telemetry Export Failed",
                    errorMessage);
            });

    connect(simulationWorker,
            &SimulationWorker::historyOverwriteConfirmationRequested,
            this,
            [this]()
            {
                const QMessageBox::StandardButton answer = QMessageBox::warning(
                    this,
                    "Start New Simulation",
                    "Telemetry data from the previous simulation is still buffered for export.\n\n"
                    "If you continue, this buffered data will be deleted and cannot be exported afterwards.\n\n"
                    "Do you want to start a new simulation anyway?",
                    QMessageBox::Yes | QMessageBox::No,
                    QMessageBox::No);

                if (answer == QMessageBox::Yes)
                {
                    QMetaObject::invokeMethod(simulationWorker,
                                              "confirmStartWithHistoryReset",
                                              Qt::QueuedConnection);
                }
            });
}

void MainWindow::sendCurrentConfigToWorker()
{
    QString jsonConfig;

    if (selectedJsonConfigStr.isEmpty())
    {
        jsonConfig = configManager_.defaultSpacecraftJson();
    }
    else
    {
        jsonConfig = selectedJsonConfigStr;
    }

    if (jsonConfig.isEmpty())
    {
        qWarning() << "[MainWindow] No spacecraft JSON config available.";
        return;
    }

    QMetaObject::invokeMethod(simulationWorker,
                              "receiveJsonConfig",
                              Qt::QueuedConnection,
                              Q_ARG(QString, jsonConfig));
}

void MainWindow::showEvent(QShowEvent *event)
{
    QMainWindow::showEvent(event);

    if (!windowHandle())
    {
        return;
    }

    QScreen *screen = windowHandle()->screen();

    if (!screen)
    {
        return;
    }

    QRect screenGeometry = screen->availableGeometry();
    resize(screenGeometry.width() * 0.7,
           screenGeometry.height() * 0.7);
}
