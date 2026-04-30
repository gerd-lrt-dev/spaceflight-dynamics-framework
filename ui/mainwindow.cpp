#include "mainwindow.h"
#include "uibuilder.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setupUI();
}

MainWindow::~MainWindow()
{

}

void MainWindow::setupUI()
{
    // Build central Widget
    QWidget *centralWidget = new QWidget(this);

    // Build vertical layout box
    vLayout = new QVBoxLayout(centralWidget);

    // Build stackedWidget
    QStackedWidget *stackedWidget = new QStackedWidget(this);

    //Create pages
    homepage = new Homepage(this);

    // Add pages to stackedWidget
    stackedWidget->addWidget(homepage);

    // Add StackedWidget to layout
    vLayout->addWidget(stackedWidget);

    // Contribute centralWidget to MainWIndow
    setCentralWidget(centralWidget);
}

void MainWindow::showEvent(QShowEvent *event)
{
    QMainWindow::showEvent(event);

    if (!windowHandle())
    {
        return;
    }

    QScreen *screen = windowHandle()->screen();

    if(!screen)
    {
        return;
    }

    QRect screenGeometry = screen->availableGeometry();
    resize(screenGeometry.width() * 0.7, screenGeometry.height() * 0.7);
}
