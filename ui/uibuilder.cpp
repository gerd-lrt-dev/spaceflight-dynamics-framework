#include "uibuilder.h"

UIBuilder::UIBuilder() {}

const QString UIBuilder::STDButtonLayout::styleSheet =
    "QPushButton#STDButton { "
        "background-color: #1B2A44; "
        "color: white; font-size: "
        "20px; font-weight: bold; "
        "padding: 12px 24px; "
        "border-radius: 12px; "
        "border: 2px solid #2F4A72; "
        "letter-spacing: 1px; }"
    "QPushButton#STDButton:hover { "
        "background-color: #24385A; "
        "border-color: #3B5C8A; }"
    "QPushButton#STDButton:pressed { "
        "background-color: #162133; "
        "border-color: #203252; }";

QPushButton* UIBuilder::createButton(const QString &buttonName, QWidget *parent)
{
    QPushButton *button = new QPushButton(buttonName, parent);
    button->setObjectName("STDButton");
    button->setFixedSize(STDButtonLayout::buttonwidth, STDButtonLayout::buttonHeight);

    button->setStyleSheet(STDButtonLayout::styleSheet);

    return button;
}

QLabel* UIBuilder::createCopyright(const QString &copyRightStr, QWidget *parent)
{
    QLabel *copyright = new QLabel(copyRightStr, parent);
    copyright->setAlignment(Qt::AlignCenter);
    copyright->setStyleSheet("color: #AFC2D9; font-size: 14px; font-weight: 500; letter-spacing: 1px;");

    return copyright;
}

QLabel* UIBuilder::createPageTitle(const QString &titleStr, QWidget *parent)
{
    QLabel *title = new QLabel(titleStr, parent);
    title->setAlignment(Qt::AlignCenter);
    title->setStyleSheet("font-size: 32px;"
                         "font-weight: bold;");

    return title;
}

QWidget *UIBuilder::setupDetailBox(const QVector<QLCDNumber*> &lcdPanels, const QVector<QString> &namesOfPanels, const QString &nameOfTitle, int columns)
{
    if (lcdPanels.size() != namesOfPanels.size())
    {
        qDebug() << "[UIBuilder] Size of Panel and Name Vector is not identical!";
        return nullptr;
    }

    if (columns <= 0)
    {
        qDebug() << "[UIBuilder] Invalid column count!";
        return nullptr;
    }

    QFrame *box = new QFrame();
    box->setFrameShape(QFrame::StyledPanel);
    box->setStyleSheet("background-color: #0E1624");
    box->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    QVBoxLayout *mainLayout = new QVBoxLayout(box);
    mainLayout->setContentsMargins(6, 6, 6, 6);
    mainLayout->setSpacing(4);

    QLabel *title = new QLabel(nameOfTitle);
    title->setAlignment(Qt::AlignCenter);
    title->setStyleSheet("color: #4FC3F7; font-weight: bold;");

    QGridLayout *grid = new QGridLayout();

    for (int i = 0; i < lcdPanels.size(); ++i)
    {
        QWidget *lcdField = createLcdField(namesOfPanels[i], lcdPanels[i], 8);

        int row = i / columns;
        int col = i % columns;

        grid->addWidget(lcdField, row, col);
    }

    mainLayout->addWidget(title);
    mainLayout->addLayout(grid);

    return box;
}

//*******************************
// Private
//*******************************
static void configureLCD(QLCDNumber* lcd, int digits)
{
    lcd->setDigitCount(digits);
    lcd->setSegmentStyle(QLCDNumber::Flat);
    lcd->setSmallDecimalPoint(false);
}

QWidget* UIBuilder::createLcdField(const QString& title, QLCDNumber* lcd, int digits)
{
    QWidget *field = new QWidget();
    QVBoxLayout *layout = new QVBoxLayout(field);

    layout->setContentsMargins(2, 2, 2, 2);
    layout->setSpacing(2);

    QLabel *label = new QLabel(title);
    label->setAlignment(Qt::AlignCenter);
    label->setStyleSheet("color: #AFC7DF; font-size: 10px;");

    configureLCD(lcd, digits);

    layout->addWidget(label);
    layout->addWidget(lcd);

    return field;
}
