#include "inputmapper.h"

#include <QKeyEvent>
#include <QString>

inputmapper::inputmapper(QSlider* mainEngineSlider, QLabel* label, QObject* parent)
    : QObject(parent),
    mE_Slider(mainEngineSlider),
    m_label(label)
{
    Q_ASSERT(mE_Slider != nullptr);
}

void inputmapper::handleKeyPress(QKeyEvent* event)
{
    if (event == nullptr || mE_Slider == nullptr)
    {
        return;
    }

    if (event->isAutoRepeat())
    {
        return;
    }

    switch (event->key())
    {
    case Qt::Key_Up:
        mE_Slider->setValue(mE_Slider->value() + 1);
        return;

    case Qt::Key_Down:
        mE_Slider->setValue(mE_Slider->value() - 1);
        return;

    case Qt::Key_D:
        ENU_RCS_PosX = true;
        break;

    case Qt::Key_A:
        ENU_RCS_NegX = true;
        break;

    case Qt::Key_W:
        ENU_RCS_PosY = true;
        break;

    case Qt::Key_S:
        ENU_RCS_NegY = true;
        break;

    case Qt::Key_E:
        ENU_RCS_PosZ = true;
        break;

    case Qt::Key_Q:
        ENU_RCS_NegZ = true;
        break;

    default:
        break;
    }
    updateFlightCommand();
}

void inputmapper::handleKeyRelease(QKeyEvent* event)
{
    if (event == nullptr)
    {
        return;
    }

    if (event->isAutoRepeat())
    {
        return;
    }

    switch (event->key())
    {
    case Qt::Key_D:
        ENU_RCS_PosX = false;
        break;

    case Qt::Key_A:
        ENU_RCS_NegX = false;
        break;

    case Qt::Key_W:
        ENU_RCS_PosY = false;
        break;

    case Qt::Key_S:
        ENU_RCS_NegY = false;
        break;

    case Qt::Key_E:
        ENU_RCS_PosZ = false;
        break;

    case Qt::Key_Q:
        ENU_RCS_NegZ = false;
        break;

    default:
        break;
    }
    updateFlightCommand();
}

void inputmapper::updateLabel(int value)
{
    if (m_label != nullptr)
    {
        m_label->setText(QString("Commanded Thrust: %1 %").arg(value));
    }
}

void inputmapper::updateFlightCommand()
{
    FlightCommand cmd{0.0, 0.0, 0.0};

    if (ENU_RCS_PosX == true && ENU_RCS_NegX == false)
    {
        cmd.translation.x = 1.0;
    }
    else if(ENU_RCS_NegX == true && ENU_RCS_PosX == false)
    {
        cmd.translation.x = -1.0;
    }
    else
    {
        cmd.translation.x = 0.0;
    }

    if (ENU_RCS_PosY == true && ENU_RCS_NegY == false)
    {
        cmd.translation.y = 1.0;
    }
    else if(ENU_RCS_NegY == true && ENU_RCS_PosY == false)
    {
        cmd.translation.y = -1.0;
    }
    else
    {
        cmd.translation.y = 0.0;
    }

    if (ENU_RCS_PosZ == true && ENU_RCS_NegZ == false)
    {
        cmd.translation.z = 1.0;
    }
    else if(ENU_RCS_NegZ == true && ENU_RCS_PosZ == false)
    {
        cmd.translation.z = -1.0;
    }
    else
    {
        cmd.translation.z = 0.0;
    }

    emit RCS_cmdRequested(cmd);
}
