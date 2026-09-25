#include "Control/inputArbiter.h"
#include <iostream>

ControlCommand InputArbiter::chooseCommand()
{
    ControlCommand cmd;

    if (!automationActive)
    {
        cmd = usrCmd_;
    }
    else
    {
        cmd.mainEngine          = autoCmd_.mainEngine;
        cmd.thrustInPercentage  = autoCmd_.thrustInPercentage;

        cmd.translation         = usrCmd_.translation;
        cmd.rotation            = usrCmd_.rotation;
        cmd.autopilotActive     = usrCmd_.autopilotActive;
        cmd.killRotation        = usrCmd_.killRotation;
    }

    if (killRotationActive || stabilizeRotationActive) cmd.rotation = autoCmd_.rotation;

    return cmd;
}

void InputArbiter::receiveUserControlCommand(const ControlCommand &userCmd)
{
    usrCmd_ = userCmd;

    automationActive        = usrCmd_.autopilotActive;
    killRotationActive      = usrCmd_.killRotation;
    stabilizeRotationActive  = usrCmd_.stabilize;
}
void InputArbiter::receiveAutoControlCommand(const ControlCommand &autoCmd)
{
    autoCmd_ = autoCmd;
}

bool InputArbiter::isKillRotationActive() const
{
    return killRotationActive;
}

bool InputArbiter::isStabilizeActive() const
{
    return stabilizeRotationActive;
}

void InputArbiter::setAutomationActiveFlag(bool on)
{
    automationActive = on;
}
