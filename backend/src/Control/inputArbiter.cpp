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

    if (killRotationActive || stablizeRotationActive) cmd.rotation = autoCmd_.rotation;

    return cmd;
}

void InputArbiter::receiveUserControlCommand(const ControlCommand &userCmd)
{
    usrCmd_ = userCmd;

    automationActive        = usrCmd_.autopilotActive;
    killRotationActive      = usrCmd_.killRotation;
    stablizeRotationActive  = usrCmd_.stabilize;
}
void InputArbiter::receiveAutoControlCommand(const ControlCommand &autoCmd)
{
    autoCmd_ = autoCmd;
}

void InputArbiter::setAutomationActiveFlag(bool on)
{
    automationActive = on;
}
