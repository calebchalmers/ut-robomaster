#include "command_move_turret_joystick.hpp"

namespace commands
{
void CommandMoveTurretJoystick::initialize() {}

void CommandMoveTurretJoystick::execute()
{
    Remote* remote = &drivers->remote;  // interface for the robot's controller

    float h = remote->getChannel(Remote::Channel::LEFT_HORIZONTAL);  // joystick horizontal [-1, 1]
    float v = remote->getChannel(Remote::Channel::LEFT_VERTICAL);    // joystick vertical [-1, 1]

    turret->inputManualAngles(0, 0);  // angles here are in radians and are absolute
}

void CommandMoveTurretJoystick::end(bool) {}

bool CommandMoveTurretJoystick::isFinished(void) const { return false; }
}  // namespace commands
