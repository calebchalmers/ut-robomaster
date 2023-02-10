#include "turret_subsystem.hpp"

namespace subsystems
{
namespace turret
{
TurretSubsystem::TurretSubsystem(tap::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      yawMotor(drivers, GM6020, MOTOR6, CAN_BUS_MOTORS, false, "yaw motor", PID_KP, PID_KI, PID_KD),
      pitchMotor(
          drivers,
          M3508,
          MOTOR7,
          CAN_BUS_MOTORS,
          false,
          "pitch motor",
          PID_KP,
          PID_KI,
          PID_KD)
{
}

void TurretSubsystem::initialize()
{
    yawMotor.initialize();
    pitchMotor.initialize();
}

void TurretSubsystem::setDesiredRpm(float yaw, float pitch)
{
    desiredRpmYaw = yaw;
    desiredRpmPitch = pitch;
}

void TurretSubsystem::refresh()
{
    yawMotor.update(desiredRpmYaw / 60.0f);
    pitchMotor.update(desiredRpmPitch / 60.0f);
}

void TurretSubsystem::runHardwareTests()
{
    // TODO
}

}  // namespace turret
}  // namespace subsystems