#ifdef TARGET_STANDARD

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "robots/robot_constants.hpp"
#include "utils/power_limiter/barrel_cooldown.hpp"

#include "drivers.hpp"
#include "drivers_singleton.hpp"

// Chassis includes ----------------------------------------
#include "subsystems/chassis/chassis_subsystem.hpp"
#include "subsystems/chassis/command_beyblade_chassis_keyboard.hpp"
#include "subsystems/chassis/command_move_chassis_joystick.hpp"
#include "subsystems/chassis/command_move_chassis_keyboard.hpp"
#include "subsystems/chassis/command_move_chassis_turret_relative_joystick.hpp"

// Agitator includes ----------------------------------------
#include "subsystems/agitator/agitator_subsystem.hpp"
#include "subsystems/agitator/command_agitator_continuous.hpp"
#include "subsystems/agitator/command_unjam_agitator.hpp"

// Flywheel includes ----------------------------------------
#include "subsystems/flywheel/command_flywheel_off.hpp"
#include "subsystems/flywheel/command_rotate_flywheel.hpp"
#include "subsystems/flywheel/flywheel_subsystem.hpp"

// Turret includes ------------------------------------------
#include "subsystems/odometry/odometry_subsystem.hpp"
#include "subsystems/turret/command_move_turret_aimbot.hpp"
#include "subsystems/turret/command_move_turret_joystick.hpp"
#include "subsystems/turret/command_move_turret_mouse.hpp"
#include "subsystems/turret/turret_subsystem.hpp"

using namespace tap::control;
using namespace tap::communication::serial;

using namespace subsystems::chassis;
using namespace subsystems::agitator;
using namespace subsystems::flywheel;
using namespace subsystems::turret;
using namespace subsystems::odometry;

using namespace commands;

using power_limiter::BarrelId;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
src::driversFunc drivers = src::DoNotUse_getDrivers;

namespace standard_control
{

// Subsystem definitions ---------------------------------------------------------
ChassisSubsystem chassis(drivers());
AgitatorSubsystem agitator1(drivers(), ID_AGITATOR_L, false);
AgitatorSubsystem agitator2(drivers(), ID_AGITATOR_R, true);
FlywheelSubsystem flywheel(drivers());
TurretSubsystem turret(drivers());
OdometrySubsystem odometry(drivers(), &chassis, &turret);

// Command definitions -----------------------------------------------------------
CommandMoveChassisJoystick moveChassisCommandJoystick(drivers(), &chassis, &turret);
CommandMoveChassisTurretRelativeJoystick moveChassisTurretRelativeCommandJoystick(
    drivers(),
    &chassis,
    &turret);
CommandMoveChassisKeyboard moveChassisCommandKeyboard(drivers(), &chassis, &turret);
CommandBeybladeChassisKeyboard beybladeChassisCommandKeyboard(drivers(), &chassis, &turret);

CommandAgitatorContinuous agitator1ContinuousCommand(drivers(), &agitator1, BarrelId::STANDARD1);
CommandAgitatorContinuous agitator2ContinuousCommand(drivers(), &agitator2, BarrelId::STANDARD2);
CommandUnjamAgitator unjamAgitator1Command(drivers(), &agitator1);
CommandUnjamAgitator unjamAgitator2Command(drivers(), &agitator2);

CommandRotateFlywheel rotateFlywheelKeyboardCommand(drivers(), &flywheel);
CommandRotateFlywheel rotateFlywheelNoAgitatorCommand(drivers(), &flywheel);
CommandRotateFlywheel rotateFlywheelWithAgitatorCommand(drivers(), &flywheel);
CommandFlywheelOff flywheelOffCommand(drivers(), &flywheel);

CommandMoveTurretJoystick moveTurretCommandJoystick(drivers(), &turret);
CommandMoveTurretJoystick moveTurretWhenChassisIsTurretRelativeCommandJoystick(drivers(), &turret);
CommandMoveTurretMouse moveTurretCommandMouse(drivers(), &turret);
CommandMoveTurretAimbot moveTurretCommandAimbot(drivers(), &turret);

// Keyboard mappings ------------------------------------------------------------
ToggleCommandMapping keyRToggled(
    drivers(),
    {&beybladeChassisCommandKeyboard},
    RemoteMapState({Remote::Key::R}));

ToggleCommandMapping keyGToggled(
    drivers(),
    {&rotateFlywheelKeyboardCommand},
    RemoteMapState({Remote::Key::G}));

HoldCommandMapping leftMouseDown(
    drivers(),
    {&agitator1ContinuousCommand, &agitator2ContinuousCommand},
    RemoteMapState(RemoteMapState::MouseButton::LEFT));

HoldCommandMapping keyXHeld(
    drivers(),
    {&unjamAgitator1Command, &unjamAgitator2Command},
    RemoteMapState({Remote::Key::X}));

HoldCommandMapping rightMouseDown(
    drivers(),
    {&moveTurretCommandAimbot},
    RemoteMapState(RemoteMapState::MouseButton::RIGHT));

// Joystick mappings ------------------------------------------------------------
HoldCommandMapping rightSwitchUp(
    drivers(),
    {&moveChassisTurretRelativeCommandJoystick,
     &moveTurretWhenChassisIsTurretRelativeCommandJoystick},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));

HoldCommandMapping rightSwitchMid(
    drivers(),
    {&moveChassisCommandJoystick, &moveTurretCommandJoystick},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));

HoldCommandMapping rightSwitchDown(
    drivers(),
    {&flywheelOffCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));

HoldCommandMapping leftSwitchMid(
    drivers(),
    {&rotateFlywheelNoAgitatorCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

HoldCommandMapping leftSwitchUp(
    drivers(),
    {&agitator1ContinuousCommand, &agitator2ContinuousCommand, &rotateFlywheelWithAgitatorCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// Register subsystems here -----------------------------------------------
void registerStandardSubsystems(src::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&agitator1);
    drivers->commandScheduler.registerSubsystem(&agitator2);
    drivers->commandScheduler.registerSubsystem(&flywheel);
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&odometry);
}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems()
{
    chassis.initialize();
    agitator1.initialize();
    agitator2.initialize();
    flywheel.initialize();
    turret.initialize();
    odometry.initialize();
}

// Set default commands here -----------------------------------------------
void setDefaultCommands(src::Drivers *)
{
    chassis.setDefaultCommand(&moveChassisCommandKeyboard);
    flywheel.setDefaultCommand(&flywheelOffCommand);
    turret.setDefaultCommand(&moveTurretCommandMouse);
}

void runStartupCommands(src::Drivers *) {}

// Register IO mappings here -----------------------------------------------
void registerMappings(src::Drivers *drivers)
{
    // Keyboard mappings ------------------------------------------------------------
    drivers->commandMapper.addMap(&keyRToggled);
    drivers->commandMapper.addMap(&leftMouseDown);
    drivers->commandMapper.addMap(&keyXHeld);
    drivers->commandMapper.addMap(&rightMouseDown);
    drivers->commandMapper.addMap(&keyGToggled);

    // Joystick mappings ------------------------------------------------------------
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&rightSwitchMid);
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchMid);
    drivers->commandMapper.addMap(&leftSwitchUp);
}
}  // namespace standard_control

namespace control
{
void initSubsystemCommands(src::Drivers *drivers)
{
    standard_control::initializeSubsystems();
    standard_control::registerStandardSubsystems(drivers);
    standard_control::setDefaultCommands(drivers);
    standard_control::runStartupCommands(drivers);
    standard_control::registerMappings(drivers);
}
}  // namespace control
#endif