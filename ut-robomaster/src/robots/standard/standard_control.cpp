#ifdef TARGET_STANDARD

#include "tap/control/command_mapper.hpp"

#include "drivers.hpp"
#include "drivers_singleton.hpp"

// Turret includes ------------------------------------------
#include "subsystems/turret/command_move_turret_joystick.hpp"
#include "subsystems/turret/turret_subsystem.hpp"

using namespace tap::control;
using namespace tap::communication::serial;

using namespace subsystems::turret;

using namespace commands;

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
TurretSubsystem turret(drivers());

// Command definitions -----------------------------------------------------------
CommandMoveTurretJoystick moveTurretCommandJoystick(drivers(), &turret);

// Register subsystems here -----------------------------------------------
void registerStandardSubsystems(src::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&turret);
}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() { turret.initialize(); }

// Set default commands here -----------------------------------------------
void setDefaultCommands(src::Drivers *) { turret.setDefaultCommand(&moveTurretCommandJoystick); }

void runStartupCommands(src::Drivers *) {}

// Register IO mappings here -----------------------------------------------
void registerMappings(src::Drivers *drivers) {}
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