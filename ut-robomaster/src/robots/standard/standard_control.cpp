#ifndef ENV_UNIT_TESTS

#ifdef TARGET_STANDARD

#include "drivers.hpp"
#include "drivers_singleton.hpp"

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/drivers.hpp"

#include "subsystems/turret/turret_subsystem.hpp"
#include "subsystems/turret/turret_command.hpp"

#include "tap/communication/gpio/leds.hpp"
using namespace tap::control;
using namespace tap::communication::serial;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
src::driversFunc drivers = src::DoNotUse_getDrivers;
tap::gpio::Leds led;

namespace standard_control
{
/* define subsystems --------------------------------------------------------*/

TurretSubsystem theTurret(drivers());  // mouse  

/* define commands ----------------------------------------------------------*/

TurretCommand turretCommand(drivers(), &theTurret);    //mouse 


/* define command mappings --------------------------------------------------*/

/* register subsystems here -------------------------------------------------*/
void registerStandardSubsystems(tap::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&theTurret);    // mouse
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems() { 
    theTurret.initialize();     // mouse
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultStandardCommands(tap::Drivers *) 
{
    theTurret.setDefaultCommand(&turretCommand);    //mouse
}

/* add any starting commands to the scheduler here --------------------------*/
void startStandardCommands(tap::Drivers *) {}

/* register io mappings here ------------------------------------------------*/
void registerStandardIoMappings(tap::Drivers *drivers)
{
}
} // namespace standard_control

namespace control
{
    void initSubsystemCommands(tap::Drivers *drivers)
    {
        standard_control::initializeSubsystems();
        standard_control::registerStandardSubsystems(drivers);
        standard_control::setDefaultStandardCommands(drivers);
        standard_control::startStandardCommands(drivers);
        standard_control::registerStandardIoMappings(drivers);
    }
}

#endif
#endif