#pragma once

#include "tap/control/command.hpp"

#include "subsystems/agitator/agitator_subsystem.hpp"

#include "drivers.hpp"

namespace commands
{
using subsystems::agitator::AgitatorSubsystem;

class CommandUnjam : public tap::control::Command
{
public:
    CommandUnjam(src::Drivers *drivers, AgitatorSubsystem *agitator)
        : drivers(drivers),
          agitator(agitator)
    {
        addSubsystemRequirement(agitator);
    }

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;
    const char *getName() const override { return "agitator unjam command"; }

private:
    src::Drivers *drivers;
    AgitatorSubsystem *agitator;

};  // class CommandFireContinuous
}  // namespace commands