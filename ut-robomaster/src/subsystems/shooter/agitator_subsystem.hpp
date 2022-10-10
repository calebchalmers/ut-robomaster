#ifndef SUBSYSTEMS_SHOOTER_AGITATOR_SUBSYSTEM_HPP_
#define SUBSYSTEMS_SHOOTER_AGITATOR_SUBSYSTEM_HPP_

#include "modm/math/filter/pid.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

namespace subsystems
{
namespace shooter
{

// Agitator uses DJI motors
class AgitatorSubsystem : public tap::control::Subsystem
{
public:
    AgitatorSubsystem(tap::Drivers *drivers, tap::motor::MotorId motorId, tap::can::CanBus canBusMotors);
    ~AgitatorSubsystem() = default;

    void initialize() override;

    void setMotorOutput(int desiredRPM);

    // Default refresh for Subsystem is no-op
    // void refresh() override;

private:
    const tap::motor::MotorId motorId;
    const tap::can::CanBus canBusMotors;
    uint16_t motorOutput; // Unused?
    modm::Pid<float> pidController; // Should the PID controller use int instead? Check kp
    tap::motor::DjiMotor motor;
    static constexpr float MAX_CURRENT_OUTPUT = 8000.0f;
};

}
}

#endif // SUBSYSTEMS_SHOOTER_AGITATOR_SUBSYSTEM_HPP_
