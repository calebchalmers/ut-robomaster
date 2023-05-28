#ifndef SUBSYSTEMS_NAVIGATION_SUBSYSTEM_HPP_
#define SUBSYSTEMS_NAVIGATION_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

#include "modm/math/geometry.hpp"
#include "subsystems/chassis/chassis_subsystem.hpp"
#include "subsystems/odometry/odometry_subsystem.hpp"

#include "drivers.hpp"

namespace subsystems
{
namespace navigation
{

using chassis::ChassisSubsystem;
using odometry::OdometrySubsystem;

class NavigationSubsystem : public tap::control::Subsystem
{
public:
    NavigationSubsystem(
        src::Drivers* drivers,
        ChassisSubsystem* chassis,
        OdometrySubsystem* odometry);
    void initialize() override;
    void refresh() override;
    const char* getName() override { return "Navigation subsystem"; }

private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;
    OdometrySubsystem* odometry;
    Vector2f target = Vector2f(0.0f);
};
}  // namespace navigation
}  // namespace subsystems

#endif