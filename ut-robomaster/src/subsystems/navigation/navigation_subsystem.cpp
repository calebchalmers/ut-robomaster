#include "navigation_subsystem.hpp"

namespace subsystems
{
namespace navigation
{
NavigationSubsystem::NavigationSubsystem(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    OdometrySubsystem* odometry)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      chassis(chassis),
      odometry(odometry)
{
}

void NavigationSubsystem::initialize() { target = Vector2f(0.0f, 0.1f); }
void NavigationSubsystem::refresh()
{
    Vector2f delta = target - odometry->getPosition();
    delta /= max(1.0f, delta.getLength());
    chassis->input(delta * 0.05f, 0.0f);
}
}  // namespace navigation
}  // namespace subsystems