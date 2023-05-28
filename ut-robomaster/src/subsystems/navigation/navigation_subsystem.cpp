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

void NavigationSubsystem::initialize() {}
void NavigationSubsystem::refresh() { chassis->input(Vector2f(0.0f), 0.1f); }
}  // namespace navigation
}  // namespace subsystems