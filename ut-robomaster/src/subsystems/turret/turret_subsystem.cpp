#include "turret_subsystem.hpp"

#include "tap/algorithms/ballistics.hpp"

#include "modm/math.hpp"
#include "robots/robot_constants.hpp"

namespace subsystems
{
namespace turret
{
using namespace tap::algorithms::ballistics;
using communication::TurretData;
using modm::Vector2f;

TurretSubsystem::TurretSubsystem(src::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      yawMotor(drivers, ID_YAW, CAN_TURRET, false, "yaw"),
      pitchMotor(drivers, ID_PITCH, CAN_TURRET, false, "pitch"),
      yawTurret(drivers, &yawMotor, YAW_PID_CONFIG),
      pitchTurret(drivers, &pitchMotor, PITCH_PID_CONFIG),
      turretOffset(0.0f, 0.0f, M_TWOPI)
{
}

void TurretSubsystem::initialize()
{
    yawTurret.initialize();
    pitchTurret.initialize();
    lastTime = tap::arch::clock::getTimeMilliseconds();
}

void TurretSubsystem::inputManualAngles(float yaw, float pitch)
{
    inputYaw = yaw;
    inputPitch = pitch;
}

void TurretSubsystem::inputTargetData(Vector3f position, Vector3f velocity, Vector3f acceleration)
{
    targetPosition = position;
    targetVelocity = velocity;
    targetAcceleration = acceleration;
}

void TurretSubsystem::setTargetWorldAngles(float yaw, float pitch)
{
    targetWorldYaw = yaw;
    targetWorldPitch = modm::min(modm::max(pitch, PITCH_MIN), PITCH_MAX);
}

void TurretSubsystem::setAimStrategy(AimStrategy aimStrategy) { this->aimStrategy = aimStrategy; }

float TurretSubsystem::getChassisYaw() { return modm::toRadian(drivers->bmi088.getYaw() - 180.0f); }
float TurretSubsystem::getTargetLocalYaw() { return targetWorldYaw - getChassisYaw(); }
float TurretSubsystem::getTargetLocalPitch() { return targetWorldPitch; }

float TurretSubsystem::getCurrentLocalYaw()
{
    return !isCalibrated ? 0.0f : yawTurret.getAngle() / BELT_RATIO - baseYaw;
}

float TurretSubsystem::getCurrentLocalPitch()
{
    return !isCalibrated ? 0.0f : pitchTurret.getAngle() - basePitch;
}

void TurretSubsystem::refresh()
{
    float bulletVelocity = 1.0f;
    float offset = 0.0f;
    uint8_t numBallisticIterations = 1;

    switch (aimStrategy)
    {
        case AimStrategy::Manual:
            setTargetWorldAngles(inputYaw, inputPitch);
            break;
        case AimStrategy::AutoAim:
        {
            updateAutoAim();
            break;
        }
        case AimStrategy::AimAssist:  // unimplemented
            break;
    }

    yawTurret.updateMotorAngle();
    pitchTurret.updateMotorAngle();

    if (!isCalibrated && yawMotor.isMotorOnline() && pitchMotor.isMotorOnline())
    {
        baseYaw = yawTurret.getAngle() / BELT_RATIO;
        basePitch = pitchTurret.getAngle() - PITCH_MIN;
        isCalibrated = true;
    }

    if (isCalibrated && !drivers->isKillSwitched())
    {
        uint32_t time = tap::arch::clock::getTimeMilliseconds();
        uint32_t dt = time - lastTime;
        lastTime = time;

        yawTurret.setAngle((baseYaw + getTargetLocalYaw()) * BELT_RATIO, dt);
        pitchTurret.setAngle(basePitch + getTargetLocalPitch(), dt);
    }
    else
    {
        yawTurret.reset();
        pitchTurret.reset();
    }
}

void TurretSubsystem::runHardwareTests()
{
    // TODO
}

void TurretSubsystem::updateAutoAim()
{
    // float turretPitch = 0.0f;
    // float turretYaw = 0.0f;
    // float projectedTravelTime = 0.0f;

    // findTargetProjectileIntersection(
    //     {targetPosition, targetVelocity, targetAcceleration},
    //     bulletVelocity,
    //     numBallisticIterations,
    //     &turretPitch,
    //     &turretYaw,
    //     &projectedTravelTime);

    // targetWorldYaw = turretYaw;
    // targetWorldPitch = turretPitch;

    // ^ Ignoring ballistics for now

    if (!drivers->beaglebone.isOnline()) return;
    if (lastTurretDataIndex == drivers->beaglebone.turretDataIndex) return;
    lastTurretDataIndex = drivers->beaglebone.turretDataIndex;

    TurretData turretData = drivers->beaglebone.getTurretData();
    if (!turretData.hasTarget) return;

    float deltaYaw = -atan(turretData.xPos / turretData.zPos);  // yaw is opposite to camera X
    float deltaPitch = atan(turretData.yPos / turretData.zPos);
    float scale = 0.5f;
    setTargetWorldAngles(targetWorldYaw + deltaYaw * scale, targetWorldPitch + deltaPitch * scale);
}
}  // namespace turret
}  // namespace subsystems