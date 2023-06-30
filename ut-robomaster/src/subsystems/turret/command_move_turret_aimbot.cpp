#include "command_move_turret_aimbot.hpp"

#include "tap/algorithms/ballistics.hpp"

using namespace tap::algorithms::ballistics;
using communication::TurretData;

namespace commands
{
void CommandMoveTurretAimbot::initialize() {
    yaw = turret->getInputYaw();
    pitch = turret->getInputPitch();
}

void CommandMoveTurretAimbot::execute() {
    TurretData cv = drivers->beagleboneCommunicator.getTurretData();
    if (cv.hasTarget) {
        Vector3f targetPosition = Vector3f(
            cv.xPos + CAMERA_X_OFFSET,
            cv.zPos + CAMERA_TO_PITCH_OFFSET,
            cv.yPos + CAMERA_TO_BARREL_OFFSET);
        Vector3f targetVelocity = Vector3f(cv.xVel, cv.zVel, cv.yVel);
        Vector3f targetAcceleration = Vector3f(cv.xAcc, cv.zAcc, cv.yAcc);
        
        float bulletVelocity = DEFAULT_EXIT_VELOCITY;

        // Get bullet velocity if ref system is connected
        if (drivers->refSerial.getRefSerialReceivingData()) {
            #if defined(TARGET_STANDARD) || defined(TARGET_SENTRY)
                bulletVelocity = drivers->refSerial.getRobotData().turret.barrelSpeedLimit17ID1;

            #elif defined(TARGET_HERO)
                bulletVelocity = drivers->refSerial.getRobotData().turret.barrelSpeedLimit42;
            #endif
        }

        uint8_t numBallisticIterations = 1;

        float turretPitch = 0.0f;
        float turretYaw = 0.0f;
        float projectedTravelTime = 0.0f;

        findTargetProjectileIntersection(
            {targetPosition, targetVelocity, targetAcceleration},
            bulletVelocity,
            numBallisticIterations,
            &turretPitch,
            &turretYaw,
            &projectedTravelTime);

        turret->inputManualAngles(turretYaw, turretPitch);      
    }

    else {
        Remote* remote = &drivers->remote;

        if (drivers->isKillSwitched()) {
            yaw = turret->getCurrentLocalYaw() + turret->getChassisYaw();
            pitch = turret->getCurrentLocalPitch();
        }
        
        else {
            float yawInput = 0.0f;
            float pitchInput = 0.0f;

            yawInput = remote->getMouseX() * MOUSE_SENS_YAW;
            pitchInput = -remote->getMouseY() * MOUSE_SENS_PITCH;

            yaw -= yawInput;
            pitch += pitchInput;
            pitch = modm::min(modm::max(pitch, PITCH_MIN), PITCH_MAX);
        }

        turret->inputManualAngles(yaw, pitch);
    }
}

void CommandMoveTurretAimbot::end(bool) {}

bool CommandMoveTurretAimbot::isFinished(void) const { return false; }
}  // namespace commands
