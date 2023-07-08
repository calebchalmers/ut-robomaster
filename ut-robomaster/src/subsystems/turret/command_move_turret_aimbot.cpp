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
        if (lastTurretDataIndex == drivers->beagleboneCommunicator.getTurretDataIndex()) return;
        lastTurretDataIndex = drivers->beagleboneCommunicator.getTurretDataIndex();

        Vector3f targetPosition = Vector3f(
                cv.xPos + CAMERA_X_OFFSET,
                cv.zPos + CAMERA_TO_PITCH_OFFSET,
                cv.yPos + CAMERA_TO_BARREL_OFFSET);
        Vector3f targetVelocity = Vector3f(cv.xVel, cv.zVel, cv.yVel);
        Vector3f targetAcceleration = Vector3f(cv.xAcc, cv.zAcc, cv.yAcc);

        if (USE_BALLISTICS) {
            // Rotate to world relative pitch
            float a = turret->getCurrentLocalPitch();
            const float matData[9] = {1.0f, 0, 0, 0, cos(a), -sin(a), 0, sin(a), cos(a)};
            modm::Matrix3f rotMat(matData);
            targetPosition = rotMat * targetPosition;
            targetVelocity = rotMat * targetVelocity;
            targetAcceleration = rotMat * targetAcceleration;

            MeasuredKinematicState kinematicState = {targetPosition, targetVelocity, targetAcceleration};
            
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
                kinematicState,
                bulletVelocity,
                numBallisticIterations,
                &turretPitch,
                &turretYaw,
                &projectedTravelTime,
                -NOZZLE_TO_PITCH_OFFSET);

            float currentWorldYaw = turret->getCurrentLocalYaw() + turret->getChassisYaw();
            turret->inputManualAngles(currentWorldYaw + turretYaw, modm::min(modm::max(turretPitch, PITCH_MIN), PITCH_MAX));
        }

        else {
            float deltaYaw = -atan(targetPosition.x / targetPosition.y);  // yaw is opposite to camera X
            float deltaPitch = atan(targetPosition.z / targetPosition.y);
            float scale = 0.006f;
            // float currentWorldYaw = turret->getCurrentLocalYaw() + turret->getChassisYaw();
            // float currentWorldPitch = turret->getCurrentLocalPitch();

            turret->inputManualAngles(
                turret->getTargetWorldYaw() + deltaYaw * scale,
                modm::min(modm::max(turret->getTargetWorldPitch() + deltaPitch * scale, PITCH_MIN), PITCH_MAX));
        } 
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
