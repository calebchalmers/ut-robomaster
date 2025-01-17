#pragma once

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/can/can_bus.hpp"
#include "tap/motor/dji_motor.hpp"

#include "modm/container/pair.hpp"
#include "utils/motor_controller/pid.hpp"

using motor_controller::PidConstants;
using tap::can::CanBus;
using namespace tap::motor;

// General constants ------------------------------------------------
constexpr CanBus CAN_WHEELS = CanBus::CAN_BUS1;
constexpr CanBus CAN_TURRET = CanBus::CAN_BUS1;
constexpr CanBus CAN_SHOOTER = CanBus::CAN_BUS2;

constexpr PidConstants PID_VELOCITY_DEFAULT = {0.75f, 12.0f, 0.0f};

constexpr bool USE_BALLISTICS = false;
constexpr int BALLISTIC_ITERATIONS = 2;

// Chassis constants ------------------------------------------------
constexpr MotorId ID_WHEEL_LF = MOTOR2;
constexpr MotorId ID_WHEEL_RF = MOTOR1;
constexpr MotorId ID_WHEEL_LB = MOTOR3;
constexpr MotorId ID_WHEEL_RB = MOTOR4;

static constexpr int WHEELS = 4;
static constexpr float WHEEL_DISTANCE_X = 0.391f;  // meters
static constexpr float WHEEL_DISTANCE_Y = 0.315f;  // meters
static constexpr float WHEEL_RADIUS = 0.1524f;     // meters
static constexpr float WHEEL_LXY = (WHEEL_DISTANCE_X + WHEEL_DISTANCE_Y) / 2.0f;
static constexpr float WHEEL_MAX_VEL = 50.0f;                                       // rad/s
static constexpr float MAX_LINEAR_VEL = WHEEL_MAX_VEL * WHEEL_RADIUS;               // m/s
static constexpr float MAX_ANGULAR_VEL = WHEEL_MAX_VEL * WHEEL_RADIUS / WHEEL_LXY;  // rad/s

constexpr PidConstants PID_WHEELS = PID_VELOCITY_DEFAULT;

// Flywheel constants ------------------------------------------------
constexpr MotorId ID_FLYWHEEL_TL = MOTOR3;
constexpr MotorId ID_FLYWHEEL_TR = MOTOR4;
constexpr MotorId ID_FLYWHEEL_BL = MOTOR5;
constexpr MotorId ID_FLYWHEEL_BR = MOTOR6;

constexpr PidConstants PID_FLYWHEEL = {0.1f, 0.1f, 0.0f};

static constexpr modm::Pair<uint16_t, float> FLYWHEEL_RPS_MAPPING[] = {
    {15, 67.0f},
    {18, 75.0f},
    {30, 122.0f}};

static constexpr int FLYWHEELS = 4;
static constexpr float DEFAULT_SPEED = 70.0f;

// Agitator constants ------------------------------------------------
constexpr MotorId ID_AGITATOR_L = MOTOR1;
constexpr MotorId ID_AGITATOR_R = MOTOR2;

constexpr PidConstants PID_AGITATOR = PID_VELOCITY_DEFAULT;

const float BALLS_PER_SEC = 10.0f;
const float BALLS_PER_REV = 8.0f;

const uint16_t BARREL_HEAT_BUFFER = 20.0f;

const float UNJAM_SPEED = 0.4f;  // rev/s

// Turret constants ------------------------------------------------
constexpr MotorId ID_YAW = MOTOR6;
constexpr MotorId ID_PITCH = MOTOR7;

constexpr float PITCH_MIN = -0.2185f;         // rad
constexpr float PITCH_MAX = 0.2299f;          // rad
constexpr float CAMERA_TO_PITCH = 0.13555f;   // distance from main camera lens to pitch axis (m)
constexpr float NOZZLE_TO_PITCH = 0.18151f;   // distance from barrel nozzle to pitch axis (m)
constexpr float CAMERA_TO_BARRELS = 0.0427f;  // vertical ctc offset from camera lens to barrel (m)
constexpr float CAMERA_X_OFFSET = -0.0335f;   // horizontal offset of main camera lens (m)

constexpr PidConstants PID_TURRET_VELOCITY = PID_VELOCITY_DEFAULT;
constexpr PidConstants PID_TURRET_POSITION = {0.1f, 0.1f, 0.0f};

static constexpr float YAW_REDUCTION = 2.0f;
static constexpr float PITCH_REDUCTION = 4.0f;

static constexpr float YAW_INPUT_SCALE = 10.0f;
static constexpr float PITCH_INPUT_SCALE = 5.0f;

static constexpr float MOUSE_SENS_YAW = 0.1f;
static constexpr float MOUSE_SENS_PITCH = 0.1f;

static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 65'000.0f,
    .ki = 0.0f,
    .kd = 3'000.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 32'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 30.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 10.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 100'183.1f,
    .ki = 0.0f,
    .kd = 1'000.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 32'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 30.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};