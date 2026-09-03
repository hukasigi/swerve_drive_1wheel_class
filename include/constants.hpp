#pragma once
#include <Arduino.h>
// units: mm, deg, rad, mm/s

struct PidParam {
        double p_gain;
        double i_gain;
        double d_gain;
};

using pin_t = uint8_t;
using ch_t  = uint8_t;

// ステアリング1
const pin_t  STEERING_MOTOR_DIR_1 = 23;
const pin_t  STEERING_MOTOR_PWM_1 = 22;
const ch_t   STEERING_MOTOR_CH_1  = 0;
const id_t   DRIVE_MOTOR_ID_1     = 0x01;
const pin_t  STEERING_ENCODER_A_1 = 27;
const pin_t  STEERING_ENCODER_B_1 = 14;
const pin_t  STEERING_LIMIT_SW_1  = 36;
const double OFFSET_DEG_1         = 45.;

// ステアリング2
const pin_t  STEERING_MOTOR_DIR_2 = 21;
const pin_t  STEERING_MOTOR_PWM_2 = 19;
const ch_t   STEERING_MOTOR_CH_2  = 1;
const id_t   DRIVE_MOTOR_ID_2     = 0x02;
const pin_t  STEERING_ENCODER_A_2 = 25;
const pin_t  STEERING_ENCODER_B_2 = 26;
const pin_t  STEERING_LIMIT_SW_2  = 39;
const double OFFSET_DEG_2         = 165.;

// ステアリング3
const pin_t  STEERING_MOTOR_DIR_3 = 18;
const pin_t  STEERING_MOTOR_PWM_3 = 17;
const ch_t   STEERING_MOTOR_CH_3  = 2;
const id_t   DRIVE_MOTOR_ID_3     = 0x04;
const pin_t  STEERING_ENCODER_A_3 = 32;
const pin_t  STEERING_ENCODER_B_3 = 33;
const pin_t  STEERING_LIMIT_SW_3  = 34;
const double OFFSET_DEG_3         = 285.;

const pin_t CAN_RX_PIN = 4; // 実際の配線に合わせて変更
const pin_t CAN_TX_PIN = 5; // 実際の配線に合わせて変更

// ステア制御パラメータ
const int16_t STEER_MOTOR_POWER_LIMIT = 200;
const int16_t STEER_INTEGRAL_LIMIT    = 10;
const int16_t RANGE                   = 360;

// ドライブ制御パラメータ
const int16_t DRIVE_MOTOR_POWER_LIMIT = 255.;
const int16_t DRIVE_INTEGRAL_LIMIT    = 10.;

// コントローラ
const double     MAGNITUDE_DEADZONE = 15.0;
const uint32_t   CONTROL_CYCLE_MS   = 10; // 10ms = 100Hz
constexpr double CONTROL_CYCLE_S    = CONTROL_CYCLE_MS / 1000.0;

// PIDパラメータ
const struct PidParam STEERING_PID_PARAM = {.p_gain = 4.4, .i_gain = 0.2, .d_gain = 0.0};
const struct PidParam DRIVE_PID_PARAM    = {.p_gain = 1.2, .i_gain = 0.0, .d_gain = 0.0};

// FreeRTOS
constexpr uint32_t CONTROL_LOOP_TASK_STACK_SIZE = 8192;
constexpr uint8_t  CONTROL_LOOP_TASK_PRIORITY   = 10;
constexpr uint32_t LOOP_DELAY_MS                = 10;

constexpr std::array<double, 3> STEER_ZERO_ANGLE_DEG = {45.0, 165.0, 285.0}; // 各ステア原点

constexpr double DRIVE_RADIUS = 50;

constexpr int8_t   ENCODER_SIGN_1     = 1;
constexpr int8_t   ENCODER_SIGN_2     = 1;
constexpr int8_t   ENCODER_SIGN_3     = 1;
constexpr uint32_t ENCODER_RESOLUTION = 8192;

constexpr double        GEAR_RATIO       = 1.0;
static constexpr double DRIVE_GEAR_RATIO = 19.0 / 1.0; // モーター:ホイールの速度比

static constexpr double STEER_GEAR_RATIO_MOTOR_TO_STEER = 65.0 / 27.0;

static const int32_t CALIBRATING_DUTY = 150;

// ホイール物理最大速度（実機に合わせて調整）  482rpmなので1200mm/s
const double DRIVE_MAX_SPEED_MM_S = 1000.0;
