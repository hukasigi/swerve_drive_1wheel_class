#include "AnglePid.h"
#include "IncrementalPid.h"
#include "PS4Controller.h"
#include "nnct/interfaces/interfaces.hpp"
#include <Arduino.h>

using namespace nnct::interfaces;

class Steering {
    public:
        Steering(Motor* motor, IncrementalEncoder* encoder, LimitSwitch* limit_switch, AnglePID* pid, double offset_deg) {
            this->motor         = motor;
            this->encoder       = encoder;
            this->limit_switch  = limit_switch;
            this->pid           = pid;
            this->target_degree = 0;
            this->offset_degree = offset_deg;
        }
        bool calibrate_zero() { // 0点合わせ
            uint32_t startTime = millis();
            this->motor->run(CALIBRATING_DUTY);
            if (this->limit_switch->active()) { // ONから始まったら、一度OFFになるまで待つ
                this->motor->run(-CALIBRATING_DUTY);
                while (this->limit_switch->active()) {
                    if (millis() - startTime > CALIBRATING_TIMEOUT_MS) {
                        this->motor->stop();
                        return false;
                    }
                }
                this->motor->stop();
                this->encoder->clear();
                this->motor->run(CALIBRATING_DUTY);
            }

            while (!this->limit_switch->active()) { // OFF→ONになるまで待つ
                if (millis() - startTime > CALIBRATING_TIMEOUT_MS) {
                    this->motor->stop();
                    return false;
                }
            }
            this->motor->stop();
            this->encoder->clear();
            return true;
        }
        void   set_target(double degree) { this->target_degree = degree; }
        double get_current_degree() {
            double degree = (encoder->getCount() * 360.0 / ENCODER_RESOLUTION) / STEER_GEAR_RATIO_MOTOR_TO_STEER;

            // offset を角度計算時に補正
            degree -= this->offset_degree;

            return normalizeAngleDeg(degree);
        }
        void update(double dt) {
            double current_degree = this->get_current_degree();
            double duty           = this->pid->update(this->target_degree, current_degree, dt);
            double error          = pid->getError();

            this->motor->run(duty, -1);
        }

    private:
        static double normalizeAngleDeg(double a) {
            while (a > 180.0)
                a -= 360.0;
            while (a < -180.0)
                a += 360.0;
            return a;
        }

        Motor*              motor;
        IncrementalEncoder* encoder;
        LimitSwitch*        limit_switch;
        AnglePID*           pid;

        double target_degree;
        double offset_degree;

        static const int32_t    CALIBRATING_DUTY                = 150;
        static const uint32_t   CALIBRATING_TIMEOUT_MS          = 8000;
        static const uint32_t   ENCODER_RESOLUTION              = 8192;
        static constexpr double STEER_GEAR_RATIO_MOTOR_TO_STEER = 65.0 / 27.0;
};

class Drive {
    public:
        Drive(RobomasMotor* motor, IncrementalPID* pid) {
            this->motor       = motor;
            this->pid         = pid;
            this->target_mm_s = 0.0;
        }
        void   set_target(double drive_target_mm_s) { this->target_mm_s = drive_target_mm_s; }
        double get_current_mm_s() { // rpm -> mm/s
            double       motor_rpm = this->motor->rpm();
            const double wheel_rpm = motor_rpm / DRIVE_GEAR_RATIO;
            return wheel_rpm * 2.0 * M_PI * DRIVE_RADIUS / 60.0;
        }
        void update(double dt) {
            double current_mm_s = this->get_current_mm_s();
            double drive_duty   = this->pid->update(this->target_mm_s, current_mm_s, dt);
            this->motor->run(drive_duty);
        }

    private:
        RobomasMotor*   motor;
        IncrementalPID* pid;

        double target_mm_s;

        static constexpr double DRIVE_RADIUS     = 50.0;       // mm
        static constexpr double DRIVE_GEAR_RATIO = 19.0 / 1.0; // モーター:ホイールの速度比
};

class SwerveDrive {
    public:
        SwerveDrive(Drive* drive, Steering* steering) {
            this->drive    = drive;
            this->steering = steering;
        }
        bool init() {
            if (!this->steering->calibrate_zero()) {
                return false;
            }
            return true;
        }
        void update(double dt) {
            this->steering->update(dt);
            this->drive->update(dt);
        }
        void set_target(double degree, double drive_target_mm_s) {
            double current_degree = this->steering->get_current_degree();

            OptimizedParams params = optimizeSteerAngle(degree, current_degree);

            this->steering->set_target(params.degree);
            this->drive->set_target(drive_target_mm_s * params.drive_dir);
        }

    private:
        Drive*    drive;
        Steering* steering;

        struct OptimizedParams {
                double degree;
                int8_t drive_dir;
        };

        static double normalizeAngleDeg(double a) {
            while (a > 180.0)
                a -= 360.0;
            while (a < -180.0)
                a += 360.0;
            return a;
        }
        // static OptimizedParams optimizeSteerAngle(double steer_target, double currentAngleDeg) {
        //     const double    error = normalizeAngleDeg(steer_target - currentAngleDeg);
        //     OptimizedParams result{steer_target, 1};

        //     if (error > 90.0) {
        //         result.degree    = normalizeAngleDeg(steer_target - 180.0);
        //         result.drive_dir = -1;
        //     } else if (error < -90.0) {
        //         result.degree    = normalizeAngleDeg(steer_target + 180.0);
        //         result.drive_dir = -1;
        //     }
        //     return result;
        // }
        static OptimizedParams optimizeSteerAngle(double steer_target, double currentAngleDeg) {
            double error = normalizeAngleDeg(steer_target - currentAngleDeg);

            OptimizedParams result;
            result.drive_dir = 1;

            // 90°を超えるなら、車輪を180°反転して
            // ドライブ方向を逆にする
            if (error > 90.0) {
                error -= 180.0;
                result.drive_dir = -1;
            } else if (error < -90.0) {
                error += 180.0;
                result.drive_dir = -1;
            }

            // 現在角度の近くにある目標角度を作る
            result.degree = currentAngleDeg + error;

            return result;
        }
};

struct PidParam {
        double p_gain;
        double i_gain;
        double d_gain;
};

using pin_t = uint8_t;
using ch_t  = uint8_t;

const pin_t STEERING_MOTOR_DIR = 23;
const pin_t STEERING_MOTOR_PWM = 22;
const ch_t  STEERING_MOTOR_CH  = 0;
const id_t  DRIVE_MOTOR_ID     = 0x01;
const pin_t STEERING_ENCODER_A = 27;
const pin_t STEERING_ENCODER_B = 14;
const pin_t STEERING_LIMIT_SW  = 36;
const pin_t CAN_RX_PIN         = 4; // 実際の配線に合わせて変更
const pin_t CAN_TX_PIN         = 5; // 実際の配線に合わせて変更

const double OFFSET_1 = 315.;

const int16_t STEER_MOTOR_POWER_LIMIT = 200.;
const int16_t STEER_INTEGRAL_LIMIT    = 10.;
const int16_t RANGE                   = 360;

const int16_t DRIVE_MOTOR_POWER_LIMIT = 50.;
const int16_t DRIVE_INTEGRAL_LIMIT    = 10.;

const double STICK_DEADZONE       = 15.0;
const double DRIVE_MAX_SPEED_MM_S = 1000.0;

const uint32_t        CONTROL_CYCLE_MS   = 10; // 10ms = 100Hz
const struct PidParam STEERING_PID_PARAM = {.p_gain = 2.5, .i_gain = 0.0, .d_gain = 0.0};
const struct PidParam DRIVE_PID_PARAM    = {.p_gain = 1.2, .i_gain = 0.0, .d_gain = 0.0};

const uint32_t CONTROL_LOOP_TASK_STACK_SIZE = 8192;
const uint8_t  CONTROL_LOOP_TASK_PRIORITY   = 10;

TaskHandle_t control_loop_task_handle;

Motor              steering_motor(STEERING_MOTOR_DIR, STEERING_MOTOR_PWM, STEERING_MOTOR_CH);
IncrementalEncoder steering_encoder(STEERING_ENCODER_A, STEERING_ENCODER_B);
LimitSwitch        steering_limit_switch(STEERING_LIMIT_SW);
AnglePID steering_pid(STEERING_PID_PARAM.p_gain, STEERING_PID_PARAM.i_gain, STEERING_PID_PARAM.d_gain, -STEER_MOTOR_POWER_LIMIT,
                      STEER_MOTOR_POWER_LIMIT, -STEER_INTEGRAL_LIMIT, STEER_INTEGRAL_LIMIT, RANGE);
Steering steering(&steering_motor, &steering_encoder, &steering_limit_switch, &steering_pid, OFFSET_1);

RobomasMotor   drive_motor(DRIVE_MOTOR_ID);
RobomasCAN     can(CAN_RX_PIN, CAN_TX_PIN);
IncrementalPID drive_pid(DRIVE_PID_PARAM.p_gain, DRIVE_PID_PARAM.i_gain, DRIVE_PID_PARAM.d_gain, -DRIVE_MOTOR_POWER_LIMIT,
                         DRIVE_MOTOR_POWER_LIMIT, -DRIVE_INTEGRAL_LIMIT, DRIVE_INTEGRAL_LIMIT);
Drive          drive(&drive_motor, &drive_pid);

SwerveDrive swerve_drive(&drive, &steering);

void control_loop_task(void* args) {
    TickType_t wake_time = xTaskGetTickCount();

    while (true) {
        swerve_drive.update(CONTROL_CYCLE_MS / 1000.0);

        // ドライブモータの指令値を CAN で送信
        if (!can.send(&drive_motor)) {
            // Serial.println("Drive CAN send failed");
        }
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(CONTROL_CYCLE_MS));
    }
}

void setup() {
    Serial.begin(115200);
    if (!can.begin()) {
        Serial.println("CAN begin failed");
        while (true) {
        }
    }
    if (!swerve_drive.init()) {
        Serial.println("SwerveDrive init failed");
        while (true) {
        }
    }
    PS4.begin("08:d1:f9:37:41:f2");
    xTaskCreate(control_loop_task, "ControlLoopTask", CONTROL_LOOP_TASK_STACK_SIZE, NULL, CONTROL_LOOP_TASK_PRIORITY,
                &control_loop_task_handle);
}

void loop() {

    if (!PS4.isConnected()) {
        Serial.println("PS4 not connected");
        delay(100);
        return;
    }

    int rx = PS4.RStickX();
    int ry = PS4.RStickY();

    double stickMag = hypot((double)rx, (double)ry);

    if (stickMag <= STICK_DEADZONE) {
        swerve_drive.set_target(steering.get_current_degree(), 0.0);
        drive_pid.reset();
        delay(10);
        return;
    }

    double degree = atan2((double)ry, (double)rx) * 180.0 / M_PI;

    double maxStickMag = 127.0 * sqrt(2.0);

    double drive_target_mm_s = (stickMag - STICK_DEADZONE) / (maxStickMag - STICK_DEADZONE) * DRIVE_MAX_SPEED_MM_S;

    drive_target_mm_s = constrain(drive_target_mm_s, 0.0, DRIVE_MAX_SPEED_MM_S);

    static uint32_t last_log_ms = 0;
    uint32_t        now         = millis();
    swerve_drive.set_target(degree, drive_target_mm_s);

    delay(10);
}