#pragma once

#include "cmsis_os.h"
#include "information/device.hpp"
#include "information/pid.hpp"
#include "init.hpp"
#include "stm32f4xx_hal.h"
#include "tim.h"

#define M3508_GEAR_RATIO 1.0 / 19.0
#define CHASSIS_GEAR_RATIO 44.0 / 18.0

#define IN_TO_METER(pv) (pv * 0.0254f)
#define METER_TO_IN(pv) (pv * 39.37f)

#define WHEEL_DIAM 1.62
#define WHEEL_RADIUS WHEEL_DIAM / 2.0
#define WHEEL_CUM WHEEL_DIAM* PI

#define RS_TO_RPM(pv) (pv * 9.549297)

#define CHAS_LINEAR(pv) (pv * M3508_GEAR_RATIO * CHASSIS_GEAR_RATIO * WHEEL_RADIUS)
#define CHAS_DELINEAR(pv) (pv / (M3508_GEAR_RATIO * CHASSIS_GEAR_RATIO * WHEEL_RADIUS))

#define METERSPS_TO_RPM(pv) ((pv * 461.539f))

#define RAIL_LEN 66.0 // 66in

extern double railPosition;
extern float lastChassisAngle;

namespace chassis {

class chassisMotor : public canMotor {
private:
    pidInstance* PID;

public:
    chassisMotor(int16_t ID, float32_t lC, float32_t uC, filter::Kalman filter) : canMotor(ID, -100, 100, filter, 8191, 19) {}
    chassisMotor(int16_t ID, float32_t lC, float32_t uC, pidInstance& pid, filter::Kalman filter) : canMotor(ID, -100, 100, filter, 8191, 19), PID(&pid) {}
    chassisMotor(int16_t ID, pidInstance& pid, filter::Kalman filter) : canMotor(ID, -100, 100, filter, 8191, 19), PID(&pid) {}

    void setPID(pidInstance& pid) {
        PID = &pid;
    }
};
extern chassisMotor c1Motor;

enum chassisStates {
    notRunning = 0,
    manual = 1,
    toTargetVel = 2,
    toTargetPos = 3,
    yield = 4,
};
extern chassisStates currState;

struct chassisTargets {
    float m1;
};

enum CtrlTypes {
    CURRENT,
    VOLTAGE,
    SPEED,
};
extern CtrlTypes ctrlType;

extern filter::Kalman chassisVelFilter;

extern void pwmInitialize();

extern void task();

extern void update();

extern void act();

extern void rcToPower(double angle, double magnitude, double yaw);

extern void sendChassisMessage(float m1);

extern void updateRailPosition();

extern void profiledMove(float distance);

extern void positionMove(float distance);

} // namespace chassis
