/**
 * @file main.cpp
 * @author Alex Au (alex_acw@outlook.com)
 * @brief
 * @version 0.1
 * @date 2019-04-24
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "DR16.h"
#include "MotorControl.hpp"
#include "MotorControlImpl.hpp"
#include "PID_aw.hpp"
#include "Persistence.hpp"
#include "ShellManager.h"
#include "Snail_Sentry.hpp"
#include "arm_math.h"
#include "boardSetup.h"
#include "buzzer.h"
#include "ch.hpp"
#include "hal.h"
#include "motor.h"

#define DEG2ENC(deg) (deg * 8192 * 19 / 360)

/**
 * @brief Same filter config for both CAN to receive all motor
 * feedbacks
 */
const CAN_FilterConfig_t *const canFilter[] = {
    // Motor_CANFilter_200 is to prevent calling motor feedback
    // callback by redirecting command from other device
    &Motor_CANFilter_200, &Motor_CANFilter_201_207, &Motor_CANFilter_208_20B};

// following are startup parameters for pid control, these are tested
// on free spinning situation only
MotorControl::PID_aw_params speedPidParam_3508 =
    MotorControl::PID_aw_params("3508_speed", 20.0);
MotorControl::PID_aw_params orientationPidParam_3508 =
    MotorControl::PID_aw_params("3508_orientation", 0.2);

using ArmMotorController =
    MotorControl::GenericController<MotorControl::PID_aw,
                                    MotorControl::PID_aw_params,
                                    MotorControl::C6X0_MotorAccessor>;

ArmMotorController motors[6] = {
    ArmMotorController(&MOTOR_CAN1(0x201), orientationPidParam_3508,
                       speedPidParam_3508, 16384, TIME_MS2I(200), 200,
                       TIME_MS2I(200), 8192 * 10 / 360, 10000),
    ArmMotorController(&MOTOR_CAN1(0x202), orientationPidParam_3508,
                       speedPidParam_3508, 16384, TIME_MS2I(200), 200,
                       TIME_MS2I(200), 8192 * 10 / 360, 10000),
    ArmMotorController(&MOTOR_CAN1(0x203), orientationPidParam_3508,
                       speedPidParam_3508, 16384, TIME_MS2I(200), 200,
                       TIME_MS2I(200), 8192 * 10 / 360, 10000),
    ArmMotorController(&MOTOR_CAN1(0x204), orientationPidParam_3508,
                       speedPidParam_3508, 16384, TIME_MS2I(200), 200,
                       TIME_MS2I(200), 8192 * 10 / 360, 10000),
    ArmMotorController(&MOTOR_CAN1(0x205), orientationPidParam_3508,
                       speedPidParam_3508, 16384, TIME_MS2I(200), 200,
                       TIME_MS2I(200), 8192 * 10 / 360, 10000),
    ArmMotorController(&MOTOR_CAN1(0x206), orientationPidParam_3508,
                       speedPidParam_3508, 16384, TIME_MS2I(200), 200,
                       TIME_MS2I(200), 8192 * 10 / 360, 10000),
};
ArmMotorController &motorL1 = motors[0];
ArmMotorController &motorL2 = motors[1];
ArmMotorController &motorL3 = motors[2];
ArmMotorController &motorR1 = motors[3];
ArmMotorController &motorR2 = motors[4];
ArmMotorController &motorR3 = motors[5];

const int clearance = DEG2ENC(5);
const int maxOrientation[6] = {-clearance,  DEG2ENC(110), -clearance,
                               DEG2ENC(80), -clearance,   DEG2ENC(40)};
const int minOrientation[6] = {-DEG2ENC(80), clearance,     -DEG2ENC(40),
                               clearance,    -DEG2ENC(110), clearance};

MotorControl::ControllerBase *motorControllers[] = {
    &motorL1, &motorL2, &motorL3, &motorR1, &motorR2, &motorR3};

static MotorControl::MotorControlThread<TIME_MS2I(2), 256> motorCtrlThd(
    motorControllers,
    sizeof(motorControllers) / sizeof(MotorControl::ControllerBase *));

void homeAllMotor();

int main(void)
{
    halInit();
    chSysInit();
    board_setup_all();
    Buzzer_initialize();

    CAN_Manager_start(
        canFilter, sizeof(canFilter) / sizeof(CAN_FilterConfig_t *), canFilter,
        sizeof(canFilter) / sizeof(CAN_FilterConfig_t *));

    // Initialize motor monitoring thread and data structures
    Motor_initialize();
    speedPidParam_3508.addParams();
    orientationPidParam_3508.addParams();
    motorCtrlThd.run(NORMALPRIO);

    palSetPadMode(GPIOI, 5, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOA, 2, PAL_MODE_OUTPUT_PUSHPULL);
    palClearPad(GPIOI, 5);
    palClearPad(GPIOA, 2);

    Param_LoadAll();

    // setup shell for tuning
    static const ShellCommand shellCommands[] = {g_Persistence_ShellCommand,
                                                 {NULL, NULL}};
    ShellManager_start(shellCommands);

    static volatile unsigned ledBits = 0x1F;
    palSetLine(LINE_LED_GREEN);
    homeAllMotor();
    DR16_start();
    while (true)
    {
        if (ledBits & 0b1U)
        {
            ledBits |= (0b1U << 9);
            palToggleLine(LINE_LED_GREEN);
            palToggleLine(LINE_LED_RED);
        }
        ledBits >>= 1;
        palWritePort(GPIOG, ledBits);
        chThdSleepMilliseconds(250);
    }
}

const int DEG_PER_SEC = 720;

static bool wasConnected = false;
void DR16_onReceiveProcess()
{
    static int targetOrientation[6];
    static systime_t lastupdate;
    static unsigned lastS2 = g_DR16_RC_Value.rc.s2;
    if (!wasConnected)
    {
        for (unsigned i = 0; i < sizeof(motors) / sizeof(ArmMotorController);
             i++)
        {
            targetOrientation[i] = motors[i].getMotor().accumulated_orientation;
        }
        lastupdate = chVTGetSystemTimeX();
        lastS2 = g_DR16_RC_Value.rc.s2;
    }
    float dt = (float)chVTTimeElapsedSinceX(lastupdate) / chTimeS2I(1);
    lastupdate = chVTGetSystemTimeX();

    int dR = dt * DEG_PER_SEC * 8192 * 19 *
             (g_DR16_RC_Value.rc.ch1 - RC_CH_VALUE_MID) /
             RC_CH_VALUE_ABS_RANGE / 360;

    int dL = dt * DEG_PER_SEC * 8192 * 19 *
             (g_DR16_RC_Value.rc.ch3 - RC_CH_VALUE_MID) /
             RC_CH_VALUE_ABS_RANGE / 360;

    switch (g_DR16_RC_Value.rc.s1)  // upper right switch
    {
        case RC_SW_DOWN:
            targetOrientation[0] -= dL;
            targetOrientation[3] += dR;
            break;
        case RC_SW_MID:
            targetOrientation[1] += dL;
            targetOrientation[4] -= dR;
            break;
        case RC_SW_UP:
            targetOrientation[2] += dL;
            targetOrientation[5] -= dR;
            motorL3.setRPM(10000 * (g_DR16_RC_Value.rc.ch3 - RC_CH_VALUE_MID) /
                           RC_CH_VALUE_ABS_RANGE);
            // motorR3.setRPM(-10000 * (g_DR16_RC_Value.rc.ch1 -
            // RC_CH_VALUE_MID) /
            //                RC_CH_VALUE_ABS_RANGE);
            break;
    }
    for (unsigned i = 0; i < sizeof(motors) / sizeof(ArmMotorController); i++)
    {
        if (targetOrientation[i] > maxOrientation[i])
            targetOrientation[i] = maxOrientation[i];
        else if (targetOrientation[i] < minOrientation[i])
            targetOrientation[i] = minOrientation[i];
    }

    if (lastS2 != g_DR16_RC_Value.rc.s2)
        switch (g_DR16_RC_Value.rc.s2)  // upper left switch
        {
            case RC_SW_DOWN:
                palTogglePad(GPIOI, 5);
                break;
            case RC_SW_MID:
                break;
            case RC_SW_UP:
                palTogglePad(GPIOA, 2);
                break;
        }
    motorL1.setEncOrientation(targetOrientation[0]);
    motorL2.setEncOrientation(targetOrientation[1]);
    // motorL3.setEncOrientation(targetOrientation[2]);
    motorR1.setEncOrientation(targetOrientation[3]);
    motorR2.setEncOrientation(targetOrientation[4]);
    motorR3.setEncOrientation(targetOrientation[5]);
    wasConnected = true;
    lastS2 = g_DR16_RC_Value.rc.s2;
};

void DR16_disconnect_cb()
{
    wasConnected = false;
    palClearPad(GPIOI, 5);
    palClearPad(GPIOA, 2);
    motorL1.brake();
    motorL2.brake();
    motorL3.brake();
    motorR1.brake();
    motorR2.brake();
    motorR3.brake();
}

void homeOneMotor(ArmMotorController &motor, int speed, unsigned int current)
{
    chibios_rt::EventListener evtLis;
    motor.evtSrc.registerOne(&evtLis, 0);
    motor.setOutputAbsMax(current);
    motor.setRPM(speed);
    chibios_rt::BaseThread::waitAnyEvent(ALL_EVENTS);
    if (evtLis.getAndClearFlags() | MotorControl::stallEvt)
        motor.resetAccumulatedOrientation();
    motor.brake();
    motor.evtSrc.unregister(&evtLis);
    Buzzer_asyncPlayNote(2000, 100, TIME_MS2I(200));
}
// perform homing sequence
void homeAllMotor()
{
    homeOneMotor(motorL3, 500, 5000);
    homeOneMotor(motorL1, 500, 5000);
    homeOneMotor(motorL2, -500, 5000);
    homeOneMotor(motorR3, -500, 5000);
    homeOneMotor(motorR1, -500, 5000);
    homeOneMotor(motorR2, 500, 5000);
    motorL1.brake();
    motorL2.brake();
    motorL3.brake();
    motorR1.brake();
    motorR2.brake();
    motorR3.brake();
};
