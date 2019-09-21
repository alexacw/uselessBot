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

ArmMotorController motorL1(&MOTOR_CAN1(0x201), orientationPidParam_3508,
                           speedPidParam_3508, 16384, TIME_MS2I(200), 200,
                           TIME_MS2I(200), 8192 * 10 / 360, 1000);
ArmMotorController motorL2(&MOTOR_CAN1(0x202), orientationPidParam_3508,
                           speedPidParam_3508, 16384, TIME_MS2I(200), 200,
                           TIME_MS2I(200), 8192 * 10 / 360, 1000);
ArmMotorController motorL3(&MOTOR_CAN1(0x203), orientationPidParam_3508,
                           speedPidParam_3508, 16384, TIME_MS2I(200), 200,
                           TIME_MS2I(200), 8192 * 10 / 360, 1000);

MotorControl::ControllerBase *motorControllers[] = {&motorL1, &motorL2,
                                                    &motorL3};

static MotorControl::MotorControlThread<TIME_MS2I(2), 256> motorCtrlThd(
    motorControllers,
    sizeof(motorControllers) / sizeof(MotorControl::ControllerBase *));

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

    Param_LoadAll();
    DR16_start();

    // setup shell for tuning
    static const ShellCommand shellCommands[] = {g_Persistence_ShellCommand,
                                                 {NULL, NULL}};
    ShellManager_start(shellCommands);

    static volatile unsigned ledBits = 0x1F;
    palSetLine(LINE_LED_GREEN);
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

const int DEG_PER_SEC = 90;

static bool wasConnected = false;
void DR16_onReceiveProcess()
{
    static int lastOrientation[3];
    if (!wasConnected)
    {
        lastOrientation[0] = motorL1.getMotor().accumulated_orientation;
        lastOrientation[1] = motorL2.getMotor().accumulated_orientation;
        lastOrientation[2] = motorL3.getMotor().accumulated_orientation;
    }
    static systime_t lastupdate;
    float dt = (float)chVTTimeElapsedSinceX(lastupdate) / chTimeS2I(1);
    switch (g_DR16_RC_Value.rc.s1)  // upper right switch
    {
        case RC_SW_DOWN:
            lastOrientation[0] += dt * DEG_PER_SEC * 8192 * 19 *
                                  (g_DR16_RC_Value.rc.ch0 - RC_CH_VALUE_MID) /
                                  RC_CH_VALUE_ABS_RANGE / 360;
            break;

        case RC_SW_MID:
            lastOrientation[1] += dt * DEG_PER_SEC * 8192 * 19 *
                                  (g_DR16_RC_Value.rc.ch0 - RC_CH_VALUE_MID) /
                                  RC_CH_VALUE_ABS_RANGE / 360;
            break;

        case RC_SW_UP:
            lastOrientation[2] += dt * DEG_PER_SEC * 8192 * 19 *
                                  (g_DR16_RC_Value.rc.ch0 - RC_CH_VALUE_MID) /
                                  RC_CH_VALUE_ABS_RANGE / 360;
            break;
    }
    motorL1.setEncOrientation(lastOrientation[0]);
    motorL2.setEncOrientation(lastOrientation[1]);
    motorL3.setEncOrientation(lastOrientation[2]);
    lastupdate = chVTGetSystemTimeX();
    wasConnected = true;
};

void DR16_disconnect_cb()
{
    wasConnected = false;
    motorL1.brake();
    motorL2.brake();
    motorL3.brake();
}
