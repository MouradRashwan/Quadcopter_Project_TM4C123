/*
 * QuadPWM_defs.h
 *
 *  Created on: Mar 10, 2019
 *      Author: Administrator
 */

#ifndef QUADPWM_DEFS_H_
#define QUADPWM_DEFS_H_

typedef enum
{
     PWM_SUCCESS = 0, PWM_ERROR = 1
} PWMStatus_t;

typedef enum
{
    UPDATE_COMPLETE = 0x00000000U, UPDATE_PENDING = (PWM_GEN_0_BIT
            | PWM_GEN_1_BIT | PWM_GEN_2_BIT | PWM_GEN_3_BIT)
} UpdateStatus_t;

typedef enum
{
    CENTER_ALIGNED = 0,
    LEFT_ALIGNED = 1U,
    RIGHT_ALIGNED = 2U,
    UKNOWN_ALIGNED = 3U
} SignalAlign_t;

typedef struct
{
    uint32_t ui32PulseInTicks1;
    uint32_t ui32PulseInTicks2;
    uint32_t ui32PulseInTicks3;
    uint32_t ui32PulseInTicks4;
} PulseWidth_t;

#endif /* QUADPWM_DEFS_H_ */
