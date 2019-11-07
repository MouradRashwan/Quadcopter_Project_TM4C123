/*
 * MOTOR_defs.h
 *
 *  Created on: Mar 25, 2019
 *      Author: Administrator
 */

#ifndef MOTOR_DRIVER_MOTOR_DEFS_H_
#define MOTOR_DRIVER_MOTOR_DEFS_H_

#define MOTOR_PULSE_MIN_MICROS              (1000U)
#define MOTOR_PULSE_MAX_MICROS              (2000U)

#define PWM_GET_TICKS_FROM_MICROS(MICROS)   ((MICROS) * (QuadPWM_getClk() / 1000000U))

typedef enum
{
    MOTOR_SUCCESS = 0, MOTOR_ERROR = 1
} MOTORStatus_t;

#endif /* MOTOR_DRIVER_MOTOR_DEFS_H_ */
