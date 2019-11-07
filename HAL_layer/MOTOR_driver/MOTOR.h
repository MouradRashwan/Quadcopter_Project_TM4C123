/*
 * MOTOR.h
 *
 *  Created on: Mar 25, 2019
 *      Author: Administrator
 */

#ifndef MOTOR_DRIVER_MOTOR_H_
#define MOTOR_DRIVER_MOTOR_H_

MOTORStatus_t MOTOR_PWM_init(void);

void MOTOR_update(PulseWidth_t * const ptPulseWidth);

void MOTOR_setSpeedON(void);

void MOTOR_setSpeedOFF(void);

void MOTOR_setSpeedFULL(void);

void MOTOR_increaseSpeed(const uint32_t ui32DeltaInMicros);

void MOTOR_decreaseSpeed(const uint32_t ui32DeltaInMicros);

#endif /* MOTOR_DRIVER_MOTOR_H_ */
