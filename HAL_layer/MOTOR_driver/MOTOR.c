/*
 * MOTOR.c
 *
 *  Created on: Mar 25, 2019
 *      Author: Administrator
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "MOTOR_driver.h"

void MOTOR_update(PulseWidth_t * const ptPulseWidth)
{
    /* Set QuadPWM output pulses */
    QuadPWM_setPulses(ptPulseWidth);
    QuadPWM_enableOutputs();
    QuadPWM_updateReset();
    QuadPWM_disableOutputs();
    QuadPWM_update();
}

MOTORStatus_t MOTOR_PWM_init(void)
{
    /* Initialize QuadPWM module */
    return (MOTORStatus_t) QuadPWM_init(PWM_SIGNAL_FREQ, PWM_SIGNAL_ALIGN);
}

void MOTOR_setSpeedON(void)
{
    PulseWidth_t tPulseWidth;

    /* Get current output pulses */
    QuadPWM_getPulses(&tPulseWidth);

    /* Initialize QuadPWM outputs to 1000 us */
    tPulseWidth.ui32PulseInTicks1 = PWM_GET_TICKS_FROM_MICROS(
            MOTOR_READY_PULSE_MIN_MICROS);
    tPulseWidth.ui32PulseInTicks2 = PWM_GET_TICKS_FROM_MICROS(
            MOTOR_READY_PULSE_MIN_MICROS);
    tPulseWidth.ui32PulseInTicks3 = PWM_GET_TICKS_FROM_MICROS(
            MOTOR_READY_PULSE_MIN_MICROS);
    tPulseWidth.ui32PulseInTicks4 = PWM_GET_TICKS_FROM_MICROS(
            MOTOR_READY_PULSE_MIN_MICROS);

    /* Update Motors */
    MOTOR_update(&tPulseWidth);
}

void MOTOR_setSpeedOFF(void)
{
    PulseWidth_t tPulseWidth;

    /* Get current output pulses */
    QuadPWM_getPulses(&tPulseWidth);

    /* Initialize QuadPWM outputs to 1000 us */
    tPulseWidth.ui32PulseInTicks1 = PWM_GET_TICKS_FROM_MICROS(
            MOTOR_PULSE_MIN_MICROS);
    tPulseWidth.ui32PulseInTicks2 = PWM_GET_TICKS_FROM_MICROS(
            MOTOR_PULSE_MIN_MICROS);
    tPulseWidth.ui32PulseInTicks3 = PWM_GET_TICKS_FROM_MICROS(
            MOTOR_PULSE_MIN_MICROS);
    tPulseWidth.ui32PulseInTicks4 = PWM_GET_TICKS_FROM_MICROS(
            MOTOR_PULSE_MIN_MICROS);

    /* Update Motors */
    MOTOR_update(&tPulseWidth);
}

void MOTOR_setSpeedFULL(void)
{
    PulseWidth_t tPulseWidth;

    /* Get current output pulses */
    QuadPWM_getPulses(&tPulseWidth);

    /* Initialize QuadPWM outputs to 1000 us */
    tPulseWidth.ui32PulseInTicks1 = PWM_GET_TICKS_FROM_MICROS(
            MOTOR_PULSE_MAX_MICROS);
    tPulseWidth.ui32PulseInTicks2 = PWM_GET_TICKS_FROM_MICROS(
            MOTOR_PULSE_MAX_MICROS);
    tPulseWidth.ui32PulseInTicks3 = PWM_GET_TICKS_FROM_MICROS(
            MOTOR_PULSE_MAX_MICROS);
    tPulseWidth.ui32PulseInTicks4 = PWM_GET_TICKS_FROM_MICROS(
            MOTOR_PULSE_MAX_MICROS);

    /* Update Motors */
    MOTOR_update(&tPulseWidth);
}

void MOTOR_increaseSpeed(const uint32_t ui32DeltaInMicros)
{
    PulseWidth_t tPulseWidth;

    /* Get current output pulses */
    QuadPWM_getPulses(&tPulseWidth);

    /* Increase output pulses */
    tPulseWidth.ui32PulseInTicks1 += PWM_GET_TICKS_FROM_MICROS(
            ui32DeltaInMicros);
    tPulseWidth.ui32PulseInTicks2 += PWM_GET_TICKS_FROM_MICROS(
            ui32DeltaInMicros);
    tPulseWidth.ui32PulseInTicks3 += PWM_GET_TICKS_FROM_MICROS(
            ui32DeltaInMicros);
    tPulseWidth.ui32PulseInTicks4 += PWM_GET_TICKS_FROM_MICROS(
            ui32DeltaInMicros);

    /* Get value within boundaries [MOTOR_READY_PULSE_MAX_MICROS & MOTOR_READY_PULSE_MIN_MICROS] */
    getValueWithinLimits(
            &tPulseWidth.ui32PulseInTicks1,
            PWM_GET_TICKS_FROM_MICROS(MOTOR_READY_PULSE_MAX_MICROS),
            PWM_GET_TICKS_FROM_MICROS(MOTOR_READY_PULSE_MIN_MICROS));

    getValueWithinLimits(
            &tPulseWidth.ui32PulseInTicks2,
            PWM_GET_TICKS_FROM_MICROS(MOTOR_READY_PULSE_MAX_MICROS),
            PWM_GET_TICKS_FROM_MICROS(MOTOR_READY_PULSE_MIN_MICROS));

    getValueWithinLimits(
            &tPulseWidth.ui32PulseInTicks3,
            PWM_GET_TICKS_FROM_MICROS(MOTOR_READY_PULSE_MAX_MICROS),
            PWM_GET_TICKS_FROM_MICROS(MOTOR_READY_PULSE_MIN_MICROS));

    getValueWithinLimits(
            &tPulseWidth.ui32PulseInTicks4,
            PWM_GET_TICKS_FROM_MICROS(MOTOR_READY_PULSE_MAX_MICROS),
            PWM_GET_TICKS_FROM_MICROS(MOTOR_READY_PULSE_MIN_MICROS));

    /* Update Motors */
    MOTOR_update(&tPulseWidth);
}

void MOTOR_decreaseSpeed(const uint32_t ui32DeltaInMicros)
{
    PulseWidth_t tPulseWidth;

    /* Get current output pulses */
    QuadPWM_getPulses(&tPulseWidth);

    /* Decrease output pulses */
    tPulseWidth.ui32PulseInTicks1 -= PWM_GET_TICKS_FROM_MICROS(
            ui32DeltaInMicros);
    tPulseWidth.ui32PulseInTicks2 -= PWM_GET_TICKS_FROM_MICROS(
            ui32DeltaInMicros);
    tPulseWidth.ui32PulseInTicks3 -= PWM_GET_TICKS_FROM_MICROS(
            ui32DeltaInMicros);
    tPulseWidth.ui32PulseInTicks4 -= PWM_GET_TICKS_FROM_MICROS(
            ui32DeltaInMicros);

    /* Get value within boundaries [MOTOR_READY_PULSE_MAX_MICROS & MOTOR_READY_PULSE_MIN_MICROS] */
    getValueWithinLimits(
            &tPulseWidth.ui32PulseInTicks1,
            PWM_GET_TICKS_FROM_MICROS(MOTOR_READY_PULSE_MAX_MICROS),
            PWM_GET_TICKS_FROM_MICROS(MOTOR_READY_PULSE_MIN_MICROS));
    getValueWithinLimits(
            &tPulseWidth.ui32PulseInTicks2,
            PWM_GET_TICKS_FROM_MICROS(MOTOR_READY_PULSE_MAX_MICROS),
            PWM_GET_TICKS_FROM_MICROS(MOTOR_READY_PULSE_MIN_MICROS));
    getValueWithinLimits(
            &tPulseWidth.ui32PulseInTicks3,
            PWM_GET_TICKS_FROM_MICROS(MOTOR_READY_PULSE_MAX_MICROS),
            PWM_GET_TICKS_FROM_MICROS(MOTOR_READY_PULSE_MIN_MICROS));
    getValueWithinLimits(
            &tPulseWidth.ui32PulseInTicks4,
            PWM_GET_TICKS_FROM_MICROS(MOTOR_READY_PULSE_MAX_MICROS),
            PWM_GET_TICKS_FROM_MICROS(MOTOR_READY_PULSE_MIN_MICROS));

    /* Update Motors */
    MOTOR_update(&tPulseWidth);
}
