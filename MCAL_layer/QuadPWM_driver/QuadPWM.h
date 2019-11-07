/*
 * QuadPWM.h
 *
 *  Created on: Mar 10, 2019
 *      Author: Administrator
 */

#ifndef QUADPWM_H_
#define QUADPWM_H_

PWMStatus_t QuadPWM_init(uint32_t ui32PWMFreqInHz, SignalAlign_t tSignalAlign); /* [B6 - B4 - E4 - C4] */

void QuadPWM_setPulses(PulseWidth_t *ptPulseWidth);

void QuadPWM_getPulses(PulseWidth_t *ptPulseWidth);

void QuadPWM_enableOutputs(void);

void QuadPWM_disableOutputs(void);

void QuadPWM_update(void);

void QuadPWM_updateReset(void);

UpdateStatus_t QuadPWM_getUpdateStatus(void);

uint32_t QuadPWM_getPeriod(void);

uint32_t QuadPWM_getFreq(void);

uint32_t QuadPWM_getClk(void);

#endif /* QUADPWM_H_ */
