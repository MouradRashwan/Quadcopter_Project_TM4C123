/*
 * REMOTE.h
 *
 *  Created on: Mar 25, 2019
 *      Author: Administrator
 */

#ifndef REMOTE_DRIVER_REMOTE_H_
#define REMOTE_DRIVER_REMOTE_H_

void REMOTE_init(const uint32_t ui32Buadrate);

int32_t REMOTE_receiveByte(void);

void REMOTE_transmitByte(uint8_t ui8Data);

void REMOTE_transmitData(void * pvData, uint32_t ui32DataLen);

#endif /* REMOTE_DRIVER_REMOTE_H_ */
