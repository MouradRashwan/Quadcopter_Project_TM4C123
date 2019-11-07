/*
 * I2C_defs.h
 *
 *  Created on: Feb 19, 2019
 *      Author: Administrator
 */

#ifndef I2C_DEFS_H_
#define I2C_DEFS_H_

#define I2C_DEVICES_MAX     128U

typedef struct I2CObject
{
    uint32_t ui32I2cClkBase;
    uint32_t ui32I2cAddrBase;
    uint32_t ui32PortClkBase;
    uint32_t ui32PortAddrBase;
    uint32_t ui32PortPinSCL;
    uint32_t ui32PortPinSDA;
    uint32_t ui32PortPctlSCL;
    uint32_t ui32PortPctlSDA;
} I2CObject_t;

typedef enum I2CDirection
{
    TRANSMIT = 0, RECEIVE = 1
} I2CDirection_t;

typedef enum I2CStatus
{
    ERROR = 0xFF,
    I2C_SUCCESS = I2C_MASTER_ERR_NONE,
    I2C_ERR_ADDR_ACK = I2C_MASTER_ERR_ADDR_ACK,
    I2C_ERR_DATA_ACK = I2C_MASTER_ERR_DATA_ACK,
    I2C_ERR_ARB_LOST = I2C_MASTER_ERR_ARB_LOST,
    I2C_ERR_CLK_TOUT = I2C_MASTER_ERR_CLK_TOUT
} I2CStatus_t;

typedef enum I2CCommand
{
    I2C_SINGLE_SEND = I2C_MASTER_CMD_SINGLE_SEND,
    I2C_SINGLE_RECEIVE = I2C_MASTER_CMD_SINGLE_RECEIVE,
    I2C_BURST_SEND_START = I2C_MASTER_CMD_BURST_SEND_START,
    I2C_BURST_SEND_CONT = I2C_MASTER_CMD_BURST_SEND_CONT,
    I2C_BURST_SEND_FINISH = I2C_MASTER_CMD_BURST_SEND_FINISH,
    I2C_BURST_SEND_STOP = I2C_MASTER_CMD_BURST_SEND_STOP,
    I2C_BURST_SEND_ERROR_STOP = I2C_MASTER_CMD_BURST_SEND_ERROR_STOP,
    I2C_BURST_RECEIVE_START = I2C_MASTER_CMD_BURST_RECEIVE_START,
    I2C_BURST_RECEIVE_CONT = I2C_MASTER_CMD_BURST_RECEIVE_CONT,
    I2C_BURST_RECEIVE_FINISH = I2C_MASTER_CMD_BURST_RECEIVE_FINISH,
    I2C_BURST_RECEIVE_ERROR_STOP = I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP
} I2CCommand_t;

#endif /* I2C_DEFS_H_ */