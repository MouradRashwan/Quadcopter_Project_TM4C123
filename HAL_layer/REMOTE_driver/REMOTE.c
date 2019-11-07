/*
 * REMOTE.c
 *
 *  Created on: Mar 25, 2019
 *      Author: Administrator
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "REMOTE_driver.h"

void REMOTE_init(const uint32_t ui32Buadrate)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))
    {
    }
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    GPIOPinConfigure(GPIO_PC6_U3RX);
    GPIOPinConfigure(GPIO_PC7_U3TX);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART3))
    {
    }
    UARTConfigSetExpClk(
            UART3_BASE, SysCtlClockGet(), ui32Buadrate,
            (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE | UART_CONFIG_WLEN_8));
    UARTEnable(UART3_BASE);
}

int32_t REMOTE_receiveByte(void)
{
    return UARTCharGetNonBlocking(UART3_BASE);
}

void REMOTE_transmitByte(uint8_t ui8Data)
{
    UARTCharPut(UART3_BASE, ui8Data);
}

void REMOTE_transmitData(void * pvData, uint32_t ui32DataLen)
{
    uint32_t i;

    for (i = 0; i < ui32DataLen; i++)
    {
        UARTCharPut(UART3_BASE, ((uint8_t *) pvData)[i]);
    }
}
