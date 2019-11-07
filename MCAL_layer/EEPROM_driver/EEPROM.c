/*
 * EEPROM.c
 *
 *  Created on: Nov 22, 2018
 *      Author: Administrator
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "EEPROM_driver.h"

EEPROMstatus_t EEPROM_init(void)
{
    EEPROMstatus_t tEEPROMstatus;

    SysCtlPeripheralEnable(EEPROM_CLK_BASE);
    while (!SysCtlPeripheralReady(EEPROM_CLK_BASE))
    {
    }

    tEEPROMstatus = (EEPROMstatus_t) EEPROMInit();

    return tEEPROMstatus;
}

EEPROMstatus_t EEPROM_readData(uint32_t ui32Address, void * const pvData,
                               uint32_t ui32DataSize)
{
    EEPROMstatus_t tEEPROMstatus = EEPROM_SUCCESS;
    uint32_t ui32Remain, ui32SizeMultipleOf4, ui32AddressMultipleOf4,
            *pui32Buffer;

    ui32Remain = ui32DataSize % EEPROM_WORD_SIZE;
    if (ui32Remain != 0)
    {
        ui32SizeMultipleOf4 = ui32DataSize + (EEPROM_WORD_SIZE - ui32Remain);
    }
    else
    {
        ui32SizeMultipleOf4 = ui32DataSize;
    }

    ui32Remain = ui32Address % EEPROM_WORD_SIZE;
    if (ui32Remain != 0)
    {
        ui32AddressMultipleOf4 = ui32Address + (EEPROM_WORD_SIZE - ui32Remain);
    }
    else
    {
        ui32AddressMultipleOf4 = ui32Address;
    }

    pui32Buffer = (uint32_t *) calloc((ui32SizeMultipleOf4 / EEPROM_WORD_SIZE),
    EEPROM_WORD_SIZE);

    EEPROMRead(pui32Buffer, ui32AddressMultipleOf4, ui32SizeMultipleOf4);

    memcpy(pvData, pui32Buffer, ui32DataSize);

    free(pui32Buffer);

    return tEEPROMstatus;
}

EEPROMstatus_t EEPROM_writeData(uint32_t ui32Address, const void * const pvData,
                                uint32_t ui32DataSize)
{
    EEPROMstatus_t tEEPROMstatus = EEPROM_SUCCESS;
    uint32_t ui32Remain, ui32SizeMultipleOf4, ui32AddressMultipleOf4,
            *pui32Buffer;

    ui32Remain = ui32DataSize % EEPROM_WORD_SIZE;
    if (ui32Remain != 0)
    {
        ui32SizeMultipleOf4 = ui32DataSize + (EEPROM_WORD_SIZE - ui32Remain);
    }
    else
    {
        ui32SizeMultipleOf4 = ui32DataSize;
    }

    ui32Remain = ui32Address % EEPROM_WORD_SIZE;
    if (ui32Remain != 0)
    {
        ui32AddressMultipleOf4 = ui32Address + (EEPROM_WORD_SIZE - ui32Remain);
    }
    else
    {
        ui32AddressMultipleOf4 = ui32Address;
    }

    pui32Buffer = (uint32_t *) calloc((ui32SizeMultipleOf4 / EEPROM_WORD_SIZE),
    EEPROM_WORD_SIZE);

    memcpy(pui32Buffer, pvData, ui32DataSize);

    tEEPROMstatus = (EEPROMstatus_t) EEPROMProgram(pui32Buffer,
                                                   ui32AddressMultipleOf4,
                                                   ui32SizeMultipleOf4);
    free(pui32Buffer);

    return tEEPROMstatus;
}

bool EEPROM_isErased(uint32_t ui32Address, uint32_t ui32DataSize)
{
    bool bIsErased = true;
    uint32_t ui32Remain, ui32SizeMultipleOf4, ui32AddressMultipleOf4,
            *pui32Buffer, i;

    ui32Remain = ui32DataSize % EEPROM_WORD_SIZE;
    if (ui32Remain != 0)
    {
        ui32SizeMultipleOf4 = ui32DataSize + (EEPROM_WORD_SIZE - ui32Remain);
    }
    else
    {
        ui32SizeMultipleOf4 = ui32DataSize;
    }

    ui32Remain = ui32Address % EEPROM_WORD_SIZE;
    if (ui32Remain != 0)
    {
        ui32AddressMultipleOf4 = ui32Address + (EEPROM_WORD_SIZE - ui32Remain);
    }
    else
    {
        ui32AddressMultipleOf4 = ui32Address;
    }

    pui32Buffer = (uint32_t *) calloc((ui32SizeMultipleOf4 / EEPROM_WORD_SIZE),
    EEPROM_WORD_SIZE);

    EEPROMRead(pui32Buffer, ui32AddressMultipleOf4, ui32SizeMultipleOf4);

    for (i = 0; i < ui32SizeMultipleOf4; i += EEPROM_WORD_SIZE)
    {
        if (pui32Buffer[i] != 0xFFFFFFFFU)
        {
            bIsErased = false;
            break;
        }
    }

    free(pui32Buffer);

    return bIsErased;
}

EEPROMstatus_t EEPROM_MassErase(void)
{
    EEPROMstatus_t tEEPROMstatus;

    tEEPROMstatus = (EEPROMstatus_t) EEPROMMassErase();

    return tEEPROMstatus;

}
