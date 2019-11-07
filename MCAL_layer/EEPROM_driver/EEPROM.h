/*
 * EEPROM.h
 *
 *  Created on: Nov 22, 2018
 *      Author: Administrator
 */

#ifndef EEPROM_H_
#define EEPROM_H_

EEPROMstatus_t EEPROM_init(void);

EEPROMstatus_t EEPROM_readData(uint32_t ui32Address, void * const pvData,
                               uint32_t ui32DataSize);

EEPROMstatus_t EEPROM_writeData(uint32_t ui32Address, const void * const pvData,
                                uint32_t ui32DataSize);

bool EEPROM_isErased(uint32_t ui32Address, uint32_t ui32DataSize);

EEPROMstatus_t EEPROM_MassErase(void); /* fill 1's */

#endif /* EEPROM_H_ */
