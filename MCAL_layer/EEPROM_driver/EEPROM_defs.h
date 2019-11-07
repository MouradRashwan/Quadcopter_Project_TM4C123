/*
 * EEPROM_defs.h
 *
 *  Created on: Nov 22, 2018
 *      Author: Administrator
 */

#ifndef EEPROM_DEFS_H_
#define EEPROM_DEFS_H_

#define EEPROM_WORD_SIZE        4U /* bytes */
#define EEPROM_CLK_BASE         SYSCTL_PERIPH_EEPROM0

typedef enum EEPROMstatus
{
    EEPROM_SUCCESS = 0, EEPROM_ERROR = 1
} EEPROMstatus_t;

#endif /* EEPROM_DEFS_H_ */
