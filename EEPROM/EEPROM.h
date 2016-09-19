/*
 * EEPROM.h
 *
 * Created: 02/02/2013 19:47:56
 * Author: Alliaume Rico
 */ 


#ifndef EEPROM_H_
#define EEPROM_H_

// Include
#include <avr/eeprom.h>
#include "../kkdrone.h"

// Prototype
void Initial_EEPROM_Config_Load(void);
void Save_Config_to_EEPROM(void);
void Set_EEPROM_Default_Config(void);

void eeprom_write_byte_changed( uint8_t *  addr, uint8_t value );
void eeprom_write_block_changes( const uint8_t * src, void * dest, size_t size );


#endif /* EEPROM_H_ */