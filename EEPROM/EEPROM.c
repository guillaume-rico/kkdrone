/*
 * EEPROM.c
 *
 * Created: 02/02/2013 19:48:09
 * Author: Alliaume Rico
 */ 

#include "EEPROM.h"

/*
 * \fn void eeprom_write_byte_changed( uint8_t * addr, uint8_t value )
 * \brief Change a byte in EEPROM
 * \param[in] addr
 * \param[in] value 
 */
void eeprom_write_byte_changed( uint8_t * addr, uint8_t value )
{
  if(eeprom_read_byte(addr) != value)
  {
    eeprom_write_byte( addr, value );
  }
}

/*
 * \fn void eeprom_write_block_changes( const uint8_t * src, void * dest, size_t size )
 * \brief Write a block in memory
 * \param[in] src
 * \param[in] dest
 * \param[in] size
 */
void eeprom_write_block_changes( const uint8_t * src, void * dest, size_t size )
{
  size_t len;

  for(len=0;len<size;len++)
  {
    eeprom_write_byte_changed( dest,  *src );

    src++;
    dest++;
  }
}

/*
 * \fn void Initial_EEPROM_Config_Load(void)
 * \brief Load config
 */
void Initial_EEPROM_Config_Load(void)
{
  // load up last settings from EEPROM
  if(eeprom_read_byte((uint8_t*) EEPROM_DATA_START_POS )!=0x47)
  {
    Config.setup = 0x47;
    Set_EEPROM_Default_Config();
    // write to eeProm
    Save_Config_to_EEPROM();
  } else {
    // read eeprom
    eeprom_read_block(&Config, (void*) EEPROM_DATA_START_POS, sizeof(CONFIG_STRUCT));
  }
}

/*
 * \fn void void Set_EEPROM_Default_Config(void)
 * \brief Init config
 */
void Set_EEPROM_Default_Config(void)
{
  Config.RollGyroDirection     = GYRO_REVERSED;
  Config.PitchGyroDirection    = GYRO_NORMAL;
  Config.YawGyroDirection      = GYRO_NORMAL;

  Config.RxChannel1ZeroOffset  = 1520;
  Config.RxChannel2ZeroOffset  = 1520;
  Config.RxChannel3ZeroOffset  = 1120;
  Config.RxChannel4ZeroOffset  = 1520;
}

/*
 * \fn void Save_Config_to_EEPROM(void)
 * \brief Save config in EEPROM
 */
void Save_Config_to_EEPROM(void)
{
  // write to eeProm
  cli();
  eeprom_write_block_changes( (const void*) &Config, (void*) EEPROM_DATA_START_POS, sizeof(CONFIG_STRUCT));  //current_config CONFIG_STRUCT
  sei();
}
