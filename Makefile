
CC= "C:\Program Files\Atmel\Atmel Studio 6.0\extensions\Atmel\AVRGCC\3.4.1.95\AVRToolchain\bin\avr-gcc.exe"
CCHEX= "C:\Program Files\Atmel\Atmel Studio 6.0\extensions\Atmel\AVRGCC\3.4.1.95\AVRToolchain\bin\avr-gcc.exe"
CFLAGS=-funsigned-char -funsigned-bitfields -Os -fpack-struct -fshort-enums -Wall -c -std=gnu99 -MD -MP -mmcu=atmega168pa 
TARGET=kkdrone.hex

${TARGET}: kkdrone.elf
${CCHEX} -O ihex -R .eeprom -R .fuse -R .lock -R .signature  "kkdrone.elf" "kkdrone.hex"
 

kkdrone.elf: ADC/ADC.o CONFIG/config.o EEPROM/EEPROM.o ISR/ISR.o REGUL/regulation.o kkdrone.o
   ${CC} -o kkdrone.elf  ADC/ADC.o CONFIG/config.o EEPROM/EEPROM.o ISR/ISR.o kkdrone.o REGUL/regulation.o   -Wl,-Map="kkdrone.map" -Wl,--start-group -Wl,-lm  -Wl,--end-group  -mmcu=atmega168pa  


ADC/ADC.o: ../ADC/ADC.c
   ${CC} $(CFLAGS) -MF "ADC/ADC.d" -MT"ADC/ADC.d" -MT"ADC/ADC.o" -o"ADC/ADC.o" "../ADC/ADC.c" 
   
CONFIG/config.o: ../CONFIG/config.c
   ${CC} $(CFLAGS) -MF "CONFIG/config.d" -MT"CONFIG/config.d" -MT"CONFIG/config.o" -o"CONFIG/config.o" "../CONFIG/config.c"
   
EEPROM/EEPROM.o: ../EEPROM/EEPROM.c
   ${CC} $(CFLAGS) -MF "EEPROM/EEPROM.d" -MT"EEPROM/EEPROM.d" -MT"EEPROM/EEPROM.o" -o"EEPROM/EEPROM.o" "../EEPROM/EEPROM.c" 
   
ISR/ISR.o: ../ISR/ISR.c
   ${CC} $(CFLAGS) -MF "ISR/ISR.d" -MT"ISR/ISR.d" -MT"ISR/ISR.o" -o"ISR/ISR.o" "../ISR/ISR.c"
  
REGUL/regulation.o: ../REGUL/regulation.c
   ${CC} $(CFLAGS) -MF "REGUL/regulation.d" -MT"REGUL/regulation.d" -MT"REGUL/regulation.o" -o"REGUL/regulation.o" "../REGUL/regulation.c"
   
kkdrone.o: ../kkdrone.c
   ${CC} $(CFLAGS) -MF "kkdrone.d" -MT"kkdrone.d" -MT"kkdrone.o" -o"kkdrone.o" "../kkdrone.c"  
    
