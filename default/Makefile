###############################################################################
# Makefile for the project ExcavatorControlUnit
###############################################################################

## General Flags
PROJECT = ExcavatorControlUnit
MCU = at90can128
TARGET = ExcavatorControlUnit.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99                -DF_CPU=16000000UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=ExcavatorControlUnit.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Include Directories
INCLUDES = -I"C:\ExcavatorControlUnit\ExcavatorControlUnit\." -I"C:\ExcavatorControlUnit\ExcavatorControlUnit\uartlib" -I"C:\ExcavatorControlUnit\ExcavatorControlUnit\avr-can-lib" -I"C:\ExcavatorControlUnit\ExcavatorControlUnit\avr-can-lib\src" 

## Libraries
LIBS = -lm -lc 

## Objects that must be built in order to link
OBJECTS = initializations.o kinematics.o pid.o main.o waypoint.o uart.o resolvers.o valves.o at90can.o at90can_buffer.o at90can_disable_dyn_filter.o at90can_error_register.o at90can_get_buf_message.o at90can_get_dyn_filter.o at90can_get_message.o at90can_send_buf_message.o at90can_send_message.o at90can_set_dyn_filter.o at90can_set_mode.o can_buffer.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) ExcavatorControlUnit.hex ExcavatorControlUnit.eep ExcavatorControlUnit.lss size

## Compile
initializations.o: ../initializations.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

kinematics.o: ../kinematics.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

pid.o: ../pid.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

main.o: ../main.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

waypoint.o: ../waypoint.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

uart.o: ../uartlib/uart.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

resolvers.o: ../resolvers.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

valves.o: ../valves.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

at90can.o: ../avr-can-lib/src/at90can.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

at90can_buffer.o: ../avr-can-lib/src/at90can_buffer.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

at90can_disable_dyn_filter.o: ../avr-can-lib/src/at90can_disable_dyn_filter.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

at90can_error_register.o: ../avr-can-lib/src/at90can_error_register.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

at90can_get_buf_message.o: ../avr-can-lib/src/at90can_get_buf_message.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

at90can_get_dyn_filter.o: ../avr-can-lib/src/at90can_get_dyn_filter.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

at90can_get_message.o: ../avr-can-lib/src/at90can_get_message.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

at90can_send_buf_message.o: ../avr-can-lib/src/at90can_send_buf_message.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

at90can_send_message.o: ../avr-can-lib/src/at90can_send_message.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

at90can_set_dyn_filter.o: ../avr-can-lib/src/at90can_set_dyn_filter.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

at90can_set_mode.o: ../avr-can-lib/src/at90can_set_mode.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

can_buffer.o: ../avr-can-lib/src/can_buffer.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) ExcavatorControlUnit.elf dep/* ExcavatorControlUnit.hex ExcavatorControlUnit.eep ExcavatorControlUnit.lss ExcavatorControlUnit.map


## Other dependencies
-include $(shell mkdir dep 2>NUL) $(wildcard dep/*)

