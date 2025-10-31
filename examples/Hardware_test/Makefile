KILOMBO_PATH := /home/bencelin/kilombo
ROOT         := $(KILOMBO_PATH)/examples/networkdesign_hardware/networkdesign
# These are for the simulator
#CFLAGS += -I$(KILOMBO_PATH)/src
#LDFLAGS += -L$(KILOMBO_PATH)/build/src -lsim -lSDL -ljansson -lm

# Make file for compiling a kilobot program, both for the
# kilobot simulator and for the real kilobot.
# The program can consist of multiplse files. Files and paths to various libraries are
# configured below, these needs to be adapted to the system.

# Real‐bot Kilobot library:
KILOHEADERS  := $(ROOT)/kilolib               # 🡐 this folder must contain kilolib.h
KILOLIB      := $(ROOT)/kilolib/build/kilolib.a # 🡐 this must be the .a file

$(KILOLIB):
	$(MAKE) -C $(KILOHEADERS)
#list of c files in this project
SOURCES=turnLeft.c 

#name of the executable file
EXECUTABLE=turnLeft

# Path to the official kilolib, used for real bots only
# Set it here, or as an environment variable in the shell:
# export KILOHEADERS=/your/path/  
#
# where the kilobot headers live
# where your hardware example lives



# path of kilombo.h distributed with the simulator but also needed
# when compiling the user program for the real kilobot (avr-gcc has different default paths)
#SIMHEADERS=/usr/local/include
# export KILOHEADERS=/path/to/kilobot/kilolib


#path to kilolib.a in the official kilolib, needed for real bots only
# KILOLIB    =$(KILOHEADERS)/build/kilolib.a

#compilation flags for simulated version
#SIM_CFLAGS = -c -g -O2 -Wall -std=c99  #-I$(KILOHEADERS)

#linking flags for simulated version
#SIM_LFLAGS = -lsim -lSDL -lm -ljansson

# linking flags to compile headless
# SIM_LFLAGS = -lheadless  -lm -ljansson


# Makefile targets.
# sim (default target) is the simulator program
# hex is the .hex file (program) for the real kilobot.
# all builds both.

# Note: the hex targer requires the AVR toolchain and
# the kilolib library to be installed. See simulator README.

sim: $(EXECUTABLE)
hex: $(EXECUTABLE).hex
all: sim hex

clean :
	rm *.o $(EXECUTABLE) *.elf *.hex

# # # # # # # # # # The following should be generic and not need changes # # # # # # # # # # # # # 

# compiling for the real bot
#

CC = avr-gcc
AVRAR = avr-ar
AVROC = avr-objcopy
AVROD = avr-objdump
AVRUP = avrdude

#PFLAGS = -P usb -c avrispmkII # user to reprogram OHC
# AVR toolchain flags
CFLAGS = -mmcu=atmega328p -DF_CPU=8000000UL -Wall -gdwarf-2 -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -I$(KILOHEADERS) -DKILOBOT -I$(KILOMBO_PATH)/src#-I$(KILOMBO_PATH)/src

#for a floating point printf
#CFLAGS += -Wl,-u,vfprintf -lprintf_flt -lm

ASFLAGS = $(CFLAGS)

FLASH = -R .eeprom -R .fuse -R .lock -R .signature
EEPROM = -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0  

%.lss: %.elf
	$(AVROD) -d -S $< > $@

%.hex: %.elf
	$(AVROC) -O ihex $(FLASH) $< $@

%.eep: %.elf
	$(AVROC) -O ihex $(EEPROM) $< $@

%.bin: %.elf
	$(AVROC) -O binary $(FLASH) $< $@ 

$(EXECUTABLE).elf: $(SOURCES) $(KILOLIB)
	$(CC) $(CFLAGS) -o $@ $< $(KILOLIB)
	@echo "------ AVR Memory Usage ------"
	@avr-size --format=avr --mcu=atmega328p $@
# make syntax:
# $@ left-hand side of :
# $^ right-hand side of :
# $< first item of right-hand side of :



# From here on, we compile for the simulator
# 

#which c compiler to use. gcc works well.
SIM_CC     := gcc
SIM_CFLAGS := -c -g -O2 -Wall -std=c99 \
              -I$(KILOMBO_PATH)/src
SIM_LDFLAGS:= -L$(KILOMBO_PATH)/build/src -lsim -lSDL -ljansson -lm

# compile .c → .o for simulator only
%.o: %.c
	$(SIM_CC) $(SIM_CFLAGS) $< -o $@

# link simulator executable
$(EXECUTABLE): $(OBJECTS)
	$(SIM_CC) $(SIM_LDFLAGS) -o $@ $(OBJECTS)
