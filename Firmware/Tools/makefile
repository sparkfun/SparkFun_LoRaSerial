######################################################################
# makefile
#
# Builds the LoRaSerial support programs
######################################################################

##########
# Source files
##########

EXECUTABLES += Slow_Read
EXECUTABLES += VcServerTest

INCLUDES = settings.h ../LoRaSerial_Firmware/Virtual_Circuit_Protocol.h

COMMON_LIB = Common_Lib.a

LIB_OBJS = RadioV2.o Terminal.o

##########
# Buid tools and rules
##########

GCC = gcc
CFLAGS = -flto -O3 -Wpedantic -pedantic-errors -Wall -Wextra -Werror -Wno-unused-variable -Wno-unused-parameter
CC = $(GCC) $(CFLAGS)

%.o: %.c $(INCLUDES)
	$(CC) -c -o $@ $<

%: %.c $(INCLUDES)
	$(CC) $(CFLAGS) -o $@ $<

##########
# Buid all the sources - must be first
##########

.PHONY: all

all: $(EXECUTABLES)

##########
# Buid the libraries
##########

$(COMMON_LIB): $(LIB_OBJS)
	gcc --version
	ar rvs $@ $^

##########
# Build the executables
##########

Slow_Read: Slow_Read.c $(COMMON_LIB)
	$(CC) -o $@ $^

VcServerTest: VcServerTest.c $(COMMON_LIB)
	$(CC) -o $@ $^

##########
# Build the firmware
##########

LoRaSerial:	../LoRaSerial/LoRaSerial.ino ../LoRaSerial/*.ino ../LoRaSerial/*.h
	~/Arduino/arduino-cli compile --fqbn SparkFun:samd:LoRaSerial ../LoRaSerial/LoRaSerial.ino --export-binaries -v

########
# Clean the build directory
##########

.PHONY: clean

clean:
	rm -f *.o *.a $(EXECUTABLES)
