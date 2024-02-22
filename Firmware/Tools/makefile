######################################################################
# makefile
#
# Builds the LoRaSerial support programs
######################################################################

##########
# Source files
##########

EXECUTABLES += Send_Receive
EXECUTABLES += Slow_Read
EXECUTABLES += VcServerTest

INCLUDES += settings.h
INCLUDES += Send_Receive.h
INCLUDES += ../LoRaSerial/Virtual_Circuit_Protocol.h

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

Send_Receive: Send_Receive.c $(COMMON_LIB)
	$(CC) -o $@ $^

Slow_Read: Slow_Read.c $(COMMON_LIB)
	$(CC) -o $@ $^

VcServerTest: VcServerTest.c $(COMMON_LIB)
	$(CC) -o $@ $^

##########
# Build the firmware
##########

libraries:
	echo ----------------------------------
	cd  ~/Arduino/libraries

	#          Crypto@0.4.0

	echo FlashStorage_SAMD
	cd   ../FlashStorage_SAMD
	git  checkout  --quiet  master
	git  pull  --quiet  origin  master
	git  checkout  --quiet  v1.3.2

	echo JC_Button
	cd   ../JC_Button
	git  checkout  --quiet  master
	git  pull  --quiet  origin  master
	git  checkout  --quiet  2.1.2

	echo RadioLib
	cd   ../RadioLib
	git  checkout  --quiet  master
	git  pull  --quiet  origin  master
	git  checkout  --quiet  5.6.0

	echo SAMD_TimerInterrupt
	cd   ../SAMD_TimerInterrupt
	git  checkout  --quiet  main
	git  pull  --quiet  origin  main
	git  checkout  --quiet  v1.9.0

	echo WDTZero
	cd   ../WDTZero
	git  checkout  --quiet  master
	git  pull  --quiet  origin  master

	echo ----------------------------------
	~/Arduino/arduino-cli lib list

LoRaSerial:	../LoRaSerial/LoRaSerial.ino ../LoRaSerial/*.ino ../LoRaSerial/*.h
	~/Arduino/arduino-cli compile --fqbn SparkFun:samd:LoRaSerial ../LoRaSerial/LoRaSerial.ino --export-binaries -v

########
# Clean the build directory
##########

.PHONY: clean

clean:
	rm -f *.o *.a $(EXECUTABLES)
