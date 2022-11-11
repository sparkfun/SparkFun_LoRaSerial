@echo off
if [%1]==[] goto usage

@echo Programming for SparkFun LoRaSerial
@pause
:loop

@echo -
@echo Programming binary: %1

@echo Unlock bootloader
atprogram.exe -t atmelice -i swd -cl 20mhz -d atsamd21g18a write --fuses --offset 0x804000 --values 0xFFC7E0D8

@echo Programming firmware
atprogram.exe -t atmelice -i swd -cl 20mhz -d atsamd21g18a chiperase program -f %1 --verify

@echo Lock bootloader
atprogram.exe -t atmelice -i swd -cl 20mhz -d atsamd21g18a write --fuses --offset 0x804000 --values 0xFAC7E0D8

@echo Done programming! Ready for next board.
@pause

goto loop

:usage
@echo Missing the hex file. Ex: batch_program.bat LoRaSerial_Firmware_v10_Combined.hex