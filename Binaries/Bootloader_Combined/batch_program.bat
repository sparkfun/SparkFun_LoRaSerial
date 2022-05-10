@echo off
if [%1]==[] goto usage

@echo Programming for SparkFun LoRaSerial
@pause
:loop

@echo -
@echo Programming binary: %1

atprogram.exe -t atmelice -i swd -cl 20mhz -d atsamd21g18a chiperase program -f %1 --verify

@echo Done programming! Ready for next board.
@pause

goto loop

:usage
@echo Missing the hex file. Ex: batch_program.bat LoRaSerial_Firmware_v10_Combined.hex
