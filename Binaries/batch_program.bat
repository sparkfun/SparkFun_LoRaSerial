@echo Programming SparkFun LoRaSerial

rem if [%1]==[] goto usage

:loop

@echo Programming binary: %1 on %2

@echo Flashing bootloader and  firmware...
@bossac.exe -i -d --port=%2 -U true -i -e -w -v %1 -R 

@echo Done programming! Ready for next board.
rem @pause

rem goto loop

rem :usage
rem @echo Missing the binary file and com port arguments. Ex: batch_program.bat LoRaSerial_Firmware_v10.bin COM29