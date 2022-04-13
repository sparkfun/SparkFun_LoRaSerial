@echo Programming SparkFun LoRaSerial

rem if [%1]==[] goto usage

:loop

@echo Programming binary: %1 on %2

mode COM36 BAUD=1200 PARITY=n DATA=8 dtr=on
set /p x="REALLY BIG TEST TO KEEP the port open for a smidgeREALLY BIG TEST TO KEEP the port open for a smidgeREALLY BIG TEST TO KEEP the port open for a smidgeREALLY BIG TEST TO KEEP the port open for a smidgeREALLY BIG TEST TO KEEP the port open for a smidgeREALLY BIG TEST TO KEEP the port open for a smidge" <nul >\\.\COM36
rem echo 1>\\.\COM36

bossac.exe -i -d --port=%2 -U true -i -e -w -v %1 -R

@echo Flashing bootloader and firmware...
rem @bossac.exe -a -i -d --port=%2 -U true -i -e -w -v %1 -R
rem bossac.exe -a -i -d --port=COM36 -U true -e -w -v LoRaSerial_Firmware_v01.bin -R
rem powershell works: $port= new-Object System.IO.Ports.SerialPort COM36,1200,None,8,one


@echo Done programming! Ready for next board.
rem @pause


rem goto loop

rem :usage
rem @echo Missing the binary file and com port arguments. Ex: batch_program.bat LoRaSerial_Firmware_v10.bin COM29