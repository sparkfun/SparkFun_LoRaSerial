@echo off

if [%1]==[] goto usage

SET /A a = 0

rem Because the device is USB, the communication rate does not matter.
mode COM29 BAUD=57600 PARITY=n DATA=8

:start

echo The soldering iron is hot! %a% > \\.\%1
echo The soldering iron is hot! %a%

set /A a = a + 1

rem Wait a second
timeout /t 1

goto start

:usage
@echo Missing the COM port argument. Ex: sendSerial.bat COM29