Programming ATtiny84x in Circuit
===========================================================

The Qwiic Twist uses an ATtiny84A that must be programmed with firmware. This is done with spring pins aka Pogo Pins and an AVR programmer of your choice. We've had success with using an [Arduino as the ISP programmer](https://www.arduino.cc/en/Tutorial/BuiltInExamples/ArduinoISP) but we use the [Tiny AVR Programmer](https://www.sparkfun.com/products/11801) for day to day work.

1) Get the Tiny AVR Programmer and the [necessary drivers](https://learn.sparkfun.com/tutorials/tiny-avr-programmer-hookup-guide#driver-installation) installed. 
2) Clear the VCC solder jumper from the Tiny AVR Programmer. This jumper is closed by default and will power the target at 5V. While 5V will not harm the ATtiny targets, but many Qwiic boards have 3.3V sensitive sensors and peripherals so it's best to power the target through a Qwiic cable.
3) Solder 6-pin [female header](https://www.sparkfun.com/products/115) to the bottom of the programmer
4) Assemble the [SparkFun ISP Pogo Adapter](https://www.sparkfun.com/products/11591)
5) Attach the pogo adapter to the bottom of the programmer.

![Programming ISP via Pogo Pins on ATtiny84A](https://github.com/sparkfun/Qwiic_Twist/blob/master/Programming/Images/ATtiny%20Pogo%20ISP%20-%202.jpg?raw=true)


6) Identify and mark pin 1 on the bottom of the adapter. You'll need to know where pin 1 is to align it on the target ISP holes.

![Programming ISP via Pogo Pins on ATtiny84A](https://github.com/sparkfun/Qwiic_Twist/blob/master/Programming/Images/ATtiny%20Pogo%20ISP%20-%203.jpg?raw=true)

7) Attach Tiny AVR Programmer to a USB extension cable to allow free movement of the programmer. Confirm that avrdude can see the programmer.
8) Connect the target device to a development board using a Qwiic cable so that it is powered.
9) Download the contents of the 'Programming' folder so that you have **avrdude.exe**, **avrdude.conf**, and **program_qwiic_twist.bat** in a folder.
9) Open a command line, navigate to 'program_qwiic_twist.bat' and run. It should fail because no ATtiny was detected. If it failed because an attinyusb was not found then you may need to troubleshoot your Tiny AVR Programmer and/or edit the batch file to match your programmer type.
10) Press and hold the pogo pins against the 6 exposed pins on the target. Your mark on the pogo pins should match with the small white silkscreen on the ISP on the target. You may need to wiggle the Pogo adapter until you hear all 6 pins seat into the ISP footprint.

![Programming ISP via Pogo Pins on ATtiny84A](https://raw.githubusercontent.com/sparkfun/Qwiic_Twist/master/Programming/Images/ATtiny%20Pogo%20ISP%20-%201.jpg)


11) Press enter with your free hand to begin programming.
![Programming ISP via Pogo Pins on ATtiny84A](https://github.com/sparkfun/Qwiic_Twist/blob/master/Programming/Images/avrdude%20output.jpg?raw=true)

