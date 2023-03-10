# Building LoRaSerial Firmware

The open source [LoRaSerial firmware](https://github.com/sparkfun/SparkFun_LoRaSerial) builds under Windows and Linux.  Use the following procedures to setup the environment and perform the build:

* [Windows](http://docs.sparkfun.com/SparkFun_LoRaSerial/firmware_build/#Windows)
* [Linux](http://docs.sparkfun.com/SparkFun_LoRaSerial/firmware_build/#Linux)

# Virtual Machine

Execute the following commands in the [VirualBox](https://www.virtualbox.org/) application to create a virtual machine:

1. Click on the Machine menu item
2. Click on the New submenu item
3. Specify the machine Name, e.g.: Sparkfun_LoRaSerial_22.04.1
4. Select Type, e.g.: Linux
5. Select Version, e.g.: Ubuntu (64-bit)
6. Click the Next> button
7. Select the memory size: 7168
8. Click the Next> button
9. Click on Create a virtual hard disk now
10. Click the Create button
11. Select VDI (VirtualBox Disk Image)
12. Click the Next> button
13. Select Dynamically allocated
14. Click the Next> button
15. Select the disk size: 128 GB
16. Click the Create button
17. Click on the Settings button
18. Click on Storage
19. Click the empty CD icon
20. On the right-hand side, click the CD icon
21. Click on Choose a disk file...
An operating system is needed for the virtual machine to enable it to run applications.  Download one of the following operating system images (.iso files):

* [Ubuntu Desktop (Linux)](https://ubuntu.com/download/desktop)
* Windows

22. Choose one of the .iso files downloaded above
23. Click the Open button
24. Click on Network
25. Under 'Attached to:' select Bridged Adapter
26. Click the OK button
27. Click the Start button
28. Install the operating system
29. Log into the operating system

* For Ubuntu:
    1. Click on Activities
    2. Type terminal into the search box
    3. In the terminal window
        a. sudo apt install -y net-tools openssh-server
        b. ifconfig

        Write down the IP address

    4. On the PC (Linux) in a terminal window
        a. ssh-keygen -t rsa -f ~/.ssh/SparkFun_LoRaSerial_22.04.1
        b. ssh-copy-id -o IdentitiesOnly=yes -i ~/.ssh/SparkFun_LoRaSerial_22.04.1  &lt;username&gt;@&lt;IP address&gt;
        c. ssh -Y &lt;username&gt;@&lt;IP address&gt;

## Ubuntu Build Environment

The following setup and build instructions were tested on Ubuntu 22.04.1.  The build can run on a physical or virtual CPU.  The following section describes how to setup the a virtual machine.  Further sections describe the build environment and how to perform the build.

Execute the following commands to create the build environment for the SparkFun LoRaSerial Firmware:

1. sudo adduser $USER dialout
2. sudo shutdown -r 0

Reboot to ensure that the dialout privilege is available to the user

3. sudo apt update
4. sudo apt install -y  git  gitk  git-cola  minicom  python3-pip
5. sudo pip3 install pyserial
6. mkdir ~/SparkFun
7. cd ~/SparkFun
8. nano serial-57600.sh

Insert the following text into the file:

    #!/bin/bash
    #   serial-57600.sh
    #
    #   Shell script to read the serial data from the LoRaSerial USB port
    #
    #   Parameters:
    #       1:  ttyACMn
    #
    sudo minicom -b 57600 -8 -D /dev/$1 < /dev/tty

9. chmod +x serial-57600.sh

Get the SparkFun LoRaSerial Firmware sources

10. mkdir ~/SparkFun/LoRaSerial
11. cd ~/SparkFun/LoRaSerial
12. git clone [https://github.com/sparkfun/SparkFun_LoRaSerial](https://github.com/sparkfun/SparkFun_LoRaSerial) .

Install the Arduino IDE

13. mkdir ~/SparkFun/arduino
14. cd ~/SparkFun/arduino
15. wget https://downloads.arduino.cc/arduino-1.8.15-linux64.tar.xz
16. tar -xvf ./arduino-1.8.15-linux64.tar.xz
17. cd arduino-1.8.15/
18. sudo ./install.sh
19. arduino

# Setup the Arduino Build Environment

Install the ARM Cortex-M0+ tools.  See [SAMD21 MiniDev Hookup Guide](https://learn.sparkfun.com/tutorials/samd21-minidev-breakout-hookup-guide/setting-up-arduino) for more documentation.

1. Click on Tools in the menu bar
2. Click on Board
3. Click on Board Manager ...
4. Scroll down to Arduino SAMD Boards (32-bits ARM Cortex-M0+)
5. Click on Arduino SAMD Boards (32-bits ARM Cortex-M0+)
6. Select version 1.8.13
7. Click on the install button
8. Click on the close button

## Install the SparkFun Board Packages

9. Click on File in the menu bar
10. Click on Preferences
11. Go down to the Additional Boards Manager URLs text box
12. Only if the textbox already has a value, go to the end of the value or values and add a comma
13. Add the link: https://raw.githubusercontent.com/sparkfun/Arduino_Boards/main/IDE_Board_Manager/package_sparkfun_index.json
14. Note the value in Sketchbook location
15. Click the OK button
16. Click on Tools in the menu bar
17. Click on Board
18. Click on Board Manager ...
19. Scroll down and click on SparkFun SAMD Boards (dependency: Arduino SAMD Boards 1.8.1)
20. Select version 1.8.9
21. Click on the install button
22. Click on the close button
23. Click on Tools in the menu bar
24. Click on Board
25. Click on SparkFun SAMD (32-bits ARM Cortex-M0+) Boards
26. Click on SparkFun LoRaSerial

## Install the required libraries

The following procedure installs the libraries needed to successfully build the LoRaSerial firmware.

27. Click on Tools in the menu bar
28. Click on Manage Libraries...
29. For each library:

    a. Enter the library name into the search box
    b. Scroll down the list if necessary to locate the specified libray
    c. Click on the library
    d. Select the specified version
    e. Click on the install button

30. Install the following libraries:

    a. Crypto, v0.4.0
    a. FlashStorage_SAMD, v1.3.2
    a. JC_Button, v2.1.2
    a. RadioLib, v5.1.2
    a. SAMD_TimerInterrupt, v1.9.0

31. Click on File in the menu bar
32. Click on Quit

## External Libraries

The WDT library is not registered with the Arduino IDE.  It must be added outside of the Arduino IDE environment.  Install the external libraries adding them to the Sketchbook location noted above.

33. cd ~/Arduino/libraries
34. mkdir WDTZero
35. cd WDTZero/
36. git clone https://github.com/javos65/WDTZero.git .

# Build the LoRaSerial Firmware

Use the following procedure to get the project ready to build

1. arduino
2. Click on File in the menu bar
3. Click on Open
4. Select the project file: ~/SparkFun/LoRaSerial/Firmware/LoRaSerial/LoRaSerial.ino
5. Click on the Open button
6. Click on Tools in the menu bar
7. Verify that Board lists "SparkFun LoRaSerial"

## Connect the LoRaSerial Device

8. Plug the LoRaSerial device into a USB port on the PC
9. In the arduino program, click on Tools
10. Click on Ports
11. Select the port for the LoRaSerial device

## Compile

12. Click on Sketch in the menu bar
13. Click on Verify/Compile
14. Using an editor or the Arduino IDE ake any source file changes until the Sketch (program) compiles cleanly

## Upload the Firmware

15. Click on Sketch
16. Click on Upload

# Arduino CLI

The firmware can be compiled using [Arduino CLI](https://github.com/arduino/arduino-cli). This makes compilation fairly platform independent and flexible. All release candidates and firmware releases are compiled using Arduino CLI using a github action. You can see the source of the action [here](https://github.com/sparkfun/SparkFun_RTK_Firmware/blob/main/.github/workflows/compile-release.yml), and use it as a starting point for Arduino CLI compilation.
