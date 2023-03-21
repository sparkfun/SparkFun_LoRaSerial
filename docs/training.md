# Training

Training is the process of passing a radio's set of parameters to other radios so that they can successfully communicate.

![Training button on LoRaSerial](Original/SparkFun_LoRaSerial_-_Train.png)

*The train button on the LoRaSerial*

There are two ways to initiate training:

* Pressing the **Train** button on the end of the LoRaSerial device
* Entering the **ATT** command

Training is performed by a server radio sending parameters to the client radio. Upon reception, the client radio saves the parameters in non-volatile memory and then reboots with the new parameters.

Pressing and releasing the training button for different lengths of time initiates the following behaviors.

A ball-point pen is best for pressing the training button. *Be gentle.* The training button can be damaged with too much force.

## Training as Client

Press and hold the training button until the yellow LED starts to blink then release the button. The cylon pattern appears on the green LEDs. The device is now ready to receive settings from a Server. Once settings are received from a server, the device saves the parameters to NVM and then reboots with the new settings.

If training needs to be canceled, press and release the training button before the yellow LED starts to blink to exit training mode. No settings will be modified. The radio will return to normal operation.

## Training as Server

Press and hold the training button until the blue LED starts to blink then release the button. The Cylon pattern appears on the green LEDs. In this mode, the radio will respond to any training client with its settings. The radio will not exit this mode unless it is powered cycled or the **ATZ** reset command is issued.

This mode (Training as Server) is convenient when multiple clients need to be paired to a server. For example, when a device is configured to be a Server in Multipoint mode, putting it in *Training as Server* mode will allow a user to put another radio into *Training as Client*. The radios will communicate and the Server radio will configure the client radio. Any number of client radios can be quickly and conveniently configured this way.

## Simple Point-To-Point Training

If a radio is set for P2P mode, entering training mode as a server will generate a new random NetID and AES key. In other words, pushing the Train button and releasing it after the blue LED flashes on one radio will cause that radio to generate new, unique/secure P2P settings. On the other radio, press the Train button and release it after the yellow LED flashes. The radios will be trained to the new AES key and NetID, and the radios will reset, then immediately link up. This is helpful in the field if you need to put a pair of radios onto a different network and encryption key from other radios that may be nearby.

## Factory Reset

Press and hold the training button for 10 seconds to Factory Reset the radio. Release the button when all the LEDs blink quickly three times during the reset.  Just before reset the NVM is erased and LoRaSerial returns to factory settings. Note: You can also return a radio to factory defaults using the **ATF** command.

## Temporary Training Server

It is possible to use a temporary training server for multipoint mode. The only difference between a temporary server and a regular server is that the parameters are not saved to the non-volatile storage of the server using the ATW command before exiting command mode.

## Example: Training a Batch of Radios

To get a known set of parameters onto a network of radios, choose one radio to be the server. Enter command mode on the chosen server radio and set the radio's parameters as desired. Once complete, save the settings to NVM (**ATW** command) then enter training mode (**ATT**). From here, you may either use the Training buttons on the Client radios or you may use AT commands. Here is the same scenario in step-by-step instructions:

Server:

* Enter command mode with **+++**
* Optionally, start from factory defaults with **ATF**
* Set the desired parameters
* Set AT-Server=1
* Record settings with **ATW**
* Enter training mode with **ATT**
* Have other radios enter into Training as Client mode
* Power cycle Server radio when clients are trained

Clients can enter training via the **ATT** command:

* Enter command mode with **+++**
* Enter training mode with **ATT**
* The client will now obtain settings from the Server
* Settings saved to NVM
* Radio reboots

Alternatively, the Client can enter training via the button:

* Press the training button and release it when the yellow LED
* The client will now obtain settings from the Server
* Settings saved to NVM
* Radio reboots

## Command Scripts

A command script is simply a text file containing the AT commands the user would like to program. For example, the following script will enter command mode, reset the device to defaults, set the radio to be Server, record settings to NVM, and then exit command mode.

    +++
    ATF
    AT-Server=1
    ATW
    ATO

Copying and pasting command scripts into a terminal program is an efficient way of configuring a radio.

## Virtual Circuit Training

It is recommended to use a command script to initialize the server radio when performing virtual-circuit training. The client radios can either use command mode or the training button to enter training mode.

Temporary Server (Doesn't save settings):

* If the radio is not already in point-to-point or multi-point mode, hold the training button down for 15 seconds
* Connect to the LoRaSerial radio via USB or the serial port
* Enter command mode with +++
* Start from factory defaults by issuing the ATF command
* Issue the following commands:
  * AT-OperatingMode=2
  * AT-Server=1
  * AT-SelectLedUse=2
  * ATG
* Set any of the other parameters
* Enter training mode with ATT command
* Wait for clients to be trained
* Exit command mode with ATO command or reboot with ATZ command

Server (Saves settings):

* If not already at factory reset, hold the training button down for 15 seconds
* Connect to the LoRaSerial radio via USB or the serial port
* Enter command mode with +++
* Start from factory defaults by issuing the ATF command
* Issue the following commands:
  * AT-OperatingMode=2
  * AT-Server=1
  * AT-SelectLedUse=2
  * ATG
* Set any of the other parameters
* Enter training mode with ATT command
* Wait for clients to be trained
* Save parameters with ATW command
* Always reboot with ATZ command

Client:

* 2 second training button press
* Receives training parameters from server
* Saves new parameters
* Reboots

Client:

* Connect to the LoRaSerial radio via USB or the serial port
* Enter command mode using +++
* Enter training using the ATT command
* Receives training parameters from server
* Saves new parameters
* Reboots

## Training Parameters

The training parameters for radio communication fall into two groups:

* Radio parameters
* Radio protocol parameters

Other optional sets of training parameters may be communicated between the radios when the corresponding copyXxxx parameter is set to true (1). The optional parameter sets are:

* Serial parameters
* Debug parameters
* Trigger parameters

For more information about the various radio settings, please see [AT Commands](http://docs.sparkfun.com/SparkFun_LoRaSerial/at_commands/).

## Security Considerations

**Summary:** We recommend training radios located next to each other, in a secure area with antennas removed, to limit the amount of RF emissions and potential security issues. If security is a great concern, do not use training. Instead, AT commands can be used to set the settings between radios including AES keys.

During training, the Server will broadcast sensitive information including AES keys. Additionally, all radios used in training will use the default, published training settings. These include the following important parameters:

* 14dBm lower power transmission
* Default training frequency and hop tables
* Default training AES key

By default, radios will use the lowest power transmission possible of 14dBm. This is to limit RF eavesdropping. While data is encrypted and frequency hopping is used, the key *is* publicly published, and the hop table could be deduced. In theory, the transmitted parameters, including link keys that are trained to the Client could be obtained. 

If desired, the AES key used during training can be assigned to all units either using training (note the security issue) or via AT commands. This allows future training to be secure.

