# Training

Training is the process of passing a radio's set of parameters to other radios so that they can successfully communicate.

There are two ways to initiate training:

![Training button on LoRaSerial](Original/SparkFun_LoRaSerial_-_Train.png)

* Pressing the **Train** button on the end of the LoRaSerial device
* Entering the **ATT** command

Training is performed by a server radio sending parameters to the client radio. Upon reception, the client radio saves the parameters in non-volatile memory and then reboots with the new parameters.

Pressing and releasing the training button for different lengths of time initiates the following behaviors:

## Training as Client

Press and hold the training button for 2 seconds. When the cylon pattern appears, release the button. The device is now ready to receive settings from a Server. Once settings are received from a server, the device saves the parameters to NVM and then reboots with the new settings.

Press and hold the training button for an additional 2 seconds to exit the Training as Client mode. No settings will be modified. The radio will return to normal operation.

## Training as Server

Press and hold the training button for 5 seconds. When the yellow LED flashes three times you know you've held the button long enough. In this mode, the radio will respond to any training client with its settings. The radio will not exit this mode unless it is powered cycled or the **ATZ** reset command is issued.

This mode (Training as Server) is convenient when multiple clients need to be paired to a server. For example, when a device is configured to be a Server in Multipoint mode, putting it in *Training as Server* mode will allow a user to put another radio into *Training as Client*. The radios will communicate and the Server radio will configure the client radio. Any number of client radios can be quickly and conveniently configured.

### Point-To-Point Training as Server

If a radio is set for P2P mode, entering training mode as a server will generate a new random NetID and AES key. This is helpful in the field if you need to put a pair of radios onto a different network and encryption key from other radios that may be nearby.

## Factory Reset

Press and hold the training button for 15 seconds to enter Factory Reset mode. You know you've held the Training button long enough when the blue LED flashes 3 times. Releasing the button will then return LoRaSerial to factory settings and erase NVM.

## Temporary Training Server

It is possible to use temporary training servers for multipoint and virtual circuit modes. The only difference between a temporary server and a regular server is that the parameters are not saved to the non-volatile storage using the ATW command before exiting command mode.

## Example: Training a Batch of Radios

To get a known set of parameters onto a network of radios, choose one radio to be the server. Enter command mode on the chosen server radio and set the radio's parameters as desired. Once complete, save the settings to NVM (**ATW** command) then enter training mode (**ATT**). From here, you may either use the Training buttons on the Client radios or you may use AT commands.

Server:

* Enter command mode with **+++**
* Optionally, start from factory defaults with **ATF**
* Set the desired parameters
* Set AT-Server=1
* Record settings with **ATW**
* Enter training mode with **ATT**
* Have other radios enter into Training as Client mode
* Power cycle Server radio when clients are trained

Clients:

* Enter command mode with **+++**
* Enter training mode with **ATT**
* The client will now obtain settings from the Server
* Settings saved to NVM
* Radio reboots

Alternatively, the Client can enter training via the button:

* Hold the training button for 2 seconds
* The client will now obtain settings from the Server
* Settings saved to NVM
* Radio reboots

## Virtual Circuit Training

It is recommended to use a command script to initialize the server radio when performing virtual-circuit training. The client radios can either use command mode or the training button to enter training mode.

Temporary Server (Doesn't save settings):

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

# Training Parameters

The training parameters for radio communication fall into two groups:

* Radio parameters
* Radio protocol parameters

Other optional sets of training parameters may be communicated between the radios when the corresponding copyXxxx parameter is set to true (1). The optional parameter sets are:

* Serial parameters
* Debug parameters
* Trigger parameters

For more information about the various radio settings, please see [AT Commands]().