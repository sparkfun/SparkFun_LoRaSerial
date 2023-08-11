const uint8_t pin_blue_LED = 31;

const uint8_t pin_cts = 30;
const uint8_t pin_rts = 38;

#define INVERT_RTS_CTS      0

#include "Arch_SAMD.h"

// Initial entrypoint following any runtime library initialization
void setup()
{
    char * action[] = {"None", "Reset", "Interrupt", "Reserved"};
    uint32_t value;

    samdBeginSerial(57600);
    samdBeginBoard();
    randomSeed(5);

    //Enable the flow of data from the remote serial port into the SAMD
    updateRTS(true);


    // Read the brown-out-detector (BOD33)
    SerialUSB.print("BOD33: ");
    value = *(uint32_t *)0x40000834;
    SerialUSB.println(value, HEX);
//    SerialUSB.println();

    SerialUSB.print("    Level: ");
    SerialUSB.print(1.5 + (((value >> 16) & 0x3f) * 0.0354));
    SerialUSB.println(" Volts");

    SerialUSB.print("    Prescaler Select: divide by ");
    SerialUSB.print(2 << (value >> 12 & 0xf));
    SerialUSB.println(" Clocks");

    SerialUSB.print("    Clock: ");
    SerialUSB.println(value & 0x200 ? "Stable" : "Stopped");

    SerialUSB.print("    Mode: ");
    SerialUSB.println(value & 0x100 ? "Sampling" : "Continuous");

    SerialUSB.print("    Run in standby sleep mode: ");
    SerialUSB.println(value & 0x40 ? "Enabled" : "Disabled");

    SerialUSB.print("    Action: ");
    SerialUSB.println(action[(value >> 3) & 3]);

    SerialUSB.print("    Enabled: ");
    SerialUSB.println(value & 2 ? "Yes" : "NO");
}

// Idle loop for the CPU
void loop()
{
    static uint32_t lastToggle;

    if ((millis() - lastToggle) >= 100)
    {
        lastToggle = millis();
        digitalWrite(pin_blue_LED, !digitalRead(pin_blue_LED));
    }
}

//Returns true if CTS is asserted (host says it's ok to send data)
void updateRTS(bool assertRTS)
{
    digitalWrite(pin_rts, assertRTS ^ INVERT_RTS_CTS);
}
