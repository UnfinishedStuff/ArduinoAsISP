# ArduinoAsISP
Library-ified version of the ArduinoISP sketch


This is a crudely library-ified version of the ArduinoISP sketch which comes as an example in Arduino IDE version 1.8.13, converted for use as a library rather than a stand-alone sketch.  This allows other programs to import it and run it alongside other code.

This is currently in a working state where:
  * The pins are hard-coded
  * The loop is essentially locking, i.e. once started you won't "break out of" the loop of behaving like an ISP

These currently aren't problems for me as my use-case is "do other stuff then act as an ISP forever", but I'll try to remember to take a look at them and make them more sensible when I get time (!).

# Installing

No fancy Arduino-IDE library installer I'm afraid, create a folder called `ArduinoAsISP` in your Arduino install's `libraries` folder, and save the `ArduinoAsISP.h` and `ArduinoAsISP.cpp` files there.  Then you should be able to import the library in the IDE and run it.

On Windows the `libraries` folder is located in `C:\Users\[username]\Documents\Arduino\`.

# Wiring

Please see the [official ArduinoISP tutorial](https://www.arduino.cc/en/pmwiki.php?n=Tutorial/ArduinoISP) for further information on how to wire the boards, and what the LEDs do.

*Essential: wiring on the programmer chip, corresponds to hardware SPI pins*
* RESET - Digital 7
* MOSI  - D10
* MISO  - D9
* SCK   - D8

*Optional: status and error LED pins for the programmer*
* LED_HB    - D4
* LED_ERR   - D6
* LED_PMODE - D5

# Example sketch

Tested in Arduino IDE v1.8.13, on a SAMD21G18 chip flashed with Seeed Studio's Seeeduino Xiao bootloader, and used to program an ATmega328p.

```C++
#include "ArduinoAsISP.h"

void setup() 
  {
  }

void loop() 
  {
  // Create an object of type ArduinoAsISP
  ArduinoAsISP programmer;
  // Run the ISP loop forever
  programmer.runloop();
  }

```
