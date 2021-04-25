/*

Library-ified version of the Arduino as ISP Script

Based on the ArduinoISP sketch distributed with
Ardino IDE v1.8.13

Original file Copyright (c) 2008-2011 Randall Bohn
If you require a license, see
http://www.opensource.org/licenses/bsd-license.php

Modified by @UnfinishedStuff 2021


Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#ifndef ArduinoAsISP_h
#define ArduinoAsISP_h

#include "Arduino.h"
#include "SPI.h"

class ArduinoAsISP
{

  public:
    ArduinoAsISP();
    void pulse(int pin, int times);
    void setup();
    void heartbeat();
    void reset_target(bool reset);
    void runloop(void);
    uint8_t getch();
    void fill(int n);
    void prog_lamp(int state);
    uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
    void empty_reply();
    void breply(uint8_t b);
    void get_version(uint8_t c);
    void set_parameters();
    void start_pmode();
    void end_pmode();
    void universal();
    void flash(uint8_t hilo, unsigned int addr, uint8_t data);
    void commit(unsigned int addr);
    unsigned int current_page();
    void write_flash(int length);
    uint8_t write_flash_pages(int length);
    uint8_t write_eeprom(unsigned int length);
    uint8_t write_eeprom_chunk(unsigned int start, unsigned int length);
    void program_page();
    uint8_t flash_read(uint8_t hilo, unsigned int addr);
    char flash_read_page(int length);
    char eeprom_read_page(int length);
    void read_page();
    void read_signature();
    void avrisp();
  private:
    #define PROG_FLICKER true
    #define SPI_CLOCK 		(1000000/6)
    #define RESET     	7
    #define LED_HB    	4
    #define LED_ERR   	6
    #define LED_PMODE 	5

    #define PIN_MOSI 	10  //MOSI Phys11 D10
    #define PIN_MISO 	9  //MISO Phys10 D9
    #define PIN_SCK 	8  //SCK Phys12 D8

    #define SERIAL Serial

    #define BAUDRATE	19200

    #define HWVER 2
    #define SWMAJ 1
    #define SWMIN 18

    #define STK_OK      0x10
    #define STK_FAILED  0x11
    #define STK_UNKNOWN 0x12
    #define STK_INSYNC  0x14
    #define STK_NOSYNC  0x15
    #define CRC_EOP     0x20 //ok it is a space...

    #define beget16(addr) (*addr * 256 + *(addr+1))

    #define PTIME 30

    #define EECHUNK (32)

    int error = 0;
    int pmode = 0;

    unsigned int here;
    uint8_t buff[256]; //Global block storage
    typedef struct param{
      uint8_t devicecode;
      uint8_t revision;
      uint8_t progtype;
      uint8_t parmode;
      uint8_t polling;
      uint8_t selftimed;
      uint8_t lockbytes;
      uint8_t fusebytes;
      uint8_t flashpoll;
      uint16_t eeprompoll;
      uint16_t pagesize;
      uint16_t eepromsize;
      uint32_t flashsize;
    }
    parameter;

    parameter param;

    uint8_t hbval = 128;
    int8_t hbdelta = 8;

    bool rst_active_high;

};

#endif

