/*
NMEA2000_esp32.h

Copyright (c) 2015-2020 Timo Lappalainen, Kave Oy, www.kave.fi
Copyright (c) 2022 Karl Andersson

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Inherited object for ESP32 modules that has a TWAI driver fr use with NMEA2000 library. See also NMEA2000 library.

The library sets as default CAN Tx pin to GPIO 16 and CAN Rx pint to GPIO 4. If you
want to use other pins (I have not tested can any pins be used), add defines e.g.
#define ESP32_CAN_TX_PIN GPIO_NUM_34
#define ESP32_CAN_RX_PIN GPIO_NUM_35
before including NMEA2000_esp32.h or NMEA2000_CAN.h
*/

#ifndef _NMEA2000_ESP32_H_
#define _NMEA2000_ESP32_H_

#include "NMEA2000.h"
#include "N2kMsg.h"

#include "driver/gpio.h"

#ifndef ESP32_CAN_TX_PIN
#define ESP32_CAN_TX_PIN GPIO_NUM_16
#endif
#ifndef ESP32_CAN_RX_PIN
#define ESP32_CAN_RX_PIN GPIO_NUM_4
#endif

class tNMEA2000_esp32 : public tNMEA2000
{
private:
  bool IsOpen;
  static bool CanInUse;

protected:
  gpio_num_t TxPin;
  gpio_num_t RxPin;

protected:
  void CAN_init();

public:
  bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent = true);
  bool CANOpen();
  bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf);
  virtual void InitCANFrameBuffers();

public:
  tNMEA2000_esp32(gpio_num_t _TxPin = ESP32_CAN_TX_PIN, gpio_num_t _RxPin = ESP32_CAN_RX_PIN);
};

#endif
