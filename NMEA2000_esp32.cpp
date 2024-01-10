/*
NMEA2000_esp32.cpp

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

Inherited object for ESP32 modules that has a TWAI driver, for use with NMEA2000 library. See also NMEA2000 library.

*/

#include "driver/twai.h"
#include "NMEA2000_esp32.h"

// http://www.bittiming.can-wiki.info/ (Clock Rate set to 80Mhz)
// https://www.esacademy.com/en/library/calculators/sja1000-timing-calculator.html
// https://www.simmasoftware.com/j1939.html

// NMEA 2000 = SAE J1939-21
// Sample point should be as close to 87.5% but not past.
// SJW = 1

// #define TWAI_TIMING_CONFIG_NMEA2000() { .brp = 20, .tseg_1 = 13, .tseg_2 = 2, .sjw = 1, .triple_sampling = true }

#define TWAI_TIMING_CONFIG_NMEA2000() { .brp = 16, .tseg_1 = 16, .tseg_2 = 3, .sjw = 1, .triple_sampling = true }

bool tNMEA2000_esp32::CanInUse = false;
tNMEA2000_esp32 *pNMEA2000_esp32 = 0;

//*****************************************************************************
tNMEA2000_esp32::tNMEA2000_esp32(gpio_num_t _TxPin, gpio_num_t _RxPin) : tNMEA2000(), IsOpen(false), TxPin(_TxPin), RxPin(_RxPin)
{
}

//*****************************************************************************
bool tNMEA2000_esp32::CANOpen()
{
  if (IsOpen)
    return true;

  if (CanInUse)
    return false; // currently prevent accidental second instance. Maybe possible in future.

  pNMEA2000_esp32 = this;
  IsOpen = true;
  CAN_init();

  CanInUse = IsOpen;

  return IsOpen;
}

//*****************************************************************************
bool tNMEA2000_esp32::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool /*wait_sent*/)
{
  twai_message_t message;
  message.flags = 0;
  message.extd = 1;
  message.data_length_code = len > 8 ? 8 : len;
  message.dlc_non_comp = len > 8 ? 1 : 0;
  message.ss = 1;
  message.identifier = id;
  memcpy(message.data, buf, len);

  esp_err_t res = twai_transmit(&message, 0);

  // Queue message for transmission
  if (res == ESP_ERR_INVALID_STATE)
    twai_initiate_recovery();
  
  return res == ESP_OK;
}

//*****************************************************************************
bool tNMEA2000_esp32::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf)
{
  twai_message_t message;

  if (twai_receive(&message, 0) == ESP_OK && message.extd)
  {
    id = message.identifier;
    len = message.data_length_code;

    memcpy(buf, message.data, message.data_length_code);

    return true;
  }
  return false;
}

//*****************************************************************************
void tNMEA2000_esp32::CAN_init()
{
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TxPin, RxPin, TWAI_MODE_NORMAL);

  g_config.rx_queue_len = 50;
  g_config.tx_queue_len = 50;
  g_config.intr_flags = ESP_INTR_FLAG_LOWMED;

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_NMEA2000();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
  {
    return;
  }

  // Start TWAI driver
  twai_start();
}

//*****************************************************************************
void tNMEA2000_esp32::InitCANFrameBuffers()
{
  tNMEA2000::InitCANFrameBuffers(); // call main initialization
}
