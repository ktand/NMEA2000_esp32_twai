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
#include "driver/twai.h"
#include "esp_log_level.h"
#include "sdkconfig.h"

#ifndef ESP32_CAN_TX_PIN
#define ESP32_CAN_TX_PIN GPIO_NUM_16
#endif
#ifndef ESP32_CAN_RX_PIN
#define ESP32_CAN_RX_PIN GPIO_NUM_4
#endif
#ifndef ESP32_CAN_RX_TICKS_WAIT
#define ESP32_CAN_RX_TICKS_WAIT 0
#endif

// The TWAI driver's receive and transmit queues, in frames. A full bus
// (250 kbit/s) carries about 1800 frames a second, so 32 frames hold under
// 20 ms of it: longer than that between two reads and frames are lost.
#ifndef ESP32_CAN_RX_QUEUE_LEN
#define ESP32_CAN_RX_QUEUE_LEN 32
#endif
#ifndef ESP32_CAN_TX_QUEUE_LEN
#define ESP32_CAN_TX_QUEUE_LEN 32
#endif

#ifndef ESP32_CAN_STATISTICS
#define ESP32_CAN_STATISTICS 0
#endif

// #define ESP32_CAN_ISR_IN_IRAM

typedef void (*alerts_cb_t)(uint32_t alerts, bool is_error);

class tNMEA2000_esp32 : public tNMEA2000
{
  private:
    bool IsOpen;
    static bool CanInUse;

#if ESP32_CAN_STATISTICS == 1
    unsigned int RxBits = 0;
    unsigned int RxPackets = 0;

    unsigned int TxBits = 0;
    unsigned int TxPackets = 0;

    static void Timer_tick(void *arg);
#endif

  protected:
    gpio_num_t TxPin;
    gpio_num_t RxPin;

    TickType_t receive_wait_ticks;

    // TWAI driver queue lengths in frames (SetTWAIQueueLengths()).
    uint32_t rx_queue_len = ESP32_CAN_RX_QUEUE_LEN;
    uint32_t tx_queue_len = ESP32_CAN_TX_QUEUE_LEN;

    TaskHandle_t alert_task_handle;

    SemaphoreHandle_t alert_task_semaphore;
    alerts_cb_t alerts_callback = nullptr;
    // Per-frame logging in CANSendFrame()/CANGetFrame(), set by SetLogLevel():
    // a flag rather than esp_log_level_get() for every frame. Starts from the
    // default log level, as the per-frame check did.
    bool LogFrames = CONFIG_LOG_DEFAULT_LEVEL >= ESP_LOG_INFO;

    static const int ERROR_ALERTS_TO_WATCH = TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF | TWAI_ALERT_RX_FIFO_OVERRUN;
    static const int DATA_EVENTS_TO_WATCH = TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_RX_DATA;
    static const int ALERTS_TO_WATCH = ERROR_ALERTS_TO_WATCH | DATA_EVENTS_TO_WATCH;

#if ESP32_CAN_STATISTICS == 1
    unsigned int RxBitsPerSeconds = 0;
    unsigned int RxPacketsPerSecond = 0;

    unsigned int TxBitsPerSecond = 0;
    unsigned int TxPacketsPerSecond = 0;
#endif

  protected:
    void CAN_init();
    // The data alerts (a wakeup of the alert task for every frame) only for
    // an alerts callback; the error alerts always.
    uint32_t AlertsToWatch() const
    {
        return alerts_callback != nullptr ? ALERTS_TO_WATCH : ERROR_ALERTS_TO_WATCH;
    }

  public:
    tNMEA2000_esp32(gpio_num_t _TxPin = ESP32_CAN_TX_PIN, gpio_num_t _RxPin = ESP32_CAN_RX_PIN, TickType_t rxWaitTicks = ESP32_CAN_RX_TICKS_WAIT);

    bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent = true);
    bool CANOpen();
    bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf);

    virtual void InitCANFrameBuffers();

    void SetAlertsCallback(alerts_cb_t cb);

    void SetLogLevel(esp_log_level_t level);

    // Sets the TWAI driver's queue lengths in frames (defaults
    // ESP32_CAN_RX_QUEUE_LEN and ESP32_CAN_TX_QUEUE_LEN). Call before Open();
    // ignored once the driver is installed. Each frame takes about 20 bytes.
    void SetTWAIQueueLengths(uint32_t rxQueueLen, uint32_t txQueueLen);
    uint32_t GetTWAIRxQueueLength() const
    {
        return rx_queue_len;
    }
    uint32_t GetTWAITxQueueLength() const
    {
        return tx_queue_len;
    }

  private:
    [[noreturn]] static void alert_task(void *parameter);

    static void canIdToN2k(unsigned long id, unsigned char &prio, unsigned long &pgn, unsigned char &src, unsigned char &dst);
};

#endif
