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

#include "NMEA2000_esp32.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"

// http://www.bittiming.can-wiki.info/ (Clock Rate set to 80Mhz)
// https://www.esacademy.com/en/library/calculators/sja1000-timing-calculator.html
// https://www.simmasoftware.com/j1939.html

// NMEA 2000 = SAE J1939-21
// Sample point should be as close to 87.5% but not past.
// SJW = 1

// #define TWAI_TIMING_CONFIG_NMEA2000() { .brp = 20, .tseg_1 = 13, .tseg_2 = 2, .sjw = 1, .triple_sampling = true }

#define TWAI_TIMING_CONFIG_NMEA2000()                                                                                                         \
    {                                                                                                                                         \
        .clk_src = (twai_clock_source_t)0, .quanta_resolution_hz = 0, .brp = 16, .tseg_1 = 16, .tseg_2 = 3, .sjw = 1, .triple_sampling = true \
    }

#define CAN_FRAME_HEADER_BITS 52

bool tNMEA2000_esp32::CanInUse = false;
tNMEA2000_esp32 *pNMEA2000_esp32 = 0;

//*****************************************************************************
tNMEA2000_esp32::tNMEA2000_esp32(gpio_num_t _TxPin, gpio_num_t _RxPin, TickType_t _rxWaitTicks)
    : tNMEA2000(), IsOpen(false), TxPin(_TxPin), RxPin(_RxPin), RxWaitTicks(_rxWaitTicks)
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

    // Queue message for transmission
    esp_err_t res = twai_transmit(&message, 0);

    if (res == ESP_ERR_INVALID_STATE)
    {
        twai_status_info_t status;

        if (twai_get_status_info(&status) == ESP_OK)
        {
            // ESP_LOGI(TAG, "twai status = %x", status.state);
            switch (status.state)
            {
            case TWAI_STATE_STOPPED:
                // ESP_LOGI(TAG, "twai_start");
                twai_start();
                break;
            case TWAI_STATE_BUS_OFF:
                // ESP_LOGI(TAG, "twai_initiate_recovery");
                twai_initiate_recovery();
                break;
            default:
                break;
            }
            return false;
        }
    }
#if ESP32_CAN_STATISTICS == 1    
    if (res == ESP_OK)
    {
        TxBits += CAN_FRAME_HEADER_BITS + len * 8;
        TxPackets++;
    }
#endif
    return res == ESP_OK;
}

//*****************************************************************************
bool tNMEA2000_esp32::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf)
{
    twai_message_t message;

    if (twai_receive(&message, RxWaitTicks) == ESP_OK && message.extd)
    {
        id = message.identifier;
        len = message.data_length_code;

        memcpy(buf, message.data, message.data_length_code);

#if ESP32_CAN_STATISTICS == 1
        RxBits += CAN_FRAME_HEADER_BITS + message.data_length_code * 8;
        RxPackets++;
#endif

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

#if ESP32_CAN_STATISTICS == 1
    const esp_timer_create_args_t tick_timer_args = {
        .callback = &tNMEA2000_esp32::Timer_tick,
        .arg = this,
        .dispatch_method = (esp_timer_dispatch_t)0,
        .name = "NMEA2000_esp32_tick",
        .skip_unhandled_events = false};

    esp_timer_handle_t tick_timer = NULL;

    ESP_ERROR_CHECK(esp_timer_create(&tick_timer_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, 1000*1000));
#endif
}

#if ESP32_CAN_STATISTICS == 1
void tNMEA2000_esp32::Timer_tick(void *arg)
{
    tNMEA2000_esp32 *pThis = (tNMEA2000_esp32 *)arg;

    pThis->RxPacketsPerSecond = (unsigned long)(pThis->RxPacketsPerSecond * 0.05 + pThis->RxPackets * 0.95);
    pThis->RxPackets = 0;

    pThis->RxBitsPerSeconds = (unsigned long)(pThis->RxBitsPerSeconds * 0.05 + pThis->RxBits * 0.95);
    pThis->RxBits = 0;

    pThis->TxPacketsPerSecond = (unsigned long)(pThis->TxPacketsPerSecond * 0.05 + pThis->TxPackets * 0.95);
    pThis->TxPackets = 0;

    pThis->TxBitsPerSecond = (unsigned long)(pThis->TxBitsPerSecond * 0.05 + pThis->TxBits * 0.95);
    pThis->TxBits = 0;
}
#endif

//*****************************************************************************
void tNMEA2000_esp32::InitCANFrameBuffers()
{
    tNMEA2000::InitCANFrameBuffers(); // call main initialization
}
