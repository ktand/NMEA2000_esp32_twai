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
#include "NMEA2000.h"
#include "driver/twai.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/projdefs.h"
#include "hal/twai_types.h"

// http://www.bittiming.can-wiki.info/ (Clock Rate set to 80Mhz)
// https://www.esacademy.com/en/library/calculators/sja1000-timing-calculator.html
// https://www.simmasoftware.com/j1939.html

// NMEA 2000 = SAE J1939-21
// Sample point should be as close to 87.5% but not past.
// SJW = 1

// #define TWAI_TIMING_CONFIG_NMEA2000() { .brp = 20, .tseg_1 = 13, .tseg_2 = 2, .sjw = 1, .triple_sampling = true }

#define TAG "NMEA2000_esp32"
#define ALERT_TASK_PRIO 10

#define TWAI_TIMING_CONFIG_NMEA2000()                                                                                                         \
    {                                                                                                                                         \
        .clk_src = (twai_clock_source_t)0, .quanta_resolution_hz = 0, .brp = 16, .tseg_1 = 16, .tseg_2 = 3, .sjw = 1, .triple_sampling = true \
    }

#define CAN_FRAME_HEADER_BITS 52

bool tNMEA2000_esp32::CanInUse = false;

tNMEA2000_esp32 *pNMEA2000_esp32 = 0;

//*****************************************************************************
tNMEA2000_esp32::tNMEA2000_esp32(gpio_num_t _TxPin, gpio_num_t _RxPin, TickType_t _rxWaitTicks)
    : tNMEA2000(), IsOpen(false), TxPin(_TxPin), RxPin(_RxPin), receive_wait_ticks(_rxWaitTicks)
{
    alert_task_semaphore = xSemaphoreCreateBinary();
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

void tNMEA2000_esp32::CAN_init()
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TxPin, RxPin, TWAI_MODE_NORMAL);

    g_config.rx_queue_len = 32;
    g_config.tx_queue_len = 32;
    g_config.alerts_enabled = ALERTS_TO_WATCH;

#ifdef ESP32_CAN_ISR_IN_IRAM
    g_config.intr_flags = ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM;
#else
    g_config.intr_flags = ESP_INTR_FLAG_LEVEL3;
#endif

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));

    // Start TWAI driver
    ESP_ERROR_CHECK(twai_start());

    // Create alert task
    xTaskCreatePinnedToCore(alert_task, "twai_alert_task", 2048, this, ALERT_TASK_PRIO, nullptr, tskNO_AFFINITY);

    // Allow alert task to run
    xSemaphoreGive(alert_task_semaphore);

#if ESP32_CAN_STATISTICS == 1
    const esp_timer_create_args_t tick_timer_args = {
        .callback = &tNMEA2000_esp32::Timer_tick,
        .arg = this,
        .dispatch_method = (esp_timer_dispatch_t)0,
        .name = "NMEA2000_esp32_tick",
        .skip_unhandled_events = false};

    esp_timer_handle_t tick_timer = NULL;

    ESP_ERROR_CHECK(esp_timer_create(&tick_timer_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, 1000 * 1000));
#endif
}

//*****************************************************************************
bool tNMEA2000_esp32::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent)
{
    unsigned char prio, src, dst;
    unsigned long pgn;

    twai_status_info_t status_info;

    // Check if the driver is in the running state before trying to transmit
    twai_get_status_info(&status_info);

    if (status_info.state != TWAI_STATE_RUNNING)
    {
        ESP_LOGE(TAG, "Failed to send CAN Frame: Driver is not in running state: %x", status_info.state);
        return false;
    }

    if (esp_log_level_get(TAG) >= ESP_LOG_INFO)
    {
        canIdToN2k(id, prio, pgn, src, dst);

        ESP_LOGI(TAG, "CANSendFrame Len = %d, Prio = %d, PGN = %ld, Src = %d, Dst = %d", len, prio, pgn, src, dst);
    }

    twai_message_t message;
    memset(&message, 0, sizeof(message));
    message.extd = 1;
    message.identifier = id;
    message.data_length_code = len;
    message.ss = 0;
    memcpy(message.data, buf, len);

    // Queue message for transmission
    esp_err_t res = twai_transmit(&message, wait_sent ? portMAX_DELAY : 0);

    if (res == ESP_OK)
    {
#if ESP32_CAN_STATISTICS == 1
        if (res == ESP_OK)
        {
            TxBits += CAN_FRAME_HEADER_BITS + len * 8;
            TxPackets++;
        }
#endif
        return true;
    }

    ESP_LOGE(TAG, "Failed to queue message for transmission: %d\n", res);
    return false;
}

//*****************************************************************************
bool tNMEA2000_esp32::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf)
{
    twai_message_t message;

    auto res = twai_receive(&message, receive_wait_ticks);

    if (res == ESP_OK)
    {
        if (message.extd)
        {
            id = message.identifier;
            len = message.data_length_code;

            memcpy(buf, message.data, message.data_length_code);

            unsigned char prio, src, dst;
            unsigned long pgn;

            if (esp_log_level_get(TAG) >= ESP_LOG_INFO)
            {
                canIdToN2k(id, prio, pgn, src, dst);

                ESP_LOGI(TAG, "CANGetFrame Len = %d, Prio = %d, PGN = %ld, Src = %d, Dst = %d", len, prio, pgn, src, dst);
            }

#if ESP32_CAN_STATISTICS == 1
            RxBits += CAN_FRAME_HEADER_BITS + message.data_length_code * 8;
            RxPackets++;
#endif

            return true;
        }
        return false;
    }
    else if (res != ESP_ERR_TIMEOUT)
    {
        ESP_LOGE(TAG, "twai_receive failed: %d", res);
    }
    return false;
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

void tNMEA2000_esp32::canIdToN2k(unsigned long id, unsigned char &prio, unsigned long &pgn, unsigned char &src, unsigned char &dst)
{
    unsigned char CanIdPF = (unsigned char)(id >> 16);
    unsigned char CanIdPS = (unsigned char)(id >> 8);
    unsigned char CanIdDP = (unsigned char)(id >> 24) & 1;

    src = (unsigned char)id >> 0;
    prio = (unsigned char)((id >> 26) & 0x7);

    if (CanIdPF < 240)
    {
        /* PDU1 format, the PS contains the destination address */
        dst = CanIdPS;
        pgn = (((unsigned long)CanIdDP) << 16) | (((unsigned long)CanIdPF) << 8);
    }
    else
    {
        /* PDU2 format, the destination is implied global and the PGN is extended */
        dst = 0xff;
        pgn = (((unsigned long)CanIdDP) << 16) | (((unsigned long)CanIdPF) << 8) | (unsigned long)CanIdPS;
    }
}

[[noreturn]] void tNMEA2000_esp32::alert_task(void *param)
{
    tNMEA2000_esp32 *pThis = (tNMEA2000_esp32 *)param;

    xSemaphoreTake(pThis->alert_task_semaphore, portMAX_DELAY);

    while (true)
    {
        uint32_t alerts;
        twai_read_alerts(&alerts, portMAX_DELAY);

        if (pThis->alerts_callback != nullptr)
        {
            pThis->alerts_callback(alerts, alerts & ERROR_ALERTS_TO_WATCH);
        }

        if (alerts & TWAI_ALERT_ABOVE_ERR_WARN)
        {
            ESP_LOGE(TAG, "One of the error counters have exceeded the error warning limit");
        }
        if (alerts & TWAI_ALERT_ERR_PASS)
        {
            ESP_LOGE(TAG, "TWAI controller has become error passive");
        }
        if (alerts & TWAI_ALERT_BUS_OFF)
        {
            ESP_LOGE(TAG, "Bus-off condition occurred");

            // Reconfigure alerts to detect bus recovery completion
            twai_reconfigure_alerts(TWAI_ALERT_BUS_RECOVERED, nullptr);

            ESP_LOGE(TAG, "Initiate bus recovery");
            twai_initiate_recovery(); // Needs 128 occurrences of bus free signal
        }
        if (alerts & TWAI_ALERT_BUS_RECOVERED)
        {
            // Bus recovery successful
            ESP_LOGI(TAG, "TWAI controller has successfully completed bus recovery");

            // Start TWAI driver
            if (twai_start() == ESP_OK)
            {
                ESP_LOGI(TAG, "TWAI Driver started");
            }
            else
            {
                ESP_LOGE(TAG, "Failed to start driver");
            }

            // Start monitoring alerts again
            twai_reconfigure_alerts(ALERTS_TO_WATCH, nullptr);
        }
    }
}

void tNMEA2000_esp32::SetLogLevel(esp_log_level_t level)
{
    esp_log_level_set(TAG, level);
}