

// #pragma once
// #include <Arduino.h>
// #include "driver/rmt.h"

// class HS1527Receiver {
// public:
//     typedef void (*CommandCallback)(uint8_t cmd);

//     CommandCallback onCommand = nullptr;

//     void begin(gpio_num_t pin) {

//         rmt_config_t cfg = {};
//         cfg.rmt_mode = RMT_MODE_RX;
//         cfg.channel  = channel;
//         cfg.gpio_num = pin;
//         cfg.clk_div  = 80;       // 1 tick = 1us
//         cfg.mem_block_num = 2;   // достаточно
//         cfg.rx_config.filter_en = true;
//         cfg.rx_config.filter_ticks_thresh = 100; // 100us
//         cfg.rx_config.idle_threshold = 10000;    // 10 ms = пауза между пачками

//         rmt_config(&cfg);
//         rmt_driver_install(channel, 2000, 0);    // небольшой буфер
//         rmt_get_ringbuf_handle(channel, &rb);
//         rmt_rx_start(channel, true);
//     }

//     void process() {
//         if (!rb) return;

//         size_t len = 0;
//         rmt_item32_t* items = (rmt_item32_t*) xRingbufferReceive(rb, &len, 0);
//         if (!items) return;

//         int count = len / sizeof(rmt_item32_t);
//         if (count < 25) {
//             vRingbufferReturnItem(rb, (void*)items);
//             return;
//         }

//         // Декодировать последние 5 импульсов (первые 20 - адрес, пока игнорируем)
//         uint8_t cmd = decode5Bits(items + 20);

//         if (cmd == lastCode) {
//             if (onCommand) onCommand(cmd);
//             lastCode = 255; // сброс
//         } else {
//             lastCode = cmd;
//         }

//         vRingbufferReturnItem(rb, (void*)items);
//     }

// private:
//     const rmt_channel_t channel = RMT_CHANNEL_0;
//     RingbufHandle_t rb = nullptr;
//     uint8_t lastCode = 255;

//     uint8_t decode5Bits(rmt_item32_t* items) {

//         uint8_t code = 0;

//         for (int i=0;i<5;i++) {
//             uint32_t high = items[i].duration0;
//             // low = items[i].duration1 (не нужен)

//             // сравниваем HIGH-длительность с порогом 450 мкс
//             bool bit = (high > 450);
//             code = (code << 1) | (bit ? 1 : 0);
//         }

//         return code;
//     }
// };


#pragma once
#include <Arduino.h>
#include "driver/rmt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

class HS1527Receiver {
public:
    typedef void (*CommandCallback)(uint8_t cmd);
    CommandCallback onCommand = nullptr;

    rmt_channel_t channel = RMT_CHANNEL_0;
    gpio_num_t pin;

    void begin(gpio_num_t _pin) {
        pin = _pin;

        rmt_config_t cfg;
        cfg.rmt_mode = RMT_MODE_RX;
        cfg.channel = channel;
        cfg.gpio_num = pin;
        cfg.clk_div = 80;
        cfg.mem_block_num = 8;
        cfg.rx_config.filter_en = true;
        cfg.rx_config.filter_ticks_thresh = 50;
        cfg.rx_config.idle_threshold = 4000;
        rmt_config(&cfg);

        // создаем драйвер с ringbuffer
        rmt_driver_install(channel, 1000, 0);  // rx_buffer_size = 1000 * 4 bytes
        rmt_get_ringbuf_handle(channel, &rb);
        rmt_rx_start(channel, true);

        // таска обработки
        xTaskCreate([](void* param){
            ((HS1527Receiver*)param)->taskLoop();
        }, "HS1527Task", 4096, this, 2, nullptr);
    }

private:
    RingbufHandle_t rb = nullptr;
    uint32_t hs_rawPacket[5] = {0};

    void taskLoop() {
        while(true) {
            if(!rb) {
                vTaskDelay(1);
                continue;
            }

            size_t rx_size = 0;
            rmt_item32_t* items = (rmt_item32_t*)xRingbufferReceive(rb, &rx_size, pdMS_TO_TICKS(10));
            if(!items) continue;

            int count = rx_size / sizeof(rmt_item32_t);
            if(count >= 25) {
                for(int i = 0; i < 5; i++)
                    hs_rawPacket[i] = items[20 + i].duration0;

                uint8_t cmd = decodePacket();
                if(onCommand) onCommand(cmd);
            }

            vRingbufferReturnItem(rb, (void*)items);
        }
    }

    uint8_t decodePacket() {
        uint8_t cmd = 0;
        for(int i = 0; i < 5; i++){
            bool bit = (hs_rawPacket[i] > 450);
            cmd = (cmd << 1) | (bit ? 1 : 0);
        }
        return cmd;
    }
};
