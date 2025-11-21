
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
