
#pragma once
#include <Arduino.h>
#include "driver/rmt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --- НАСТРОЙКИ (КОНСТАНТЫ) ---

// 1. Твой Уникальный Адрес (десятичный)
static constexpr uint32_t MY_REMOTE_ADDRESS = 951727; 

// 2. Тайминги сигнала (из твоих измерений)
static constexpr uint32_t T_LONG = 690;
static constexpr uint32_t T_SHORT = 224;
static constexpr uint32_t T_TOLERANCE = 120; // Допуск +/- 120 мкс

static constexpr uint32_t T_LONG_MIN = T_LONG - T_TOLERANCE;
static constexpr uint32_t T_LONG_MAX = T_LONG + T_TOLERANCE;
static constexpr uint32_t T_SHORT_MIN = T_SHORT - T_TOLERANCE;
static constexpr uint32_t T_SHORT_MAX = T_SHORT + T_TOLERANCE;

// 3. Параметры пакета
static constexpr uint8_t TOTAL_BITS = 25;     // Всего бит
static constexpr uint32_t PACKET_TIMEOUT = 300000; // 300 мс (время для сброса состояния)

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
        
        // --- ФИЛЬТРАЦИЯ ШУМА НА УРОВНЕ RMT ---
        cfg.rx_config.filter_en = true;
        cfg.rx_config.filter_ticks_thresh = 60; 
        cfg.rx_config.idle_threshold = 4000; 

        rmt_config(&cfg);
        rmt_driver_install(channel, 1000, 0);
        rmt_get_ringbuf_handle(channel, &rb);
        rmt_rx_start(channel, true);

        xTaskCreate([](void* param){
            ((HS1527Receiver*)param)->taskLoop();
        }, "RX_Task", 4096, this, 10, nullptr);
    }

private:
    RingbufHandle_t rb = nullptr;

    // Переменные для логики одной пачки
    uint32_t lastPacketTimestamp = 0;
    bool commandSent = false;       // Блокирует повторы пока кнопка нажата
    
    // Переменная сохранена для проверки, не пришла ли другая команда
    // во время удержания первой (хотя маловероятно).
    uint8_t lastExecutedCmd = 0; 


    void taskLoop() {
        while(true) {
            size_t rx_size = 0;
            rmt_item32_t* items = (rmt_item32_t*)xRingbufferReceive(rb, &rx_size, portMAX_DELAY);
            
            if(items) {
                int pulses = rx_size / sizeof(rmt_item32_t);

                // 1. ПЕРВИЧНЫЙ ОТСЕВ: ДЛИНА
                if (pulses >= TOTAL_BITS) {
                    processRawData(items);
                }

                vRingbufferReturnItem(rb, (void*)items);
            }
        }
    }

    void processRawData(rmt_item32_t* items) {
        uint32_t accumulatedBits = 0;

        // 2. ЦИКЛ РАСПОЗНАВАНИЯ И ПРОВЕРКИ ТАЙМИНГОВ
        for (int i = 0; i < TOTAL_BITS; i++) {
            uint32_t duration = items[i].duration0; 
            
            if (duration >= T_LONG_MIN && duration <= T_LONG_MAX) {
                accumulatedBits = (accumulatedBits << 1) | 1;
            } 
            else if (duration >= T_SHORT_MIN && duration <= T_SHORT_MAX) {
                accumulatedBits = (accumulatedBits << 1) | 0;
            } 
            else {
                // ПОМЕХА! Тайминг не совпал. Немедленный сброс.
                return; 
            }
        }

        // 3. ПРОВЕРКА АДРЕСА
        uint32_t receivedAddress = accumulatedBits >> 5; 
        uint8_t currentCmd = accumulatedBits & 0x1F;

        if (receivedAddress != MY_REMOTE_ADDRESS) {
            // Чужой пульт. Игнорируем.
            return;
        }

        // 4. ЛОГИКА МГНОВЕННОЙ РЕАКЦИИ
        handleCommandLogic(currentCmd);
    }

    void handleCommandLogic(uint8_t cmd) {
        uint32_t now = micros();

        // Проверка таймаута: Кнопка отпущена?
        if (now - lastPacketTimestamp > PACKET_TIMEOUT) {
            // Если да, сбрасываем флаг, разрешая новую команду.
            commandSent = false;
        }
        lastPacketTimestamp = now;

        // --- ТОЧКА ПРИНЯТИЯ РЕШЕНИЯ ---
        // Если команда НЕ была отправлена (commandSent == false), 
        // это значит, что это первая чистая пачка после отпускания кнопки.
        if (!commandSent) {
            // Выполняем команду
            if (onCommand) onCommand(cmd); 
            
            // Блокируем дальнейшее выполнение до следующего таймаута
            commandSent = true; 
            lastExecutedCmd = cmd;
        }
        // Если commandSent == true, то мы игнорируем текущую пачку (даже если она чистая),
        // пока пользователь держит кнопку.
    }
};