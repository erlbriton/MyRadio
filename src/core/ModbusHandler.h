#pragma once
// yuArduinoVS/src/modbus/ModbusHandler.h

#include <Arduino.h>
#include <ModbusRTU.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "core/player.h"
#include "audioI2S/AudioEx.h"
#include "core/player.h"

// Настройки — поменяй при необходимости
#define MODBUS_UART       UART_NUM_2
#define MODBUS_TX_PIN     32
#define MODBUS_RX_PIN     34
#define MODBUS_BAUDRATE   115200
#define MODBUS_STACK_SZ   4096
#define MODBUS_TASK_PRIO  2
#define MODBUS_TASK_CORE  1
#define MODBUS_REG_COUNT 600
#define MODBUS_REG_NAME  50

class ModbusHandler {
public:
    ModbusHandler() : registerMutex(nullptr), taskHandle(nullptr) {
        // регистры уже инициализируются нулями в списке инициализации
    }
    // Запуск Modbus: slaveId — адрес устройства
    void begin(uint8_t slaveId) {
        // Создаём мьютекс для защиты holdingRegisters
        registerMutex = xSemaphoreCreateMutex();
        // if (!registerMutex) {
        //     Serial.println("ModbusHandler: failed to create mutex");
        //     // продолжим, но методы чтения/записи будут небезопасны
        // }
        // Настраиваем аппаратный Serial2 — НЕ используем низкоуровневые драйверы
        // (Serial2.begin автоматически настроит прерывания и внутренний RX-буфер)
        Serial2.begin(MODBUS_BAUDRATE, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
        delay(20); // короткая пауза, чтобы Serial2 успел инициализироваться
        // Инициализация библиотеки Modbus (передаём Serial2, -1 = нет DE/RE пина)
        mb.begin(&Serial2, 15, true);
        mb.slave(slaveId);
        // Регистрируем holding registers в библиотеке — привязка к нашим переменным
        // Если библиотека хранит ссылку — это хорошо; на всякий случай будем держать
        // локальную копию и при записи обновлять и mb.Hreg()
        for (uint16_t i = 0; i < MODBUS_REG_COUNT; ++i) 
        {
            mb.addHreg(i, holdingRegisters[i]);
        }
        // Создаём задачу, которая регулярно вызывает mb.task()
        BaseType_t r = xTaskCreatePinnedToCore(
            modbusTaskEntry, "ModbusTask",
            MODBUS_STACK_SZ,
            this,
            MODBUS_TASK_PRIO,
            &taskHandle,
            MODBUS_TASK_CORE
        );
        // if (r != pdPASS) {
        //     Serial.println("ModbusHandler: failed to create ModbusTask");
        // }
    }


// Запись строки UTF-8 в Modbus-регистры, начиная с любого стартового регистра.
// Каждая строка занимает ровно 50 регистров.
// Если center == true → строка центрируется, иначе пишется слева.

void writeStationNameUtf16le(uint16_t startIndex, const char* name, bool centered) {
    constexpr size_t REG_COUNT = 50;  // всегда 50 регистров
    if (!name) return;

    // Буфер для раскодированных символов UTF-16
    uint16_t decoded[REG_COUNT];
    size_t decodedLen = 0;

    // --- UTF-8 → UTF-16 ---
    for (size_t i = 0; name[i] != '\0' && decodedLen < REG_COUNT; ) {
        uint8_t byte1 = static_cast<uint8_t>(name[i]);
        uint16_t val = 0;

        if ((byte1 & 0x80) == 0) {
            // 1-байтовый символ ASCII
            val = byte1;
            i += 1;
        } else if ((byte1 & 0xE0) == 0xC0 && name[i + 1] != '\0') {
            // 2-байтовый UTF-8
            uint8_t byte2 = static_cast<uint8_t>(name[i + 1]);
            val = ((byte1 & 0x1F) << 6) | (byte2 & 0x3F);
            i += 2;
        } else if ((byte1 & 0xF0) == 0xE0 && name[i + 2] != '\0') {
            // 3-байтовый UTF-8
            uint8_t byte2 = static_cast<uint8_t>(name[i + 1]);
            uint8_t byte3 = static_cast<uint8_t>(name[i + 2]);
            val = ((byte1 & 0x0F) << 12) |
                  ((byte2 & 0x3F) << 6) |
                  (byte3 & 0x3F);
            i += 3;
        } else {
            // Неподдержанный символ, пропускаем
            i += 1;
            continue;
        }

        decoded[decodedLen++] = val;
    }

    // --- Центрирование или вывод слева ---
    size_t leftPadding = 0;
    if (centered && decodedLen < REG_COUNT) {
        leftPadding = (REG_COUNT - decodedLen) / 2;
    }

    size_t regIndex = startIndex;
    size_t outPos = 0;

    // Пробелы слева (если надо центрировать)
    for (size_t i = 0; i < leftPadding && outPos < REG_COUNT; i++) {
        mb.Hreg(regIndex + outPos, 0x0020);
        outPos++;
    }

    // Запись символов
    for (size_t i = 0; i < decodedLen && outPos < REG_COUNT; i++) {
        mb.Hreg(regIndex + outPos, decoded[i]);
        outPos++;
    }

    // Пробелы справа
    while (outPos < REG_COUNT) {
        mb.Hreg(regIndex + outPos, 0x0020);
        outPos++;
    }
}


void writeIntRegister(uint16_t startIndex, int value){
    size_t regIndex = startIndex;
    mb.Hreg(regIndex, (uint16_t)value);
}

private:
    // Входная точка задачи (статическая -> вызывает метод на this)
    static void modbusTaskEntry(void* pv) {
        ModbusHandler* self = static_cast<ModbusHandler*>(pv);
        self->modbusTask();
    }
    // Собственно задача: только mb.task()
    void modbusTask() {
        static uint16_t lastVol = 0;  // предыдущее значение громкости
    uint16_t prevReg200 = 0xFFFF; // специально недостижимое начальное значение
    SemaphoreHandle_t volMutex = xSemaphoreCreateMutex();
    for (;;) {
        mb.task();
        //digitalWrite(12, LOW);
//-----------------------Переключение станций с экрана---------------------------------------------------------------
static Audio audio;
static Player plr;
        uint16_t reg200 = mb.Hreg(200);
        // выводим только если значение изменилось
        if (reg200 != prevReg200) {
            prevReg200 = reg200;
        }

        // проверка битов (пример с NEXT)
        if (reg200 & 0x01) {
            player.sendCommand({PR_PLAY, config.lastStation() + 1});
            audio.setVolume(mb.Hreg(201));
            mb.Hreg(200, 0);
        }
        else if(reg200 & 0x02)
        {player.sendCommand({PR_PLAY, config.lastStation() - 1});
        audio.setVolume(mb.Hreg(201));
            mb.Hreg(200, 0);
        }
//----------------------Громкость-----------------------------------------------------------------------------------------
uint16_t vol = mb.Hreg(201);
//static Audio audio;

        if (vol != lastVol) {                // если громкость изменилась
            if (vol > 255) vol = 255;        // страховка
            audio.setVolume((uint8_t)vol);   // применяем
            lastVol = vol;                   // сохраняем новое значение
            //Serial.printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Volume set to %u\n", vol);  // отладка в терминале
        }

        // uint16_t reg201 = mb.Hreg(201);
        // if(xSemaphoreTake(volMutex, pdMS_TO_TICKS(1))) {
        //     if(reg201 != audio.getVolume()) {
        //         audio.setVolume(reg201);
        //     }
        //     // синхронизация регистра Modbus с текущей громкостью
        //     mb.Hreg(201, audio.getVolume());
        //     xSemaphoreGive(volMutex);
        // }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

    // Данные
    uint16_t holdingRegisters[MODBUS_REG_COUNT] = {0};
    ModbusRTU mb;
    SemaphoreHandle_t registerMutex;
    TaskHandle_t taskHandle;    
};
