#pragma once
// yuArduinoVS/src/modbus/ModbusHandler.h

#include <Arduino.h>
#include <ModbusRTU.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Настройки — поменяй при необходимости
#define MODBUS_UART       UART_NUM_2
#define MODBUS_TX_PIN     32
#define MODBUS_RX_PIN     34
#define MODBUS_BAUDRATE   115200
#define MODBUS_REG_COUNT  170
#define MODBUS_STACK_SZ   4096
#define MODBUS_TASK_PRIO  2
#define MODBUS_TASK_CORE  1
#define MODBUS_REG_COUNT 70

class ModbusHandler {
public:
    ModbusHandler() : registerMutex(nullptr), taskHandle(nullptr) {
        // регистры уже инициализируются нулями в списке инициализации
    }
    // Запуск Modbus: slaveId — адрес устройства
    void begin(uint8_t slaveId) {
        // Создаём мьютекс для защиты holdingRegisters
        registerMutex = xSemaphoreCreateMutex();
        if (!registerMutex) {
            Serial.println("ModbusHandler: failed to create mutex");
            // продолжим, но методы чтения/записи будут небезопасны
        }
        // Настраиваем аппаратный Serial2 — НЕ используем низкоуровневые драйверы
        // (Serial2.begin автоматически настроит прерывания и внутренний RX-буфер)
        Serial2.begin(MODBUS_BAUDRATE, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
        delay(20); // короткая пауза, чтобы Serial2 успел инициализироваться
        // Инициализация библиотеки Modbus (передаём Serial2, -1 = нет DE/RE пина)
        mb.begin(&Serial2, -1, false);
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
        if (r != pdPASS) {
            Serial.println("ModbusHandler: failed to create ModbusTask");
        }
    }
// Записывает имя станции в UTF-16LE в холдинг-регистры, один регистр на символ
void writeStationNameUtf16le(uint16_t startIndex, const char* name) {
    if (!name || startIndex >= MODBUS_REG_COUNT) return;

    uint16_t regIndex = startIndex;

    for (size_t i = 0; name[i] != '\0' && regIndex < MODBUS_REG_COUNT; ) {
        uint16_t val = 0;
        uint8_t byte1 = static_cast<uint8_t>(name[i]);

        if ((byte1 & 0x80) == 0) {
            // ASCII (латиница, цифры)
            val = byte1;
            i += 1;
        } else if ((byte1 & 0xE0) == 0xC0 && name[i + 1] != '\0') {
            // Двухбайтовый UTF-8 (кириллица)
            uint8_t byte2 = static_cast<uint8_t>(name[i + 1]);
            val = ((byte1 & 0x1F) << 6) | (byte2 & 0x3F);
            i += 2;
        } else {
            // Невалидный или не поддерживаемый UTF-8 символ — пропускаем
            i += 1;
            continue;
        }
        // Записываем в регистр Modbus
        mb.Hreg(regIndex++, val);
    }
    // После строки ставим один ноль, если есть место
    if (regIndex < MODBUS_REG_COUNT) {
        mb.Hreg(regIndex, 0x0000);
    }
}
private:
    // Входная точка задачи (статическая -> вызывает метод на this)
    static void modbusTaskEntry(void* pv) {
        ModbusHandler* self = static_cast<ModbusHandler*>(pv);
        self->modbusTask();
    }
    // Собственно задача: только mb.task()
    void modbusTask() {
        for (;;) {
            mb.task();          // единственный корректный вызов для этой библиотеки
            vTaskDelay(pdMS_TO_TICKS(1)); // даём CPU другим задачам
        }
    }
    // Данные
    uint16_t holdingRegisters[MODBUS_REG_COUNT] = {0};
    ModbusRTU mb;
    SemaphoreHandle_t registerMutex;
    TaskHandle_t taskHandle;    
};
