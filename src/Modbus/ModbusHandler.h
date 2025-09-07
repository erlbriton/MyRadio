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
#define MODBUS_REG_COUNT  10
#define MODBUS_STACK_SZ   4096
#define MODBUS_TASK_PRIO  2
#define MODBUS_TASK_CORE  1

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
        for (uint16_t i = 0; i < MODBUS_REG_COUNT; ++i) {
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

    // Потокобезопасное чтение одного регистра
    uint16_t readHreg(uint16_t index) {
        uint16_t value = 0;
        if (index >= MODBUS_REG_COUNT) return 0;
        if (registerMutex) {
            if (xSemaphoreTake(registerMutex, portMAX_DELAY) == pdTRUE) {
                value = holdingRegisters[index];
                xSemaphoreGive(registerMutex);
            }
        } else {
            value = holdingRegisters[index]; // небезопасно, если мьютекс не создан
        }
        return value;
    }

    // Потокобезопасная запись одного регистра (и синхронизация с библиотекой)
    void writeHreg(uint16_t index, uint16_t value) {
        if (index >= MODBUS_REG_COUNT) return;
        if (registerMutex) {
            if (xSemaphoreTake(registerMutex, portMAX_DELAY) == pdTRUE) {
                holdingRegisters[index] = value;
                // Обновим значение в таблице библиотеки (если библиотека хранит копию)
                mb.Hreg(index, value);
                xSemaphoreGive(registerMutex);
            }
        } else {
            holdingRegisters[index] = value;
            mb.Hreg(index, value);
        }
    }

    // Запись нескольких регистров атомарно
void writeHregs(uint16_t startIndex, const uint16_t* values, size_t count) {
    if (!values) return;
    if (startIndex >= MODBUS_REG_COUNT) return;

    // вычисляем, сколько реально можем записать (без переполнения)
    size_t avail = (size_t)(MODBUS_REG_COUNT - startIndex);
    size_t maxCount = (count < avail) ? count : avail;

    if (registerMutex) {
        if (xSemaphoreTake(registerMutex, portMAX_DELAY) == pdTRUE) {
            for (size_t i = 0; i < maxCount; ++i) {
                holdingRegisters[startIndex + i] = values[i];
                mb.Hreg(startIndex + i, values[i]); // синхронизация с библиотекой
            }
            xSemaphoreGive(registerMutex);
        }
    } else {
        for (size_t i = 0; i < maxCount; ++i) {
            holdingRegisters[startIndex + i] = values[i];
            mb.Hreg(startIndex + i, values[i]);
        }
    }
}

    // Чтение нескольких регистров атомарно
    void readHregs(uint16_t startIndex, uint16_t* outBuffer, size_t count) {
    if (!outBuffer) return;
    if (startIndex >= MODBUS_REG_COUNT) return;

    size_t avail = (size_t)(MODBUS_REG_COUNT - startIndex);
    size_t maxCount = (count < avail) ? count : avail;

    if (registerMutex) {
        if (xSemaphoreTake(registerMutex, portMAX_DELAY) == pdTRUE) {
            for (size_t i = 0; i < maxCount; ++i) outBuffer[i] = holdingRegisters[startIndex + i];
            xSemaphoreGive(registerMutex);
        }
    } else {
        for (size_t i = 0; i < maxCount; ++i) outBuffer[i] = holdingRegisters[startIndex + i];
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
