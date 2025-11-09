#include "Arduino.h"
#include "core/options.h"
#include "core/config.h"
#include "core/telnet.h"
#include "core/player.h"
#include "core/display.h"
#include "core/network.h"
#include "core/netserver.h"
#include "core/controls.h"
#include "core/mqtt.h"
#include "core/optionschecker.h"
#include "core/timekeeper.h"
//#include <ModbusRTU.h>
#include "core/ModbusHandler.h"
#include "core/audiohandlers.h"


#if USE_OTA
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
#include <NetworkUdp.h>
#else
#include <WiFiUdp.h>
#endif
#include <ArduinoOTA.h>
#endif

#if DSP_HSPI || TS_HSPI || VS_HSPI
SPIClass  SPI2(HOOPSENb);
#endif

extern __attribute__((weak)) void yoradio_on_setup();

#if USE_OTA
void setupOTA(){
  if(strlen(config.store.mdnsname)>0)
    ArduinoOTA.setHostname(config.store.mdnsname);
#ifdef OTA_PASS
  ArduinoOTA.setPassword(OTA_PASS);
#endif
  ArduinoOTA
    .onStart([]() {
      player.sendCommand({PR_STOP, 0});
      display.putRequest(NEWMODE, UPDATING);
      telnet.printf("Start OTA updating %s\n", ArduinoOTA.getCommand() == U_FLASH?"firmware":"filesystem");
    })
    .onEnd([]() {
      telnet.printf("\nEnd OTA update, Rebooting...\n");
      ESP.restart();
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      telnet.printf("Progress OTA: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      telnet.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        telnet.printf("Auth Failed\n");
      } else if (error == OTA_BEGIN_ERROR) {
        telnet.printf("Begin Failed\n");
      } else if (error == OTA_CONNECT_ERROR) {
        telnet.printf("Connect Failed\n");
      } else if (error == OTA_RECEIVE_ERROR) {
        telnet.printf("Receive Failed\n");
      } else if (error == OTA_END_ERROR) {
        telnet.printf("End Failed\n");
      }
    });
  ArduinoOTA.begin();
}
#endif

//ModbusRTU mb;
// uint16_t reg0 = 0;
// uint16_t prevReg0 = 0xFFFF; // Для отслеживания изменений
ModbusHandler modbus;

void setup() {
  Serial.begin(115200);
  //----------------------Modbus--------------------------------------
 modbus.begin(1);  // Slave ID = 1
 //Serial2.begin(115200, SERIAL_8N1, 12, 14);
//----------------------------------------------------------------------------
  if(REAL_LEDBUILTIN!=255) pinMode(REAL_LEDBUILTIN, OUTPUT);
  if (yoradio_on_setup) yoradio_on_setup();
  pm.on_setup();
  config.init();
  display.init();
  player.init();
  network.begin();
  if (network.status != CONNECTED && network.status!=SDREADY) {
    netserver.begin();
    initControls();
    display.putRequest(DSP_START);
    while(!display.ready()) delay(10);
    return;
  }
  if(SDC_CS!=255) {
    display.putRequest(WAITFORSD, 0);
    Serial.print("##[BOOT]#\tSD search\t");
  }
  config.initPlaylistMode();
  netserver.begin();
  telnet.begin();
  initControls();
  display.putRequest(DSP_START);
  while(!display.ready()) delay(10);
  #ifdef MQTT_ROOT_TOPIC
    mqttInit();
  #endif
  #if USE_OTA
    setupOTA();
  #endif
  if (config.getMode()==PM_SDCARD) player.initHeaders(config.station.url);
  player.lockOutput=false;
  if (config.store.smartstart == 1) {
    delay(250);
    player.sendCommand({PR_PLAY, config.lastStation()});
    //player.sendCommand({PR_PLAY, 14});
  }
  pm.on_end_setup();
  //---------------------------------------------------------------------------------
//   mb.begin(&Serial2, true); // true = режим обработки с аппаратным буфером (IRQ)
//   mb.slave(1); // Адрес slave = 1
//  // Добавляем Holding-регистр 0
//   mb.addHreg(0, reg0); // Изначально = 0
}

void loop() {
  timekeeper.loop1();
  telnet.loop();
  if (network.status == CONNECTED || network.status==SDREADY) {
    player.loop();
#if USE_OTA
    ArduinoOTA.handle();
#endif
  }
  loopControls();
  netserver.loop();
  

  //---------------------Проверка UART------------------------------------------
  // Serial2.print("9876543210");
  // Serial2.print("\r\n"); // Перевод строки
  // delay(1000); // Раз в секунду
  // Serial2.print("012345678");
  // Serial2.print("\r\n"); // Перевод строки
  // delay(1000); // Раз в секунду

  //Для проверки увеличиваем все регистры раз в секунду
//   static uint32_t lastMillis = 0;
// if (millis() - lastMillis >= 1000) {
//         lastMillis = millis();

//         for (uint16_t i = 0; i < 10; i++) {
//             uint16_t val = modbus.readHreg(i);
//             //modbus.writeHreg(i, val + 1);
//             modbus.writeHreg(i, 20);

//     //    Для отладки — выведем текущее состояние регистров в монитор
//         Serial.println("Updated Modbus Holding Registers:");
//         for (uint16_t i = 0; i < 10; i++) {
//             uint16_t val = modbus.readHreg(i);
//             Serial.printf("Hreg[%u] = 0x%04X\n", i, val);
//         }
//     }
// }
}
