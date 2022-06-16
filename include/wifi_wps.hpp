#pragma once
#ifndef ESP32
#error "This library only supports the ESP32 MCU."
#endif
#include <Arduino.h>
#include <SPIFFS.h>
#include <atomic>
#include <esp_wps.h>
namespace arduino {
typedef void(*wifi_wps_callback)(bool connected,void* state);
class wifi_wps {
    
public:
    wifi_wps(uint32_t connect_timeout = 15000);
    void callback(wifi_wps_callback callback, void* state=nullptr);
    void update();
};
}