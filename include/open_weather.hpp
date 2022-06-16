#pragma once
#ifndef ESP32
#error "This library only supports the ESP32 MCU."
#endif
#include <Arduino.h>
namespace arduino {
struct open_weather_info final {
    float temperature;
    float feels_like;
    float pressure;
    float humidity;
    float visiblity;
    float wind_speed;
    float wind_direction;
    float wind_gust;
    float cloudiness;
    float rain_last_hour;
    float snow_last_hour;
    long utc_offset;
    time_t timestamp;
    time_t sunrise;
    time_t sunset;
    char icon[8];
    char main[32];
    char city[64];
    char description[128];
};
struct open_weather final {
    static bool fetch(float latitude, float longitude, open_weather_info* out_info);
};
}