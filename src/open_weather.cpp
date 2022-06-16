#include <open_weather.hpp>
#include <ArduinoJson.hpp>
#include <HTTPClient.h>
#include "open_weather_api_key.h"
using namespace ArduinoJson;
namespace arduino {
bool open_weather::fetch(float latitude, float longitude,open_weather_info* out_info) {
    constexpr static const char *url_format = 
        "http://api.openweathermap.org/data/2.5/weather?lat=%f&lon=%f&units=metric&lang=en&appid=%s";
    if(out_info==nullptr) { 
        return false;
    }
    char url[512];
    sprintf(url,url_format,latitude,longitude,OPEN_WEATHER_API_KEY);
    HTTPClient client;
    client.begin(url);
    if(0>=client.GET()) {
        return false;
    }
    DynamicJsonDocument doc(8192);
    deserializeJson(doc,client.getString());
    client.end();
    JsonObject obj = doc.as<JsonObject>();
    String str = obj[F("name")];
    strncpy(out_info->city,str.c_str(),64);
    out_info->visiblity = obj[F("visibility")];
    out_info->utc_offset = (long)obj[F("timezone")];
    out_info->timestamp = (time_t)(long)obj[F("dt")];
    JsonObject so = obj[F("weather")].as<JsonArray>().getElement(0).as<JsonObject>();
    str=so[F("main")].as<String>();
    strncpy(out_info->main,str.c_str(),32);
    str=so[F("description")].as<String>();
    strncpy(out_info->description,str.c_str(),128);
    str=so[F("icon")].as<String>();
    strncpy(out_info->icon,str.c_str(),8);
    so = obj[F("main")].as<JsonObject>();
    out_info->temperature = so[F("temp")];
    out_info->feels_like  = so[F("feels_like")];
    out_info->pressure = so[F("pressure")];
    out_info->humidity = so[F("humidity")];
    so = obj[F("wind")].as<JsonObject>();
    out_info->wind_speed = so[F("speed")];
    out_info->wind_direction = so[F("deg")];
    out_info->wind_gust = so[F("gust")];
    so = obj[F("clouds")].as<JsonObject>();
    out_info->cloudiness = so[F("all")];
    so = obj[F("rain")];
    out_info->rain_last_hour = so[F("1h")];
    so = obj[F("snow")];
    out_info->snow_last_hour = so[F("1h")];
    so = obj[F("sys")];
    out_info->sunrise = (time_t)(long)so[F("sunrise")];
    out_info->sunset = (time_t)(long)so[F("sunset")];
    return true;
}
}