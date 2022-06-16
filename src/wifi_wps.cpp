#include <wifi_wps.hpp>
#include <Wifi.h>
namespace arduino {
std::atomic_int wifi_wps_state;
uint32_t wifi_wps_connect_ts;
static esp_wps_config_t wifi_wps_config;
wifi_wps_callback fn_wifi_wps_callback;
void* wifi_wps_callback_state;
uint32_t wifi_wps_connect_timeout;
void wifi_wps_wifi_event(arduino_event_t* event) {
  switch (event->event_id) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      wifi_wps_connect_ts = millis();
      wifi_wps_state = 4; // connected
      break;
    case ARDUINO_EVENT_WPS_ER_SUCCESS:
      esp_wifi_wps_disable();
      delay(10);
      wifi_wps_connect_ts = millis();
      wifi_wps_state = 1; // connecting
      WiFi.begin();
      break;
    case ARDUINO_EVENT_WPS_ER_FAILED:
      esp_wifi_wps_disable();
      wifi_wps_connect_ts = millis();
      wifi_wps_state = 0; // connect
      break;
    case ARDUINO_EVENT_WPS_ER_TIMEOUT:
      esp_wifi_wps_disable();
      wifi_wps_connect_ts = millis();
      wifi_wps_state = 0; // connect
      break;
    case ARDUINO_EVENT_WPS_ER_PIN:
      // not used yet
      break;
    default:
      break;
  }
}
wifi_wps::wifi_wps(uint32_t connect_timeout) {
    wifi_wps_state = -1;
    fn_wifi_wps_callback = nullptr;
    wifi_wps_callback_state = nullptr;
    wifi_wps_connect_timeout = connect_timeout;
}
void wifi_wps::callback(wifi_wps_callback callback, void* state) {
    fn_wifi_wps_callback = callback;
    wifi_wps_callback_state = state;
    
}
void wifi_wps::update() {
    if (wifi_wps_state==-1) {
        wifi_wps_config.wps_type = WPS_TYPE_PBC;
        strcpy(wifi_wps_config.factory_info.manufacturer, "ESPRESSIF");
        strcpy(wifi_wps_config.factory_info.model_number, "ESP32");
        strcpy(wifi_wps_config.factory_info.model_name, "ESPRESSIF IOT");
        strcpy(wifi_wps_config.factory_info.device_name, "ESP32");
        WiFi.onEvent( wifi_wps_wifi_event);
        wifi_wps_connect_ts = millis();
        wifi_wps_state=0;
    }
    if(WiFi.status()!= WL_CONNECTED) {
        switch(wifi_wps_state) {
            case 0: // connect start
            wifi_wps_connect_ts = millis();
            WiFi.begin();
            wifi_wps_state = 1;
            wifi_wps_connect_ts = millis();
            break;
        case 1: // connect continue
            if(WiFi.status()==WL_CONNECTED) {
                wifi_wps_state = 4;
                if(fn_wifi_wps_callback != nullptr) {
                    fn_wifi_wps_callback(true,wifi_wps_callback_state);
                }
                Serial.println("WiFi connected to ");
                Serial.println(WiFi.SSID());
                
            } else if(millis()-wifi_wps_connect_ts>=wifi_wps_connect_timeout) {
                WiFi.disconnect();
                wifi_wps_connect_ts = millis();
                // begin wps_search
                wifi_wps_state = 2;
            }
            break;
        case 2: // WPS search
            wifi_wps_connect_ts = millis();
            esp_wifi_wps_enable(&wifi_wps_config);
            esp_wifi_wps_start(0);
            wifi_wps_state = 3; // continue WPS search
            break;
        case 3: // continue WPS search
            // handled by callback
            break;
        case 4:
            wifi_wps_state = 1; // connecting
            if(fn_wifi_wps_callback != nullptr) {
                fn_wifi_wps_callback(false,wifi_wps_callback_state);
            }
            wifi_wps_connect_ts = millis();
            WiFi.reconnect();
        }
    }
}
}  // namespace arduino