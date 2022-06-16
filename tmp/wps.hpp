#pragma once
#ifndef ESP32
#error "This library only supports the ESP32 MCU."
#endif
#include <Arduino.h>
#include <atomic>
#include <esp_wps.h>


std::atomic_int g_connect_state;
uint32_t g_connect_ts;
static esp_wps_config_t g_wps_config;
void (*fn_connect_callback)();
void (*fn_connect_disconnect_callback)();
// just initialize our WPS info
void wpsInitConfig() {
  //g_wps_config.crypto_funcs = &g_wifi_default_wps_crypto_funcs;
  g_wps_config.wps_type = WPS_TYPE_PBC;
  strcpy(g_wps_config.factory_info.manufacturer, "ESPRESSIF");
  strcpy(g_wps_config.factory_info.model_number, "ESP32");
  strcpy(g_wps_config.factory_info.model_name, "ESPRESSIF IOT");
  strcpy(g_wps_config.factory_info.device_name, "ESP32");
}
// not used yet
String wpspin2string(uint8_t a[]) {
  char wps_pin[9];
  for (int i = 0; i < 8; i++) {
    wps_pin[i] = a[i];
  }
  wps_pin[8] = '\0';
  return (String)wps_pin;
}

void WiFiEvent(WiFiEvent_t event, system_event_info_t info) {
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      g_connect_ts = millis();
      g_connect_state = 4; // connected
      break;
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
      esp_wifi_wps_disable();
      delay(10);
      g_connect_ts = millis();
      g_connect_state = 1; // connecting
      WiFi.begin();
      break;
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
      esp_wifi_wps_disable();
      g_connect_ts = millis();
      g_connect_state = 0; // connect
      break;
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
      esp_wifi_wps_disable();
      g_connect_ts = millis();
      g_connect_state = 0; // connect
      break;
    case SYSTEM_EVENT_STA_WPS_ER_PIN:
      // not used yet
      Serial.println("WPS_PIN = " + wpspin2string(info.sta_er_pin.pin_code));
      break;
    default:
      break;
  }
}
void beginNetworkAutoConnect(void (*connect_callback)(), void (*disconnect_callback)()) {
  // set up wifi
  wpsInitConfig();
  WiFi.onEvent(WiFiEvent);
  fn_connect_callback = connect_callback;
  fn_connect_disconnect_callback = disconnect_callback;
  g_connect_state = 0; // connect
  g_connect_ts = millis();
}
void continueNetworkAutoConnect() {
  bool gotSSID = false;
  File wfile;
  // we spin a state machine to handle
  // what we do when not connected:
  if (WL_CONNECTED != WiFi.status()) {
    switch (g_connect_state) {
      case 0: // connect start
        g_connect_ts = millis();
#ifdef WIFI_SSID
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
#else
        
        wfile = SD.open("/wifi.txt", "r");
        // TODO: This only accepts single line ssid and passwords and no trailing CRLFS in either. It cannot accept leading or trailing whitespace
        if (wfile && wfile.available()) {
          String ssid = wfile.readStringUntil('\n');
          ssid.trim();
          String pass = wfile.readStringUntil('\n');
          wfile.close();
          pass.trim();
          Serial.print("Read WiFi Credentials from SD - SSID: ");
          Serial.println(ssid);
          gotSSID = true;
          WiFi.begin(ssid.c_str(), pass.c_str());
        }
      
        if (!gotSSID)
          WiFi.begin();
#endif
        g_connect_state = 1; // connecting
        g_connect_ts = millis();
        break;
      case 1: // connect continue
        if (WL_CONNECTED == WiFi.status()) {
          g_connect_state = 4; // connected
          if (NULL != fn_connect_callback);
          (*fn_connect_callback)();
        } else if (millis() - g_connect_ts > (CONNECTION_TIMEOUT * 1000)) {
          WiFi.disconnect();
          g_connect_ts = millis();
#ifdef SSID
          // don't use WPS if we have an SSID
          _connect_state = 0;
#else
          g_connect_state = 2; // begin wps search
#endif
        }
        break;
      case 2: // begin WPS search
        g_connect_ts = millis();
        esp_wifi_wps_enable(&g_wps_config);
        esp_wifi_wps_start(0);
        g_connect_state = 3; // continue WPS search
        break;
      case 3: // continue WPS search
        // handled by callback
        break;
      case 4: // got disconnected
        g_connect_state = 1; // connecting
        if (NULL != fn_connect_disconnect_callback);
        (*fn_connect_disconnect_callback)();
        g_connect_ts = millis();
        WiFi.reconnect();
        break;
    }
  }
}
