#include <Arduino.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <gfx_cpp14.hpp>
#include <ili9341.hpp>
#include <tft_io.hpp>
#include <telegrama.hpp>
#include <wifi_wps.hpp>
#include <ip_loc.hpp>
#include <ntp_time.hpp>
#include <open_weather.hpp>
#include <draw_screen.hpp>
using namespace arduino;
using namespace gfx;

// NTP server

constexpr static const char* ntp_server = "time.nist.gov";

// synchronize with NTP every 60 seconds
constexpr static const int clock_sync_seconds = 60;

// synchronize with NTP every 5 minutes
constexpr static const int weather_sync_seconds = 60 * 5;

constexpr static const size16 clock_size = {120, 120};

constexpr static const size16 weather_icon_size = {48, 48};

constexpr static const size16 weather_temp_size = {120, 48};

constexpr static const uint8_t spi_host = VSPI;
constexpr static const int8_t lcd_pin_bl = 32;
constexpr static const int8_t lcd_pin_dc = 27;
constexpr static const int8_t lcd_pin_cs = 14;
constexpr static const int8_t spi_pin_mosi = 23;
constexpr static const int8_t spi_pin_clk = 18;
constexpr static const int8_t lcd_pin_rst = 33;
constexpr static const int8_t spi_pin_miso = 19;

using bus_t = tft_spi_ex<spi_host, 
                        lcd_pin_cs, 
                        spi_pin_mosi, 
                        spi_pin_miso, 
                        spi_pin_clk, 
                        SPI_MODE0,
                        false, 
                        320 * 240 * 2 + 8, 2>;
using lcd_t = ili9342c<lcd_pin_dc, 
                      lcd_pin_rst, 
                      lcd_pin_bl, 
                      bus_t, 
                      1, 
                      true, 
                      400, 
                      200>;
using color_t = color<typename lcd_t::pixel_type>;

lcd_t lcd;

#ifdef M5STACK
mpu6886 accel(i2c_container<0>::instance());
#endif

uint32_t update_ts;
uint32_t clock_sync_count;
uint32_t weather_sync_count;
time_t current_time;
srect16 clock_rect;
srect16 weather_icon_rect;
srect16 weather_temp_rect;
wifi_wps wps;
ntp_time ntp;
IPAddress ntp_ip;
float latitude;
float longitude;
long utc_offset;
char region[128];
char city[128];
open_weather_info weather_info;
using bmp_t = bitmap<typename lcd_t::pixel_type>;
uint8_t bmp_buf[bmp_t::sizeof_buffer(clock_size)];
bmp_t clock_bmp(clock_size, bmp_buf);
bmp_t weather_icon_bmp(weather_icon_size, bmp_buf);
bmp_t weather_temp_bmp(weather_temp_size, bmp_buf);
void setup() {
    Serial.begin(115200);
    SPIFFS.begin(false);
    lcd.fill(lcd.bounds(),color_t::white);
    Serial.println("Connecting...");
    while(WiFi.status()!=WL_CONNECTED) {
        wps.update();
    }
    clock_rect = srect16(spoint16::zero(), (ssize16)clock_size);
    clock_rect.offset_inplace(lcd.dimensions().width-clock_size.width ,lcd.dimensions().height-clock_size.height);
    weather_icon_rect=(srect16)weather_icon_size.bounds();
    weather_icon_rect.offset_inplace(20,20);
    weather_temp_rect = (srect16)weather_temp_size.bounds().offset(68,20);
    clock_sync_count = clock_sync_seconds;
    WiFi.hostByName(ntp_server,ntp_ip);
    ntp.begin_request(ntp_ip);
    while(ntp.requesting()) {
        ntp.update();
    }
    if(!ntp.request_received()) {
        Serial.println("Unable to retrieve time");
        while (true);
    }
    ip_loc::fetch(&latitude,&longitude,&utc_offset,region,128,city,128);
    weather_sync_count =1; // sync on next iteration
    Serial.println(weather_info.city);
    current_time = utc_offset+ntp.request_result();
    update_ts = millis();
}

void loop() {
    wps.update();
    ntp.update();
    if(ntp.request_received()) {
        Serial.println("NTP signal received");
        current_time = utc_offset+ntp.request_result();
    }
    uint32_t ms = millis();
    if (ms - update_ts >= 1000) {
        update_ts = ms;
        ++current_time;
        tm* t = localtime(&current_time);
        draw::wait_all_async(lcd);
        draw::filled_rectangle(clock_bmp, 
                              clock_size.bounds(), 
                              color_t::white);
        draw_clock(clock_bmp, *t, (ssize16)clock_size);
        draw::bitmap_async(lcd, 
                          clock_rect, 
                          clock_bmp, 
                          clock_bmp.bounds());
        if (0 == --clock_sync_count) {
            clock_sync_count = clock_sync_seconds;
            ntp.begin_request(ntp_ip);
        }
        if(0==--weather_sync_count) {
            weather_sync_count = weather_sync_seconds;
            open_weather::fetch(latitude,longitude,&weather_info);
            Serial.println("Fetched weather");
            Serial.println(weather_info.main);
            Serial.println(weather_info.icon);
            draw::wait_all_async(lcd);
            draw_weather_icon(weather_icon_bmp,weather_info,weather_icon_size);
            draw::bitmap(lcd, 
                          weather_icon_rect, 
                          weather_icon_bmp, 
                          weather_icon_bmp.bounds());
            weather_temp_bmp.fill(weather_temp_bmp.bounds(),color_t::white);
            draw_temps(weather_temp_bmp,weather_info,NAN);
            draw::bitmap(lcd, 
                          weather_temp_rect, 
                          weather_temp_bmp, 
                          weather_temp_bmp.bounds());
        }
    }
}