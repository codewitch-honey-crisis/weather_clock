#pragma once
#include <Arduino.h>
#include <SPIFFS.h>
#include <open_weather.hpp>
#include <gfx.hpp>
#include <telegrama.hpp>
template <typename Destination>
void draw_weather_icon(Destination& dst, arduino::open_weather_info& info,gfx::size16 weather_icon_size) {
    const char* path;
    if(!strcmp(info.icon,"01d")) {
        path = "/sun.jpg";
    } else if(!strcmp(info.icon,"01n")) {
        path = "/moon.jpg";
    } else if(!strcmp(info.icon,"02d")) {
        path = "/partly.jpg";
    } else if(!strcmp(info.icon,"02n")) {
        path = "/partlyn.jpg";
    } else if(!strcmp(info.icon,"10d") || !strcmp(info.icon,"10n")) {
        path = "/rain.jpg";
    } else if(!strcmp(info.icon,"04d") || !strcmp(info.icon,"04n")) {
        path = "/cloud.jpg";
    } else if(!strcmp(info.icon,"03d") || !strcmp(info.icon,"03n")) {
        if(info.snow_last_hour>0) {
            path = "/snow.jpg";
        } else if(info.rain_last_hour>0) {
            path = "/rain.jpg";
        } else {
            path = "/cloud.jpg";
        }
    } else {
        path = nullptr;
    }
    
    if(path!=nullptr) {
        File file = SPIFFS.open(path,"rb");
        gfx::draw::image( dst,dst.bounds(),&file);
        file.close();
    } else {
        Serial.print("Icon not recognized: ");
        Serial.println(info.icon);
        Serial.println(info.main);
        gfx::draw::filled_rectangle(dst,
                                    weather_icon_size.bounds(),
                                    gfx::color<typename Destination::pixel_type>::white);
    }
}
template <typename Destination>
void draw_temps(Destination& dst, arduino::open_weather_info& info, float inside) {
    gfx::open_text_info tinfo;
    char sz[32];
    float tmpF = info.temperature*1.8+32;
    sprintf(sz,"%.1fF",tmpF);
    tinfo.scale = Telegrama_otf.scale(24);
    tinfo.text = sz;
    tinfo.font = &Telegrama_otf;
    gfx::draw::text(dst,
                    dst.bounds().offset(0,12*(inside!=inside)),
                    tinfo,
                    gfx::color<typename Destination::pixel_type>::black);
    if(inside==inside) {
        tmpF = inside*1.8+32;
        sprintf(sz,"%.1fF",tmpF);
        gfx::draw::text(dst,
                        dst.bounds().offset(0,dst.dimensions().height/2).crop(dst.bounds()),
                        tinfo,
                        gfx::color<typename Destination::pixel_type>::blue);
    }
}
template <typename Destination>
void draw_clock(Destination& dst, time_t time, const gfx::ssize16& size) {
    using view_t = gfx::viewport<Destination>;
    gfx::srect16 b = size.bounds().normalize();
    uint16_t w = min(b.width(), b.height());
    
    float txt_scale = Telegrama_otf.scale(w/10);
    char* sz = asctime(localtime(&time));
    *(sz+3)=0;
    gfx::draw::text(dst,
            dst.bounds(),
            gfx::spoint16::zero(), 
            sz,
            Telegrama_otf,
            txt_scale,
            gfx::color<typename Destination::pixel_type>::black);
    sz+=4;
    *(sz+6)='\0';

    gfx::ssize16 tsz = Telegrama_otf.measure_text(gfx::ssize16::max(),
                                                gfx::spoint16::zero(),
                                                sz,
                                                txt_scale);
    gfx::draw::text(dst,
            dst.bounds().offset(dst.dimensions().width-tsz.width-1,0).crop(dst.bounds()),
            gfx::spoint16::zero(), 
            sz,
            Telegrama_otf,
            txt_scale,
            gfx::color<typename Destination::pixel_type>::black);

    gfx::srect16 sr(0, 0, w / 20, w / 5);
    sr.center_horizontal_inplace(b);
    view_t view(dst);
    view.center(gfx::spoint16(w / 2, w / 2));
    static const float rot_step = 360.0/12.0;
    bool toggle = false;
    for (float rot = 0; rot < 360; rot += rot_step) {
        view.rotation(rot);
        toggle = !toggle;
        if(toggle) {
            gfx::spoint16 marker_points1[] = {
                view.translate(gfx::spoint16(sr.x1, sr.y1)),
                view.translate(gfx::spoint16(sr.x2, sr.y1)),
                view.translate(gfx::spoint16(sr.x2, sr.y2)),
                view.translate(gfx::spoint16(sr.x1, sr.y2))};
            gfx::spath16 marker_path1(4, marker_points1);
            gfx::draw::filled_polygon(dst, marker_path1, 
                gfx::color<typename Destination::pixel_type>::gray);
            
        } else {
            gfx::spoint16 marker_points2[] = {
                view.translate(gfx::spoint16(sr.x1, sr.y1)),
                view.translate(gfx::spoint16(sr.x2, sr.y1)),
                view.translate(gfx::spoint16(sr.x2, sr.y2-sr.height()/2)),
                view.translate(gfx::spoint16(sr.x1, sr.y2-sr.height()/2))};
            gfx::spath16 marker_path2(4, marker_points2);
            gfx::draw::filled_polygon(dst, marker_path2, 
                gfx::color<typename Destination::pixel_type>::gray);    
        }
        
    }
    sr = gfx::srect16(0, 0, w / 16, w / 2);
    sr.center_horizontal_inplace(b);
    view.rotation(((time%60) / 60.0) * 360.0);
    gfx::spoint16 second_points[] = {
        view.translate(gfx::spoint16(sr.x1+sr.width()/2, sr.y1)),
        //view.translate(gfx::spoint16(sr.x2, sr.y1)),
        view.translate(gfx::spoint16(sr.x2, sr.y2)),
        view.translate(gfx::spoint16(sr.x1, sr.y2))};
    gfx::spath16 second_path(3, second_points);

    view.rotation((((time/60)%60)/ 60.0) * 360.0);
    gfx::spoint16 minute_points[] = {
        view.translate(gfx::spoint16(sr.x1+sr.width()/2, sr.y1)),
        //view.translate(gfx::spoint16(sr.x2, sr.y1)),
        view.translate(gfx::spoint16(sr.x2, sr.y2)),
        view.translate(gfx::spoint16(sr.x1, sr.y2))};
    gfx::spath16 minute_path(3, minute_points);

    sr.y1 += w / 8;
    view.rotation(((int(time/(3600.0)+.5)%(12)) / (12.0)) * 360.0);
    gfx::spoint16 hour_points[] = {
        view.translate(gfx::spoint16(sr.x1+sr.width()/2, sr.y1)),
        //view.translate(gfx::spoint16(sr.x2, sr.y1)),
        view.translate(gfx::spoint16(sr.x2, sr.y2)),
        view.translate(gfx::spoint16(sr.x1, sr.y2))};
    gfx::spath16 hour_path(3, hour_points);

    gfx::draw::filled_polygon(dst, 
                        minute_path, 
                        gfx::color<typename Destination::pixel_type>::black);

    gfx::draw::filled_polygon(dst, 
                        hour_path, 
                        gfx::color<typename Destination::pixel_type>::black);

    gfx::draw::filled_polygon(dst, 
                        second_path, 
                        gfx::color<typename Destination::pixel_type>::red);
}
