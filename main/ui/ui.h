// SquareLine LVGL GENERATED FILE
// EDITOR VERSION: SquareLine Studio 1.2.0
// LVGL VERSION: 8.3.4
// PROJECT: thisplay

#ifndef _THISPLAY_UI_H
#define _THISPLAY_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

void ui_event_Screen1(lv_event_t * e);
extern lv_obj_t * ui_Screen1;
extern lv_obj_t * ui_Label1;
void ui_event_Button1(lv_event_t * e);
extern lv_obj_t * ui_Button1;
extern lv_obj_t * ui_Label2;
void ui_event_Button2(lv_event_t * e);
extern lv_obj_t * ui_Button2;
extern lv_obj_t * ui_Label3;
extern lv_obj_t * ui_Label4;
void ui_event_Screen2(lv_event_t * e);
extern lv_obj_t * ui_Screen2;
extern lv_obj_t * ui_Screen3;
extern lv_obj_t * ui_Screen4;
extern lv_obj_t * ui_Screen5;

void Screen1Event1Clicked(lv_event_t * e);
void Screen2Event1Clicked(lv_event_t * e);

LV_IMG_DECLARE(ui_img_abstract_png);    // assets\abstract.png
LV_IMG_DECLARE(ui_img_flower_png);    // assets\flower.png


LV_FONT_DECLARE(ui_font_Fontnum);


void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
