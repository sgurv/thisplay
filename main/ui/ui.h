// SquareLine LVGL GENERATED FILE
// EDITOR VERSION: SquareLine Studio 1.1.1
// LVGL VERSION: 8.3.3
// PROJECT: DigiClock

#ifndef _DIGICLOCK_UI_H
#define _DIGICLOCK_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

void ui_event_Screen1(lv_event_t * e);
extern lv_obj_t * ui_Screen1;
extern lv_obj_t * ui_Screen1_Label1;
extern lv_obj_t * ui_Screen1_Label2;
extern lv_obj_t * ui_Screen1_Label3;
void ui_event_Screen2(lv_event_t * e);
extern lv_obj_t * ui_Screen2;
extern lv_obj_t * ui_Screen2_Slider1;
extern lv_obj_t * ui_Screen2_Switch1;
void ui_event_Screen3(lv_event_t * e);
extern lv_obj_t * ui_Screen3;
extern lv_obj_t * ui_Screen3_Roller1;
void ui_event_Screen4(lv_event_t * e);
extern lv_obj_t * ui_Screen4;
extern lv_obj_t * ui_Screen4_Button1;
void ui_event_Screen5(lv_event_t * e);
extern lv_obj_t * ui_Screen5;
extern lv_obj_t * ui_Screen5_TextArea1;

void EventScreen1Clicked(lv_event_t * e);
void EventScreen2Clicked(lv_event_t * e);
void EventScreen3Clicked(lv_event_t * e);
void EventScreen4Clicked(lv_event_t * e);
void EventScreen5Clicked(lv_event_t * e);



LV_FONT_DECLARE(ui_font_DigiClock);


void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
