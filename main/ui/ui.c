// SquareLine LVGL GENERATED FILE
// EDITOR VERSION: SquareLine Studio 1.1.1
// LVGL VERSION: 8.3.3
// PROJECT: DigiClock

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////
void ui_event_Screen1(lv_event_t * e);
lv_obj_t * ui_Screen1;
lv_obj_t * ui_Screen1_Label1;
lv_obj_t * ui_Screen1_Label2;
lv_obj_t * ui_Screen1_Label3;
void ui_event_Screen2(lv_event_t * e);
lv_obj_t * ui_Screen2;
lv_obj_t * ui_Screen2_Slider1;
lv_obj_t * ui_Screen2_Switch1;
void ui_event_Screen3(lv_event_t * e);
lv_obj_t * ui_Screen3;
lv_obj_t * ui_Screen3_Roller1;
void ui_event_Screen4(lv_event_t * e);
lv_obj_t * ui_Screen4;
lv_obj_t * ui_Screen4_Button1;
void ui_event_Screen5(lv_event_t * e);
lv_obj_t * ui_Screen5;
lv_obj_t * ui_Screen5_TextArea1;

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=1
    #error "LV_COLOR_16_SWAP should be 1 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_Screen1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        EventScreen1Clicked(e);
    }
    if(event_code == LV_EVENT_LONG_PRESSED) {
        _ui_screen_change(ui_Screen2, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0);
    }
}
void ui_event_Screen2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        EventScreen2Clicked(e);
    }
    if(event_code == LV_EVENT_LONG_PRESSED) {
        _ui_screen_change(ui_Screen3, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0);
    }
}
void ui_event_Screen3(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        EventScreen3Clicked(e);
    }
    if(event_code == LV_EVENT_LONG_PRESSED) {
        _ui_screen_change(ui_Screen4, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0);
    }
}
void ui_event_Screen4(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        EventScreen4Clicked(e);
    }
    if(event_code == LV_EVENT_LONG_PRESSED) {
        _ui_screen_change(ui_Screen5, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0);
    }
}
void ui_event_Screen5(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        EventScreen5Clicked(e);
    }
    if(event_code == LV_EVENT_LONG_PRESSED) {
        _ui_screen_change(ui_Screen1, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0);
    }
}

///////////////////// SCREENS ////////////////////
void ui_Screen1_screen_init(void)
{
    ui_Screen1 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Screen1_Label1 = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_Screen1_Label1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Screen1_Label1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Screen1_Label1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Screen1_Label1, "55:55");
    lv_obj_set_style_text_color(ui_Screen1_Label1, lv_color_hex(0x86B5F4), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Screen1_Label1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Screen1_Label1, &ui_font_DigiClock, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Screen1_Label2 = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_Screen1_Label2, 158);
    lv_obj_set_height(ui_Screen1_Label2, 15);
    lv_obj_set_align(ui_Screen1_Label2, LV_ALIGN_TOP_RIGHT);
    lv_label_set_text(ui_Screen1_Label2, "Alarm");
    lv_obj_set_style_text_align(ui_Screen1_Label2, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Screen1_Label3 = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_Screen1_Label3, 158);
    lv_obj_set_height(ui_Screen1_Label3, 15);
    lv_obj_set_align(ui_Screen1_Label3, LV_ALIGN_BOTTOM_MID);
    lv_label_set_text(ui_Screen1_Label3, "Sun 7th of Jan, 2023");
    lv_obj_set_style_text_align(ui_Screen1_Label3, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_Screen1, ui_event_Screen1, LV_EVENT_ALL, NULL);

}
void ui_Screen2_screen_init(void)
{
    ui_Screen2 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_text_color(ui_Screen2, lv_color_hex(0xDBE6F5), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Screen2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Screen2_Slider1 = lv_slider_create(ui_Screen2);
    lv_obj_set_width(ui_Screen2_Slider1, 120);
    lv_obj_set_height(ui_Screen2_Slider1, 5);
    lv_obj_set_x(ui_Screen2_Slider1, 0);
    lv_obj_set_y(ui_Screen2_Slider1, 22);
    lv_obj_set_align(ui_Screen2_Slider1, LV_ALIGN_CENTER);

    ui_Screen2_Switch1 = lv_switch_create(ui_Screen2);
    lv_obj_set_width(ui_Screen2_Switch1, 50);
    lv_obj_set_height(ui_Screen2_Switch1, 25);
    lv_obj_set_x(ui_Screen2_Switch1, 4);
    lv_obj_set_y(ui_Screen2_Switch1, -8);
    lv_obj_set_align(ui_Screen2_Switch1, LV_ALIGN_CENTER);
    lv_obj_add_state(ui_Screen2_Switch1, LV_STATE_CHECKED);       /// States

    lv_obj_add_event_cb(ui_Screen2, ui_event_Screen2, LV_EVENT_ALL, NULL);

}
void ui_Screen3_screen_init(void)
{
    ui_Screen3 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Screen3_Roller1 = lv_roller_create(ui_Screen3);
    lv_roller_set_options(ui_Screen3_Roller1, "1\n2\n3\n4\n5\n6\n7\n8\n9\n10\n11\n12\n13\n14\n15\n16",
                          LV_ROLLER_MODE_INFINITE);
    lv_obj_set_height(ui_Screen3_Roller1, 80);
    lv_obj_set_width(ui_Screen3_Roller1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_align(ui_Screen3_Roller1, LV_ALIGN_CENTER);

    lv_obj_add_event_cb(ui_Screen3, ui_event_Screen3, LV_EVENT_ALL, NULL);

}
void ui_Screen4_screen_init(void)
{
    ui_Screen4 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Screen4_Button1 = lv_btn_create(ui_Screen4);
    lv_obj_set_width(ui_Screen4_Button1, 100);
    lv_obj_set_height(ui_Screen4_Button1, 50);
    lv_obj_set_align(ui_Screen4_Button1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Screen4_Button1, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Screen4_Button1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    lv_obj_add_event_cb(ui_Screen4, ui_event_Screen4, LV_EVENT_ALL, NULL);

}
void ui_Screen5_screen_init(void)
{
    ui_Screen5 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Screen5_TextArea1 = lv_textarea_create(ui_Screen5);
    lv_obj_set_width(ui_Screen5_TextArea1, 156);
    lv_obj_set_height(ui_Screen5_TextArea1, 80);
    lv_obj_set_align(ui_Screen5_TextArea1, LV_ALIGN_TOP_MID);
    lv_textarea_set_placeholder_text(ui_Screen5_TextArea1, "Placeholder...");

    lv_obj_add_event_cb(ui_Screen5, ui_event_Screen5, LV_EVENT_ALL, NULL);

}

void ui_init(void)
{
    lv_disp_t * dispp = lv_disp_get_default();
    lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                                               true, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    ui_Screen1_screen_init();
    ui_Screen2_screen_init();
    ui_Screen3_screen_init();
    ui_Screen4_screen_init();
    ui_Screen5_screen_init();
    lv_disp_load_scr(ui_Screen1);
}
