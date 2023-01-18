// SquareLine LVGL GENERATED FILE
// EDITOR VERSION: SquareLine Studio 1.2.0
// LVGL VERSION: 8.3.4
// PROJECT: thisplay

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////
void ui_event_Screen1(lv_event_t * e);
lv_obj_t * ui_Screen1;
lv_obj_t * ui_Label1;
void ui_event_Button1(lv_event_t * e);
lv_obj_t * ui_Button1;
lv_obj_t * ui_Label2;
void ui_event_Button2(lv_event_t * e);
lv_obj_t * ui_Button2;
lv_obj_t * ui_Label3;
lv_obj_t * ui_Label4;
void ui_event_Screen2(lv_event_t * e);
lv_obj_t * ui_Screen2;
lv_obj_t * ui_Screen3;
lv_obj_t * ui_Screen4;
lv_obj_t * ui_Screen5;

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
        Screen1Event1Clicked(e);
    }
}
void ui_event_Button1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(ui_Screen2, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0);
    }
}
void ui_event_Button2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(ui_Screen3, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0);
    }
}
void ui_event_Screen2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        Screen2Event1Clicked(e);
    }
}

///////////////////// SCREENS ////////////////////
void ui_Screen1_screen_init(void)
{
    ui_Screen1 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_img_src(ui_Screen1, &ui_img_abstract_png, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label1 = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_Label1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label1, -2);
    lv_obj_set_y(ui_Label1, -48);
    lv_obj_set_align(ui_Label1, LV_ALIGN_CENTER);
    lv_obj_set_style_text_font(ui_Label1, &ui_font_Fontnum, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button1 = lv_btn_create(ui_Screen1);
    lv_obj_set_width(ui_Button1, 100);
    lv_obj_set_height(ui_Button1, 30);
    lv_obj_set_x(ui_Button1, -50);
    lv_obj_set_y(ui_Button1, 100);
    lv_obj_set_align(ui_Button1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button1, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label2 = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_Label2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label2, -51);
    lv_obj_set_y(ui_Label2, 99);
    lv_obj_set_align(ui_Label2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label2, "Menu");

    ui_Button2 = lv_btn_create(ui_Screen1);
    lv_obj_set_width(ui_Button2, 100);
    lv_obj_set_height(ui_Button2, 30);
    lv_obj_set_x(ui_Button2, 57);
    lv_obj_set_y(ui_Button2, 100);
    lv_obj_set_align(ui_Button2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button2, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label3 = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_Label3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label3, 54);
    lv_obj_set_y(ui_Label3, 99);
    lv_obj_set_align(ui_Label3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label3, "Next");

    ui_Label4 = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_Label4, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label4, -6);
    lv_obj_set_y(ui_Label4, -6);
    lv_obj_set_align(ui_Label4, LV_ALIGN_CENTER);

    lv_obj_add_event_cb(ui_Button1, ui_event_Button1, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button2, ui_event_Button2, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Screen1, ui_event_Screen1, LV_EVENT_ALL, NULL);

}
void ui_Screen2_screen_init(void)
{
    ui_Screen2 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_img_src(ui_Screen2, &ui_img_flower_png, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_Screen2, ui_event_Screen2, LV_EVENT_ALL, NULL);

}
void ui_Screen3_screen_init(void)
{
    ui_Screen3 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    //lv_obj_set_style_bg_img_src(ui_Screen3, &ui_img_fictitious_png, LV_PART_MAIN | LV_STATE_DEFAULT);

}
void ui_Screen4_screen_init(void)
{
    ui_Screen4 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

}
void ui_Screen5_screen_init(void)
{
    ui_Screen5 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

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
