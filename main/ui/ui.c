// SquareLine LVGL GENERATED FILE
// EDITOR VERSION: SquareLine Studio 1.1.1
// LVGL VERSION: 8.3.3
// PROJECT: DigiClock

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////
lv_obj_t * ui_Screen1;
lv_obj_t * ui_Screen1_TextArea1;
lv_obj_t * ui_Screen2;
lv_obj_t * ui_Screen2_Label1;

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=1
    #error "LV_COLOR_16_SWAP should be 1 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////

///////////////////// SCREENS ////////////////////
void ui_Screen1_screen_init(void)
{
    ui_Screen1 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_text_color(ui_Screen1, lv_color_hex(0xDBE6F5), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Screen1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Screen1_TextArea1 = lv_textarea_create(ui_Screen1);
    lv_obj_set_width(ui_Screen1_TextArea1, 150);
    lv_obj_set_height(ui_Screen1_TextArea1, 70);
    lv_obj_set_align(ui_Screen1_TextArea1, LV_ALIGN_CENTER);
    lv_textarea_set_max_length(ui_Screen1_TextArea1, 256);
    lv_textarea_set_placeholder_text(ui_Screen1_TextArea1, "Log...");
    lv_obj_set_style_text_color(ui_Screen1_TextArea1, lv_color_hex(0xDBE6F5), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Screen1_TextArea1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

}
void ui_Screen2_screen_init(void)
{
    ui_Screen2 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Screen2_Label1 = lv_label_create(ui_Screen2);
    lv_obj_set_width(ui_Screen2_Label1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Screen2_Label1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Screen2_Label1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Screen2_Label1, "55:55");
    lv_obj_set_style_text_color(ui_Screen2_Label1, lv_color_hex(0x86B5F4), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Screen2_Label1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Screen2_Label1, &ui_font_DigiClock, LV_PART_MAIN | LV_STATE_DEFAULT);

}

void ui_init(void)
{
    lv_disp_t * dispp = lv_disp_get_default();
    lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                                               true, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    ui_Screen1_screen_init();
    ui_Screen2_screen_init();
    lv_disp_load_scr(ui_Screen1);
}
