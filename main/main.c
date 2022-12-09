#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_lcd_st7735.h"
#include "bsp_thisplay.h"

static const char *TAG = "ws";

#define BLINK_TEST_GPIO     36 /* For debuging purpose*/

static lv_disp_t *disp;

// //LED global
lv_obj_t * led1;


void app_main(void)
{
 
    ESP_LOGI(TAG, "Test GPIO LED!");
    gpio_reset_pin(BLINK_TEST_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_TEST_GPIO, GPIO_MODE_OUTPUT);

    disp = bsp_display_start();

// //----UI
//     /* Wait for the other task done the screen operation */
    bsp_display_lock(0);
    //Button 1
    lv_obj_t * btn = lv_btn_create(lv_scr_act());     /*Add a button the current screen*/
    lv_obj_set_pos(btn, 5, 5);                            /*Set its position*/
    lv_obj_set_size(btn, 70, 30);                          /*Set its size*/
    //lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/

    lv_obj_t * label = lv_label_create(btn);          /*Add a label to the button*/
    lv_label_set_text(label, "Button");                     /*Set the labels text*/
    lv_obj_center(label);

    /*Create a LED and switch it OFF*/
    led1  = lv_led_create(lv_scr_act());
    lv_obj_set_pos(led1, 20, 50);
    //lv_obj_align(led1, LV_ALIGN_CENTER, -80, 0);
    lv_led_set_color(led1, lv_palette_main(LV_PALETTE_AMBER));
    lv_led_on(led1);

    const char * data = "Sandeep Guria";

    /*Create a 80x80 QR code*/
    lv_obj_t * qr = lv_qrcode_create(lv_scr_act(), 80, lv_color_hex3(0x33f), lv_color_hex3(0xeef));
    lv_obj_set_pos(qr, 80, 0);
    /*Set data*/
    lv_qrcode_update(qr, data, strlen(data));

    // /* Screen operation done -> release for the other task */
    bsp_display_unlock();
//----END UI

    unsigned int i = 0;
    while (1) {
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(10));
        bsp_display_lock(0);
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        //lv_timer_handler();
        if(i++ == 100){
            lv_led_toggle(led1);
            i = 0;
        }
        bsp_display_unlock();
    }

}
