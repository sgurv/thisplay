#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lvgl.h"
//#include "esp_lcd_st7735.h"
#include "bsp_thisplay.h"
#include "ui/ui.h"

#define EXAMPLE_ESP_WIFI_SSID "opt_ind"
#define EXAMPLE_ESP_WIFI_PASS "shree987"

#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK

static const char *TAG = "ws";

#define BLINK_TEST_GPIO     36 /* For debuging purpose*/

#define BUTTON_0_GPIO     0 /* For button 0 -- this is BOOT pin*/

static lv_disp_t *disp;

// //LED global
//lv_obj_t * led1;

char clock_str_buff[8];
char log_str_buff[128];

static void button_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);
static int button_get_pressed_id(void);


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        snprintf(log_str_buff,50,"2. wifi event STA start");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        snprintf(log_str_buff,50,"3. wifi event STA disconnect");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        snprintf(log_str_buff,100,"4. wifi event got ip address\nIP:" IPSTR,IP2STR(&event->ip_info.ip));
    }
}

/*Will be called by the library to read the button*/
static void button_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{

    static uint8_t last_btn = 0;

    /*Get the pressed button's ID*/
    int8_t btn_act = button_get_pressed_id();

    if(btn_act >= 0) {
        data->state = LV_INDEV_STATE_PR;
        last_btn = btn_act;
    }
    else {
        data->state = LV_INDEV_STATE_REL;
    }

    /*Save the last pressed button's ID*/
    data->btn_id = last_btn;
}

/*Get ID  (0, 1, 2 ..) of the pressed button*/
static int button_get_pressed_id(void)
{
    uint8_t i;

    /*Check to buttons see which is being pressed (assume there are 2 buttons)*/
    for(i = 0; i < 2; i++) {
        /*Return the pressed button's ID*/
        if(gpio_get_level(BUTTON_0_GPIO) == 0) { //if pressed
            return 0; //return button id 0
        }

        //.... check other buttons
    }

    /*No button pressed*/
    return -1;
}

void app_main(void)
{
 
    // ESP_LOGI(TAG, "Test GPIO LED!");
    // gpio_reset_pin(BLINK_TEST_GPIO);
    // /* Set the GPIO as a push/pull output */
    // gpio_set_direction(BLINK_TEST_GPIO, GPIO_MODE_OUTPUT);

    gpio_reset_pin(BUTTON_0_GPIO);
    /* Set the GPIO as a input */
    gpio_set_direction(BUTTON_0_GPIO, GPIO_MODE_INPUT);

    //---Wifi

    /* Initialize NVS partition */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS partition was truncated
         * and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());

        /* Retry nvs_flash_init */
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg)); //hangs-due to USB CDC for debug trace

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
	     * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    //--Display
    disp = bsp_display_start();

// //----UI
//     /* Wait for the other task done the screen operation */
    bsp_display_lock(0);
    // //Button 1
    // lv_obj_t * btn = lv_btn_create(lv_scr_act());     /*Add a button the current screen*/
    // lv_obj_set_pos(btn, 5, 5);                            /*Set its position*/
    // lv_obj_set_size(btn, 70, 30);                          /*Set its size*/
    // //lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/

    // lv_obj_t * label = lv_label_create(btn);          /*Add a label to the button*/
    // lv_label_set_text(label, "Button");                     /*Set the labels text*/
    // lv_obj_center(label);

    // /*Create a LED and switch it OFF*/
    // led1  = lv_led_create(lv_scr_act());
    // lv_obj_set_pos(led1, 20, 50);
    // //lv_obj_align(led1, LV_ALIGN_CENTER, -80, 0);
    // lv_led_set_color(led1, lv_palette_main(LV_PALETTE_AMBER));
    // lv_led_on(led1);

    // const char * data = "Sandeep Guria";

    // /*Create a 80x80 QR code*/
    // lv_obj_t * qr = lv_qrcode_create(lv_scr_act(), 80, lv_color_hex3(0x33f), lv_color_hex3(0xeef));
    // lv_obj_set_pos(qr, 80, 0);
    // /*Set data*/
    // lv_qrcode_update(qr, data, strlen(data));

    /*Register a button input device*/
    lv_indev_t * indev_button;

    lv_indev_drv_t indev_drv;

    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_BUTTON;
    indev_drv.read_cb = button_read;
    indev_button = lv_indev_drv_register(&indev_drv);

    /*Assign buttons to points on the screen*/
    static const lv_point_t btn_points[2] = {
        {0, 0},   /*Button 0 -> x:0; y:0*/
        {120, 20},  /*Button 1 -> not used*/
    };

    lv_indev_set_button_points(indev_button, btn_points);


    //Squareline
    ui_init();
    //default display log
    snprintf(log_str_buff,50,"1. Display log started");
    lv_textarea_set_text(ui_Screen5_TextArea1, log_str_buff);
    //lv_disp_load_scr(ui_Screen2);
    // /* Screen operation done -> release for the other task */
    bsp_display_unlock();
//----END UI

    unsigned int i = 0;
    uint8_t hour,minute;
    hour = 10;
    minute = 15;
    while (1) {
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(10));
        bsp_display_lock(0);
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        //lv_timer_handler();
        if(i++ == 100){
            //lv_led_toggle(led1);
            if(++minute == 60){
                if(++hour == 24){
                    hour = 0;
                }
                minute = 0;
            }
            snprintf(clock_str_buff,8,"%02d:%02d",hour,minute);
            lv_label_set_text(ui_Screen1_Label1, clock_str_buff);

            lv_textarea_set_text(ui_Screen5_TextArea1, log_str_buff);

            i = 0;
        }
        bsp_display_unlock();
    }

}
