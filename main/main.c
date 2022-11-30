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
//#include "esp_lvgl_port.h"

static const char *TAG = "ws";

// SPI2(according to ESP-IDF) connected to LCD in Waveshare-Pico LCD
#define LCD_HOST  SPI2_HOST

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Update the following configuration according to the LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define LCD_PIXEL_CLOCK_HZ     (10 * 1000 * 1000)
#define LCD_BK_LIGHT_ON_LEVEL  1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL
#define LCD_PIN_NUM_SCLK           10
#define LCD_PIN_NUM_MOSI           11
#define LCD_PIN_NUM_MISO           GPIO_NUM_NC /*not used*/
#define LCD_PIN_NUM_LCD_DC         18
#define LCD_PIN_NUM_LCD_RST        21
#define LCD_PIN_NUM_LCD_CS         9
#define LCD_PIN_NUM_BK_LIGHT       45
#define LCD_PIN_NUM_TOUCH_CS       GPIO_NUM_NC

// The pixel number in horizontal and vertical
#define LCD_H_RES              160
#define LCD_V_RES              80
// Bit number used to represent command and parameter
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8

#define LVGL_TICK_PERIOD_MS    2

#define BLINK_TEST_GPIO     36 /* For debuging purpose*/

//static lv_disp_t *disp;

//LED global
lv_obj_t * led1;

static bool lcd_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void lcd_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void lcd_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

void app_main(void)
{
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    // const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    // esp_err_t err = lvgl_port_init(&lvgl_cfg);

    ESP_LOGI(TAG, "Test GPIO LED!");
    gpio_reset_pin(BLINK_TEST_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_TEST_GPIO, GPIO_MODE_OUTPUT);

    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LCD_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    gpio_set_level(LCD_PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_PIN_NUM_SCLK,
        .mosi_io_num = LCD_PIN_NUM_MOSI,
        .miso_io_num = LCD_PIN_NUM_MISO,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = LCD_H_RES * 20 * sizeof(lv_color_t), /* buffer_size * sizeof(lv_color_t) */
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_PIN_NUM_LCD_DC,
        .cs_gpio_num = LCD_PIN_NUM_LCD_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = lcd_notify_lvgl_flush_ready,
        .user_ctx = &disp_drv, //not disp drive for esp_lvgl_port
    };

    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install ST7735 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_PIN_NUM_LCD_RST,
        .color_space = ESP_LCD_COLOR_SPACE_RGB, //TODO: complete it
        .bits_per_pixel = 16,
    };

    //I guess ST7789 is very similar to ST7735
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    /**********LVGL PORT here*/

    /* Add LCD screen */
    // const lvgl_port_display_cfg_t disp_cfg = {
    //     .io_handle = io_handle,
    //     .panel_handle = panel_handle,
    //     .buffer_size = LCD_H_RES * 20,
    //     .double_buffer = true,
    //     .hres = LCD_H_RES,
    //     .vres = LCD_V_RES,
    //     .monochrome = false,
    //     /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
    //     .rotation = {
    //         .swap_xy = false,
    //         .mirror_x = false,
    //         .mirror_y = false,
    //     },
    //     .flags = {
    //         .buff_dma = true,
    //     }
    // };

    // disp = lvgl_port_add_disp(&disp_cfg);

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    //ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));

    //ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_off(panel_handle, true));

    // ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(LCD_PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    //alloc draw buffers used by LVGL
    //it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    
    lv_color_t *buf2 = heap_caps_malloc(LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    //initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * 20);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lcd_lvgl_flush_cb;
    //disp_drv.drv_update_cb = example_lvgl_port_update_callback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lcd_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

//----UI
    /* Wait for the other task done the screen operation */
//    lvgl_port_lock(0);
    //Button 1
    lv_obj_t * btn = lv_btn_create(lv_scr_act());     /*Add a button the current screen*/
    lv_obj_set_pos(btn, 5, 5);                            /*Set its position*/
    lv_obj_set_size(btn, 70, 30);                          /*Set its size*/
    //lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/

    lv_obj_t * label = lv_label_create(btn);          /*Add a label to the button*/
    lv_label_set_text(label, "Button A");                     /*Set the labels text*/
    lv_obj_center(label);

    /*Create a LED and switch it OFF*/
    led1  = lv_led_create(lv_scr_act());
    lv_obj_set_pos(led1, 35, 50);
    //lv_obj_align(led1, LV_ALIGN_CENTER, -80, 0);
    lv_led_set_color(led1, lv_palette_main(LV_PALETTE_LIME));
    lv_led_off(led1);

    /* Screen operation done -> release for the other task */
//    lvgl_port_unlock();
//----END UI

    while (1) {
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(10));
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        lv_timer_handler();
        //gpio_set_level(BLINK_TEST_GPIO, LCD_BK_LIGHT_ON_LEVEL);
        //vTaskDelay(pdMS_TO_TICKS(10));
        //gpio_set_level(BLINK_TEST_GPIO, LCD_BK_LIGHT_OFF_LEVEL);
    }

}
