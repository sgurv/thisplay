#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "bsp_thisplay.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_st7735.h"
#include "esp_lcd_touch_cst816d.h"

static const char *TAG = "bspPico";

static esp_lcd_touch_handle_t tp = NULL;            // LCD touch panel handle
static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
static lv_disp_drv_t disp_drv;      // contains callback functions
static SemaphoreHandle_t lvgl_mux;  // LVGL mutex

// Bit number used to represent command and parameter
#define LCD_CMD_BITS           (8)
#define LCD_PARAM_BITS         (8)
#define LVGL_TICK_PERIOD_MS    (CONFIG_BSP_DISPLAY_LVGL_TICK)
#define LVGL_BUFF_SIZE_PIX     (BSP_LCD_H_RES * 10)

#define LVGL_LARGE_BUFF_SIZE_PIX     (BSP_LCD_LARGE_H_RES * 20)

static bool lvgl_port_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void lvgl_port_flush_callback(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    const int offsetx1 = area->x1;
    const int offsetx2 = area->x2;
    const int offsety1 = area->y1;
    const int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void lvgl_port_update_callback(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;

    switch (drv->rotated) {
    case LV_DISP_ROT_NONE:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    case LV_DISP_ROT_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISP_ROT_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISP_ROT_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    }
}

static void lvgl_port_tick_increment(void *arg)
{
    /* Tell LVGL how many milliseconds have elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
    
}

static esp_err_t lvgl_port_tick_init(void)
{
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvgl_port_tick_increment,
        .name = "LVGL tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    BSP_ERROR_CHECK_RETURN_ERR(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    return esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000);
}

static void lvgl_port_task(void *arg)
{
    //ESP_LOGI(TAG, "Starting LVGL task");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(20));
        bsp_display_lock(0);
        lv_timer_handler();
        bsp_display_unlock();

    }
}

/**************************************************************************************************
 *
 * I2C Function
 *
 **************************************************************************************************/
esp_err_t bsp_i2c_init(void)
{
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BSP_TP_SDA,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = BSP_TP_SCL,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = 400 * 1000//CONFIG_BSP_I2C_CLK_SPEED_HZ
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_param_config(BSP_TP_I2C_NUM, &i2c_conf));
    BSP_ERROR_CHECK_RETURN_ERR(i2c_driver_install(BSP_TP_I2C_NUM, i2c_conf.mode, 0, 0, 0));

    return ESP_OK;
}

esp_err_t bsp_i2c_deinit(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(i2c_driver_delete(BSP_TP_I2C_NUM));
    return ESP_OK;
}

/**************************************************************************************************
 *
 * Touch Panel Function
 *
 **************************************************************************************************/
esp_lcd_touch_handle_t bsp_touch_panel_init(void)
{
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_touch_handle_t tp_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816D_CONFIG();
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = BSP_LCD_LARGE_H_RES,
        .y_max = BSP_LCD_LARGE_V_RES,
        .rst_gpio_num = BSP_TP_RST,
        .int_gpio_num = BSP_TP_INT,
        .levels = {
            .reset = 0, //Reset level
            .interrupt = 1, //Active High
        },
        .flags = { //Match it with the display settings
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)BSP_TP_I2C_NUM, &tp_io_config, &tp_io_handle));
    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_touch_new_i2c_cst816d(tp_io_handle, &tp_cfg, &tp_handle));

    return tp_handle;
}

static void bsp_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t)indev_drv->user_data;
    assert(tp);

    uint16_t touchpad_x;
    uint16_t touchpad_y;
    uint8_t touchpad_cnt = 0;
    /* Read data from touch controller into memory */
    esp_lcd_touch_read_data(tp);

    /* Read data from touch controller */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(tp, &touchpad_x, &touchpad_y, NULL, &touchpad_cnt, 1);
    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x;
        data->point.y = touchpad_y;
        data->state = LV_INDEV_STATE_PRESSED;
        ESP_LOGD(TAG, "Touch position: %d,%d", touchpad_x, touchpad_y);
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static esp_err_t lvgl_port_indev_init(void)
{
    static lv_indev_drv_t indev_drv_tp;
    lv_indev_t *indev_touchpad;

    /* Initialize touch panel in sub board */
    tp = bsp_touch_panel_init();
    BSP_NULL_CHECK(tp, ESP_FAIL);

    /* Register a touchpad input device */
    lv_indev_drv_init(&indev_drv_tp);
    indev_drv_tp.type = LV_INDEV_TYPE_POINTER;
    indev_drv_tp.read_cb = bsp_touchpad_read;
    //indev_drv_tp.feedback_cb = ;
    indev_drv_tp.user_data = tp;
    indev_touchpad = lv_indev_drv_register(&indev_drv_tp);
    BSP_NULL_CHECK(indev_touchpad, ESP_ERR_NO_MEM);

    //--pointer
    lv_obj_t * btn = lv_btn_create(lv_scr_act());     /*Add a button the current screen*/
    lv_obj_set_size(btn, 8, 8); 
    // lv_obj_t *cursor;
    // cursor = lv_obj_create(lv_scr_act());
    // lv_obj_set_size(cursor, 8, 8);
    // static lv_style_t style_round;
    // //lv_style_set_bg_color(&style_round,LV_PALETTE_RED);
    // lv_style_set_bg_opa(&style_round,LV_OPA_COVER);
    // lv_style_set_radius(&style_round,8);
    // lv_obj_add_style(cursor, &style_round,1);
    //lv_obj_set_click(cursor, false);
    lv_indev_set_cursor(indev_touchpad, btn);

    return ESP_OK;
}

lv_disp_t *bsp_display_start(void){


    //small LCD backlight
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << BSP_LCD_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    gpio_set_level(BSP_LCD_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL);

    //large LCD backlight
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_lg_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << BSP_LCD_LARGE_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_lg_gpio_config));

    gpio_set_level(BSP_LCD_LARGE_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL);

    lv_init();

    //lvgl_port_display_init();

 #ifndef USE_LARGE_DISPLAY
    // ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = BSP_LCD_SPI_CLK,
        .mosi_io_num = BSP_LCD_SPI_MOSI,
        .miso_io_num = BSP_LCD_SPI_MISO,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = LVGL_BUFF_SIZE_PIX * sizeof(lv_color_t), /* buffer_size * sizeof(lv_color_t) */
    };

#else

    spi_bus_config_t buscfg = {
        .sclk_io_num = BSP_LCD_LARGE_CLK,
        .data0_io_num = BSP_LCD_LARGE_SIO_0,
        .data1_io_num = BSP_LCD_LARGE_SIO_1,
        .data2_io_num = BSP_LCD_LARGE_SIO_2,
        .data3_io_num = BSP_LCD_LARGE_SIO_3,
        .max_transfer_sz = LVGL_LARGE_BUFF_SIZE_PIX * sizeof(lv_color_t), /* buffer_size * sizeof(lv_color_t) */
        .flags = SPICOMMON_BUSFLAG_QUAD /* QSPI */
    };

#endif

    ESP_ERROR_CHECK(spi_bus_initialize(BSP_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO));

    // ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;

#ifndef USE_LARGE_DISPLAY
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BSP_LCD_DC,
        .cs_gpio_num = BSP_LCD_SPI_CS,
        .pclk_hz = BSP_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = lvgl_port_flush_ready,
        .user_ctx = &disp_drv,
    };

#else

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BSP_LCD_LARGE_DC,
        .cs_gpio_num = BSP_LCD_LARGE_CS,
        .pclk_hz = BSP_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = lvgl_port_flush_ready,
        .user_ctx = &disp_drv,
    };

#endif

    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config, &io_handle));

    // ESP_LOGI(TAG, "Install panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;

#ifndef USE_LARGE_DISPLAY    
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST,
        .color_space = ESP_LCD_COLOR_SPACE_BGR,
        .bits_per_pixel = 16,
    };

    //ST7735 LCD init
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7735(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle,1,26)); //TODO: Check why this offset required 

    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));

    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true)); //Why true?
    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
#else

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_LARGE_RST,
        .rgb_endian = ESP_LCD_COLOR_SPACE_BGR,
        .bits_per_pixel = 16,
    };

    //ST7789 LCD init
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    //ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));

    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle,0,20)); //TODO: Adjust, Check why this offset required

    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

    //ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true)); //Why true?
    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

#endif


    //alloc draw buffers used by LVGL
#ifndef USE_LARGE_DISPLAY
    //it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(LVGL_BUFF_SIZE_PIX * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    
    lv_color_t *buf2 = heap_caps_malloc(LVGL_BUFF_SIZE_PIX * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);

    //initialize LVGL draw buffers

    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LVGL_BUFF_SIZE_PIX);
#else

    //it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(LVGL_LARGE_BUFF_SIZE_PIX * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    
    lv_color_t *buf2 = heap_caps_malloc(LVGL_LARGE_BUFF_SIZE_PIX * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);

    //initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LVGL_LARGE_BUFF_SIZE_PIX);
#endif

    // ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);

#ifndef USE_LARGE_DISPLAY
    disp_drv.hor_res = BSP_LCD_H_RES;
    disp_drv.ver_res = BSP_LCD_V_RES;
#else
    disp_drv.hor_res = BSP_LCD_LARGE_H_RES;
    disp_drv.ver_res = BSP_LCD_LARGE_V_RES;
#endif

    disp_drv.flush_cb = lvgl_port_flush_callback;
    disp_drv.drv_update_cb = lvgl_port_update_callback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
    //END lvgl_port_display_init();

    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_tick_init());

    // Indev
    lvgl_port_indev_init();
    // End Indev

    lvgl_mux = xSemaphoreCreateMutex();

    xTaskCreate(lvgl_port_task, "LVGLtask", 4096, NULL, CONFIG_BSP_DISPLAY_LVGL_TASK_PRIORITY, NULL);

#ifndef USE_LARGE_DISPLAY
    //Tun on baclight by default
    gpio_set_level(BSP_LCD_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL); 
#else
    //Tun on baclight by default
    gpio_set_level(BSP_LCD_LARGE_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL); 
#endif
    return disp;
}

void bsp_display_rotate(lv_disp_t *disp, lv_disp_rot_t rotation)
{
    lv_disp_set_rotation(disp, rotation);
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    assert(lvgl_mux && "bsp_display_start must be called first");

    const TickType_t timeout_ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

void bsp_display_unlock(void)
{
    assert(lvgl_mux && "bsp_display_start must be called first");
    xSemaphoreGive(lvgl_mux);
}


esp_err_t bsp_display_backlight_off(void)
{
#ifndef USE_LARGE_DISPLAY    
    gpio_set_level(BSP_LCD_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL); 
#else
    gpio_set_level(BSP_LCD_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL);
#endif
    return ESP_OK;
}
esp_err_t bsp_display_backlight_on(void)
{
#ifndef USE_LARGE_DISPLAY
    gpio_set_level(BSP_LCD_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
#else
    gpio_set_level(BSP_LCD_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
#endif
    return ESP_OK;
}

