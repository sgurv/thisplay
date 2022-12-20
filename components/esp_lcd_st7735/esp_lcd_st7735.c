
// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include <stdio.h>
#include <stdlib.h>
#include <sys/cdefs.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_lcd_panel_interface.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_commands.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_check.h>

#include "esp_lcd_st7735.h"

typedef struct
{
    uint8_t cmd;
    uint8_t data[16];
    uint8_t data_bytes;
} lcd_init_cmd_t;

#define     ST7735_INIT_LENGTH_MASK     0x1F
#define     ST7735_INIT_DONE_FLAG       0xFF

static const char *TAG = "st7735";

static esp_err_t panel_st7735_del(esp_lcd_panel_t *panel);
static esp_err_t panel_st7735_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_st7735_init(esp_lcd_panel_t *panel);
static esp_err_t panel_st7735_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_st7735_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_st7735_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_st7735_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_st7735_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_st7735_disp_on_off(esp_lcd_panel_t *panel, bool off);

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    uint8_t fb_bits_per_pixel;
    uint8_t madctl_val; // save current value of LCD_CMD_MADCTL register
    uint8_t colmod_cal; // save surrent value of LCD_CMD_COLMOD register
} st7735_panel_t;

esp_err_t esp_lcd_new_panel_st7735(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    st7735_panel_t *st7735 = NULL;
    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    st7735 = calloc(1, sizeof(st7735_panel_t));
    ESP_GOTO_ON_FALSE(st7735, ESP_ERR_NO_MEM, err, TAG, "no mem for st7735 panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    switch (panel_dev_config->color_space) {
    case ESP_LCD_COLOR_SPACE_RGB:
        st7735->madctl_val = 0;
        break;
    case ESP_LCD_COLOR_SPACE_BGR:
        st7735->madctl_val |= ST7735_PARAM_MADCTL_BGR;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported color space");
        break;
    }

    uint8_t fb_bits_per_pixel = 0;
    switch (panel_dev_config->bits_per_pixel) {
    case 16: // RGB565
        st7735->colmod_cal = ST7735_PARAM_COLMOD_IFPF_16B_PER_PIXEL;
        fb_bits_per_pixel = 16;
        break;
    case 18: // RGB666
        st7735->colmod_cal = ST7735_PARAM_COLMOD_IFPF_18B_PER_PIXEL;
        // each color component (R/G/B) should occupy the 6 high bits of a byte, which means 3 full bytes are required for a pixel
        fb_bits_per_pixel = 24;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

    st7735->io = io;
    st7735->fb_bits_per_pixel = fb_bits_per_pixel;
    st7735->reset_gpio_num = panel_dev_config->reset_gpio_num;
    st7735->reset_level = panel_dev_config->flags.reset_active_high;
    st7735->base.del = panel_st7735_del;
    st7735->base.reset = panel_st7735_reset;
    st7735->base.init = panel_st7735_init;
    st7735->base.draw_bitmap = panel_st7735_draw_bitmap;
    st7735->base.invert_color = panel_st7735_invert_color;
    st7735->base.set_gap = panel_st7735_set_gap;
    st7735->base.mirror = panel_st7735_mirror;
    st7735->base.swap_xy = panel_st7735_swap_xy;
    st7735->base.disp_on_off = panel_st7735_disp_on_off;
    *ret_panel = &(st7735->base);
    ESP_LOGD(TAG, "new st7735 panel @%p", st7735);

    return ESP_OK;

err:
    if (st7735) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(st7735);
    }
    return ret;
}

static esp_err_t panel_st7735_del(esp_lcd_panel_t *panel)
{
    st7735_panel_t *st7735 = __containerof(panel, st7735_panel_t, base);

    if (st7735->reset_gpio_num >= 0) {
        gpio_reset_pin(st7735->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del st7735 panel @%p", st7735);
    free(st7735);
    return ESP_OK;
}

static esp_err_t panel_st7735_reset(esp_lcd_panel_t *panel)
{
    st7735_panel_t *st7735 = __containerof(panel, st7735_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7735->io;

    // perform hardware reset
    if (st7735->reset_gpio_num >= 0) {
        gpio_set_level(st7735->reset_gpio_num, st7735->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(st7735->reset_gpio_num, !st7735->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    } else { // perform software reset
        esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(20)); // spec, wait at least 5m before sending new command
    }

    return ESP_OK;
}

static esp_err_t panel_st7735_init(esp_lcd_panel_t *panel)
{
    st7735_panel_t *st7735 = __containerof(panel, st7735_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7735->io;

    // esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
    //     st7735->madctl_val,
    // }, 1);
    // esp_lcd_panel_io_tx_param(io, LCD_CMD_COLMOD, (uint8_t[]) {
    //     st7735->colmod_cal,
    // }, 1);
    

    const lcd_init_cmd_t st7735_init[] =
    {   /*cmd,data[],data_len*/
        //{ST7735_CMD_SWRESET,{},0,150},
        //{ST7735_CMD_SLPOUT,{},0,255/*500ms*/},
        /*Part 1*/
        {ST7735_CMD_FRMCTR1,{0x01, 0x2C, 0x2D},3},
        {ST7735_CMD_FRMCTR2,{0x01, 0x2C, 0x2D},3},
        {ST7735_CMD_FRMCTR3,{0x01, 0x2C, 0x2D,0x01, 0x2C, 0x2D},6},
        {ST7735_CMD_INVCTR,{0x07},1},
        {ST7735_CMD_PWCTR1,{0xA2,0x02,0x84},3},
        {ST7735_CMD_PWCTR2,{0xC5},1},
        {ST7735_CMD_PWCTR3,{0x0A,0x00},2},
        {ST7735_CMD_PWCTR4,{0x8A,0x2A},2},
        {ST7735_CMD_PWCTR5,{0x8A, 0xEE},2},
        {ST7735_CMD_VMCTR1,{0x0E},1},
        {ST7735_CMD_INVOFF,{},0},

        {ST7735_CMD_MADCTL,{st7735->madctl_val},1}, // use st7735->madctl_val after test
        {ST7735_CMD_COLMOD,{st7735->colmod_cal},1}, // .. st7735->colmod_cal
        /*Part 2*/
        {ST7735_CMD_CASET,{0x00,0x00,0x00,0x4F},4},
        {ST7735_CMD_RASET,{0x00,0x00,0x00,0x9F},4},
        {ST7735_CMD_INVON,{},0},
        /*Part 3*/
        {ST7735_CMD_GAMCTRP1,{0x02, 0x1c, 0x07, 0x12,
                            0x37, 0x32, 0x29, 0x2d,
                            0x29, 0x25, 0x2B, 0x39,
                            0x00, 0x01, 0x03, 0x10},16},
        {ST7735_CMD_GAMCTRN1,{0x03, 0x1d, 0x07, 0x06,
                            0x2E, 0x2C, 0x29, 0x2D,
                            0x2E, 0x2E, 0x37, 0x3F,
                            0x00, 0x00, 0x02, 0x10},16},
        {LCD_CMD_NOP, { 0 }, ST7735_INIT_DONE_FLAG }
    };

    //1. Hardware Reset

    //2. Execute Command Lists

    //Software reset
    esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(150)); // spec, wait at least 5m before sending new command

    // LCD goes into sleep mode and display will be turned off after power on reset, exit sleep mode first
    esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    //Loop thru lcd commands
    ESP_LOGI(TAG, "ST7735 command loop");
    int cmd = 0;
    while ( st7735_init[cmd].data_bytes != ST7735_INIT_DONE_FLAG )
    {
        ESP_LOGD(TAG, "Sending CMD: %02x, len: %d", st7735_init[cmd].cmd,
                 st7735_init[cmd].data_bytes & ST7735_INIT_LENGTH_MASK);
        esp_lcd_panel_io_tx_param(
            io, st7735_init[cmd].cmd, st7735_init[cmd].data,
            st7735_init[cmd].data_bytes & ST7735_INIT_LENGTH_MASK);
        cmd++;
    }

    //3. Turn on display
    esp_lcd_panel_io_tx_param(io, LCD_CMD_NORON, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_lcd_panel_io_tx_param(io, LCD_CMD_DISPON, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}

static esp_err_t panel_st7735_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    st7735_panel_t *st7735 = __containerof(panel, st7735_panel_t, base);
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");
    esp_lcd_panel_io_handle_t io = st7735->io;

    x_start += st7735->x_gap;
    x_end += st7735->x_gap;
    y_start += st7735->y_gap;
    y_end += st7735->y_gap;

    // define an area of frame memory where MCU can access
    esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET, (uint8_t[]) {
        (x_start >> 8) & 0xFF,
        x_start & 0xFF,
        ((x_end - 1) >> 8) & 0xFF,
        (x_end - 1) & 0xFF,
    }, 4);
    esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET, (uint8_t[]) {
        (y_start >> 8) & 0xFF,
        y_start & 0xFF,
        ((y_end - 1) >> 8) & 0xFF,
        (y_end - 1) & 0xFF,
    }, 4);
    // transfer frame buffer
    size_t len = (x_end - x_start) * (y_end - y_start) * st7735->fb_bits_per_pixel / 8;
    esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len);

    return ESP_OK;
}

static esp_err_t panel_st7735_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    st7735_panel_t *st7735 = __containerof(panel, st7735_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7735->io;
    int command = 0;
    if (invert_color_data) {
        command = LCD_CMD_INVON;
    } else {
        command = LCD_CMD_INVOFF;
    }
    esp_lcd_panel_io_tx_param(io, command, NULL, 0);
    return ESP_OK;
}

static esp_err_t panel_st7735_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    st7735_panel_t *st7735 = __containerof(panel, st7735_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7735->io;
    if (mirror_x) {
        st7735->madctl_val |= LCD_CMD_MX_BIT;
    } else {
        st7735->madctl_val &= ~LCD_CMD_MX_BIT;
    }
    if (mirror_y) {
        st7735->madctl_val |= LCD_CMD_MY_BIT;
    } else {
        st7735->madctl_val &= ~LCD_CMD_MY_BIT;
    }
    esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        st7735->madctl_val
    }, 1);
    return ESP_OK;
}

static esp_err_t panel_st7735_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    st7735_panel_t *st7735 = __containerof(panel, st7735_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7735->io;
    if (swap_axes) {
        st7735->madctl_val |= LCD_CMD_MV_BIT;
    } else {
        st7735->madctl_val &= ~LCD_CMD_MV_BIT;
    }
    esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        st7735->madctl_val
    }, 1);
    return ESP_OK;
}

static esp_err_t panel_st7735_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    st7735_panel_t *st7735 = __containerof(panel, st7735_panel_t, base);
    st7735->x_gap = x_gap;
    st7735->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_st7735_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    st7735_panel_t *st7735 = __containerof(panel, st7735_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7735->io;
    int command = 0;
    if (on_off) {
        command = LCD_CMD_DISPON;
    } else {
        command = LCD_CMD_DISPOFF;
    }
    esp_lcd_panel_io_tx_param(io, command, NULL, 0);
    return ESP_OK;
}
