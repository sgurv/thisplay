#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_cst816d.h"

static const char *TAG = "CST816D";

/* CST816D registers TODO: complete it */
#define ESP_LCD_TOUCH_CST816D_GestureID (0x01) // Gesture register
#define ESP_LCD_TOUCH_CST816D_FingerNum (0x02) //Number of fingers
#define ESP_LCD_TOUCH_CST816D_XposH     (0x03) //x high four digits
#define ESP_LCD_TOUCH_CST816D_XposL     (0x04) //x low eight bits
#define ESP_LCD_TOUCH_CST816D_YposH     (0x05) // High four digits of y
#define ESP_LCD_TOUCH_CST816D_YposL     (0x06) //y lower eight bits
#define ESP_LCD_TOUCH_CST816D_ChipID    (0xA7) //chip model
#define ESP_LCD_TOUCH_CST816D_MotionMask (0xEC) //Trigger action
#define ESP_LCD_TOUCH_CST816D_AutoSleepTime (0xF9) //auto sleep
#define ESP_LCD_TOUCH_CST816D_IrqCrl    (0xFA) //interrupt control
#define ESP_LCD_TOUCH_CST816D_AutoReset (0xFB) //Sleep without gesture //write 0 to disable
#define ESP_LCD_TOUCH_CST816D_LongPressTime (0xFC) //Long press to sleep
#define ESP_LCD_TOUCH_CST816D_DisAutoSleep  (0xFE) //Enable low power mode

/*******************************************************************************
* Function definitions
*******************************************************************************/
static esp_err_t esp_lcd_touch_cst816d_read_data(esp_lcd_touch_handle_t tp);
static bool esp_lcd_touch_cst816d_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num);
static esp_err_t esp_lcd_touch_cst816d_del(esp_lcd_touch_handle_t tp);

/* I2C read/write */
static esp_err_t touch_cst816d_i2c_read(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t *data, uint8_t len);
static esp_err_t touch_cst816d_i2c_write(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t data);

/* cst816d reset */
static esp_err_t touch_cst816d_reset(esp_lcd_touch_handle_t tp);
/* Read status and config register */
//static esp_err_t touch_cst816d_read_cfg(esp_lcd_touch_handle_t tp);

/*******************************************************************************
* Public API functions
*******************************************************************************/

esp_err_t esp_lcd_touch_new_i2c_cst816d(const esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *config, esp_lcd_touch_handle_t *out_touch)
{
    esp_err_t ret = ESP_OK;

    assert(io != NULL);
    assert(config != NULL);
    assert(out_touch != NULL);

    /* Prepare main structure */
    esp_lcd_touch_handle_t esp_lcd_touch_cst816d = heap_caps_calloc(1, sizeof(esp_lcd_touch_t), MALLOC_CAP_DEFAULT);
    ESP_GOTO_ON_FALSE(esp_lcd_touch_cst816d, ESP_ERR_NO_MEM, err, TAG, "no mem for cst816d controller");

    /* Communication interface */
    esp_lcd_touch_cst816d->io = io;

    /* Only supported callbacks are set */
    esp_lcd_touch_cst816d->read_data = esp_lcd_touch_cst816d_read_data;
    esp_lcd_touch_cst816d->get_xy = esp_lcd_touch_cst816d_get_xy;
    esp_lcd_touch_cst816d->del = esp_lcd_touch_cst816d_del;

    /* Mutex */
    esp_lcd_touch_cst816d->data.lock.owner = portMUX_FREE_VAL;

    /* Save config */
    memcpy(&esp_lcd_touch_cst816d->config, config, sizeof(esp_lcd_touch_config_t));

    /* Prepare pin for touch interrupt */
    if (esp_lcd_touch_cst816d->config.int_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t int_gpio_config = {
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = BIT64(esp_lcd_touch_cst816d->config.int_gpio_num)
        };
        ret = gpio_config(&int_gpio_config);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "GPIO config failed");
    }

    /* Prepare pin for touch controller reset */
    if (esp_lcd_touch_cst816d->config.rst_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t rst_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = BIT64(esp_lcd_touch_cst816d->config.rst_gpio_num)
        };
        ret = gpio_config(&rst_gpio_config);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "GPIO config failed");
    }

    /* Reset controller */
    ret = touch_cst816d_reset(esp_lcd_touch_cst816d);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "cst816d reset failed");

    /* Read status and config info */
    //ret = touch_cst816d_read_cfg(esp_lcd_touch_cst816d); //Not required
    //ESP_GOTO_ON_ERROR(ret, err, TAG, "cst816d init failed");

err:
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error (0x%x)! Touch controller cst816d initialization failed!", ret);
        if (esp_lcd_touch_cst816d) {
            esp_lcd_touch_cst816d_del(esp_lcd_touch_cst816d);
        }
    }

    *out_touch = esp_lcd_touch_cst816d;

    return ret;
}

static esp_err_t esp_lcd_touch_cst816d_read_data(esp_lcd_touch_handle_t tp)
{
    esp_err_t err;
    uint8_t buf[6];
    //uint8_t touch_cnt = 0;
    //uint8_t clear = 0;
    size_t i = 0;

    //New
    uint16_t x,y;

    assert(tp != NULL);

    err = touch_cst816d_i2c_read(tp, ESP_LCD_TOUCH_CST816D_GestureID, buf,6);
    ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

    /*
    Gesture reg 0x01
    --Actually for CST816S
    0x00 = "none",
    0x01 = "slide down",
    0x02 = "slide up",
    0x03 = "slide left",
    0x04 = "slide right",
    0x05 = "single click",
    0x0B = "double click",
    0x0C = "long press",
    */

    /*
    reg addr
    0x02 = number of finger
    // Event (0 = Down, 1 = Up, 2 = Contact)
    */
    if(buf[1] == 0x01){ // some finger
    
        //if(((buf[2] & 0xf0) >> 6) == 0x00){ //Down 
            if((buf[4] >> 4) == 0x00){ //ID 0
                x = (uint16_t)((uint16_t)(buf[2] & 0x0f) << 8) + (uint16_t)buf[3];
                y = (uint16_t)((uint16_t)(buf[4] & 0x0f) << 8) + (uint16_t)buf[5];
                if(x<240 && y<280){ //OK
                    portENTER_CRITICAL(&tp->data.lock);

                    tp->data.points = 1; // using only 1 point for now
                    //data->action = buf[3] >> 6;
                    tp->data.coords[0].x = x;
                    tp->data.coords[0].y = y;
                    //tp->data.coords[i].strength = 255; // its null

                    portEXIT_CRITICAL(&tp->data.lock);
                }
            }
        //}
    }

    /* Any touch data? */
    // if ((buf[0] & 0x80) == 0x00) {
    //     //touch_cst816d_i2c_write(tp, ESP_LCD_TOUCH_CST816D_READ_XY_REG, clear);
    // } else {
    //     /* Count of touched points */
    //     touch_cnt = buf[0] & 0x0f;
    //     if (touch_cnt > 5 || touch_cnt == 0) {
    //         touch_cst816d_i2c_write(tp, ESP_LCD_TOUCH_CST816D_READ_XY_REG, clear);
    //         return ESP_OK;
    //     }

    //     /* Read all points */
    //     err = touch_cst816d_i2c_read(tp, ESP_LCD_TOUCH_CST816D_READ_XY_REG + 1, &buf[1], touch_cnt * 8);
    //     ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

    //     /* Clear all */
    //     //err = touch_cst816d_i2c_write(tp, ESP_LCD_TOUCH_CST816D_READ_XY_REG, clear);
    //     //ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

    //     portENTER_CRITICAL(&tp->data.lock);

    //     /* Number of touched points */
    //     touch_cnt = 1 ; //(touch_cnt > CONFIG_ESP_LCD_TOUCH_MAX_POINTS ? CONFIG_ESP_LCD_TOUCH_MAX_POINTS : touch_cnt);
    //     tp->data.points = touch_cnt;

    //     /* Fill all coordinates */
    //     for (i = 0; i < touch_cnt; i++) {
    //         tp->data.coords[i].x = ((uint16_t)buf[(i * 8) + 3] << 8) + buf[(i * 8) + 2];
    //         tp->data.coords[i].y = (((uint16_t)buf[(i * 8) + 5] << 8) + buf[(i * 8) + 4]);
    //         tp->data.coords[i].strength = (((uint16_t)buf[(i * 8) + 7] << 8) + buf[(i * 8) + 6]);
    //     }

    //     portEXIT_CRITICAL(&tp->data.lock);
    // }

    return ESP_OK;
}

static bool esp_lcd_touch_cst816d_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    assert(tp != NULL);
    assert(x != NULL);
    assert(y != NULL);
    assert(point_num != NULL);
    assert(max_point_num > 0);

    portENTER_CRITICAL(&tp->data.lock);

    /* Count of points */
    *point_num = (tp->data.points > max_point_num ? max_point_num : tp->data.points);

    for (size_t i = 0; i < *point_num; i++) {
        x[i] = tp->data.coords[i].x;
        y[i] = tp->data.coords[i].y;

        if (strength) {
            strength[i] = tp->data.coords[i].strength;
        }
    }

    /* Invalidate */
    tp->data.points = 0;

    portEXIT_CRITICAL(&tp->data.lock);

    return (*point_num > 0);
}

static esp_err_t esp_lcd_touch_cst816d_del(esp_lcd_touch_handle_t tp)
{
    assert(tp != NULL);

    /* Reset GPIO pin settings */
    if (tp->config.int_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.int_gpio_num);
    }

    /* Reset GPIO pin settings */
    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.rst_gpio_num);
    }

    free(tp);

    return ESP_OK;
}

/*******************************************************************************
* Private API function
*******************************************************************************/

/* Reset controller */
static esp_err_t touch_cst816d_reset(esp_lcd_touch_handle_t tp)
{
    assert(tp != NULL);

    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, !tp->config.levels.reset), TAG, "GPIO set level error!");
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, tp->config.levels.reset), TAG, "GPIO set level error!");
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return ESP_OK;
}

// static esp_err_t touch_cst816d_read_cfg(esp_lcd_touch_handle_t tp)
// {
//     uint8_t buf[4];

//     assert(tp != NULL);

//     ESP_RETURN_ON_ERROR(touch_cst816d_i2c_read(tp, ESP_LCD_TOUCH_CST816D_PRODUCT_ID_REG, (uint8_t *)&buf[0], 3), TAG, "cst816d read error!");
//     ESP_RETURN_ON_ERROR(touch_cst816d_i2c_read(tp, ESP_LCD_TOUCH_CST816D_CONFIG_REG, (uint8_t *)&buf[3], 1), TAG, "cst816d read error!");

//     ESP_LOGI(TAG, "TouchPad_ID:0x%02x,0x%02x,0x%02x", buf[0], buf[1], buf[2]);
//     ESP_LOGI(TAG, "TouchPad_Config_Version:%d", buf[3]);

//     return ESP_OK;
// }

static esp_err_t touch_cst816d_i2c_read(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t *data, uint8_t len)
{
    assert(tp != NULL);
    assert(data != NULL);

    /* Read data */
    return esp_lcd_panel_io_rx_param(tp->io, reg, data, len);
}

static esp_err_t touch_cst816d_i2c_write(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t data)
{
    assert(tp != NULL);

    // *INDENT-OFF*
    /* Write data */
    return esp_lcd_panel_io_tx_param(tp->io, reg, (uint8_t[]){data}, 1);
    // *INDENT-ON*
}
