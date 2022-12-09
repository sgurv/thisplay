

#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2s_std.h"
#include "lvgl.h"


/* Display */
#define BSP_LCD_SPI_MOSI      (GPIO_NUM_11)
#define BSP_LCD_SPI_MISO      (GPIO_NUM_NC)
#define BSP_LCD_SPI_CLK       (GPIO_NUM_10)
#define BSP_LCD_SPI_CS        (GPIO_NUM_9)
#define BSP_LCD_DC            (GPIO_NUM_18)
#define BSP_LCD_RST           (GPIO_NUM_21)
#define BSP_LCD_BK_LIGHT      (GPIO_NUM_45)

#define LCD_BK_LIGHT_ON_LEVEL  1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL

/* Button */
#define BSP_BUTTONS_IO       (GPIO_NUM_6) // Push button

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
 *
 * LCD interface
 *
 * ESP32-S2-LCD Pico has 0.90inch ST7735 display controller.
 * It features 16-bit colors and 160x80 resolution.
 * Backlight control signal is disconnected on LCD board.
 *
 * LVGL is used as graphics library. LVGL is NOT thread safe, therefore the user must take LVGL mutex
 * by calling bsp_display_lock() before calling and LVGL API (lv_...) and then give the mutex with
 * bsp_display_unlock().
 **************************************************************************************************/
#define BSP_LCD_H_RES              (160)
#define BSP_LCD_V_RES              (80)
#define BSP_LCD_PIXEL_CLOCK_HZ     (20 * 1000 * 1000)
#define BSP_LCD_SPI_NUM            (SPI2_HOST)

#if CONFIG_BSP_ERROR_CHECK
#define BSP_ERROR_CHECK_RETURN_ERR(x)   ESP_ERROR_CHECK(x)
#define BSP_ERROR_CHECK_RETURN_NULL(x)  ESP_ERROR_CHECK(x)
#define BSP_NULL_CHECK(x, ret)          assert(x)
#else
#define BSP_ERROR_CHECK_RETURN_ERR(x) do { \
        esp_err_t err_rc_ = (x);            \
        if (unlikely(err_rc_ != ESP_OK)) {  \
            return err_rc_;                 \
        }                                   \
    } while(0)

#define BSP_ERROR_CHECK_RETURN_NULL(x)  do { \
        if (unlikely((x) != ESP_OK)) {      \
            return NULL;                    \
        }                                   \
    } while(0)

#define BSP_NULL_CHECK(x, ret)      do { \
        if ((x) == NULL) {      \
            return ret;                    \
        }                                   \
    } while(0)
#endif

//lv_disp_t *bsp_sem_test(void); //TEST

/**
 * @brief Initialize display and graphics library
 *
 * This function initializes SPI, display controller and starts LVGL handling task.
 *
 * @return Pointer to LVGL display or NULL when error occured
 */
lv_disp_t *bsp_display_start(void);

/**
 * @brief Take LVGL mutex
 *
 * @param timeout_ms Timeout in [ms]. 0 will block indefinitely.
 * @return true  Mutex was taken
 * @return false Mutex was NOT taken
 */
bool bsp_display_lock(uint32_t timeout_ms);

/**
 * @brief Give LVGL mutex
 *
 */
void bsp_display_unlock(void);

/* Backlight functions are not implemented - Kaluga board doesn't provide backlight control
   These functions are here to provide consistent API with other Board Support Packages */
esp_err_t bsp_display_backlight_on(void);
esp_err_t bsp_display_backlight_off(void);

/**
 * @brief Rotate screen
 *
 * Display must be already initialized by calling bsp_display_start()
 *
 * @param[in] disp Pointer to LVGL display
 * @param[in] rotation Angle of the display rotation
 */
void bsp_display_rotate(lv_disp_t *disp, lv_disp_rot_t rotation);

#ifdef __cplusplus
}
#endif
