#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <lvgl.h>
#include <ESP_Panel_Library.h>
#include <ESP_IOExpander_Library.h>
#include <ui.h>
#include <lv_conf.h>
#include <functional>

/* LVGL porting configurations */
#define LVGL_TICK_PERIOD_MS     (2)
#define LVGL_TASK_MAX_DELAY_MS  (500)
#define LVGL_TASK_MIN_DELAY_MS  (1)
#define LVGL_BUF_SIZE           (ESP_PANEL_LCD_H_RES * 20) // 800*20 

#define TP_RST 1
#define LCD_BL 2
#define LCD_RST 3
#define SD_CS 4
#define USB_SEL 5

// I2C Pin define
#define I2C_MASTER_NUM 0
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_SCL_IO 9

#define LVGL_TASKNAME             "lvgl"
#define LVGL_TASK_STACK_SIZE      4 * 1024
#define LVGL_TASK_PRIORITY        10
#define LVGL_COREID               0

#define USE_LCD_TOUCH       0

class Display
{
private:
    static HardwareSerial *debugPort; // Biến tĩnh để giữ tham chiếu
    static ESP_Panel *panel;
    static ESP_IOExpander *expander;
    static SemaphoreHandle_t lvgl_mux;                  // LVGL mutex
    TaskHandle_t TaskDisplay_Handler;
    static void lvgl_port_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
    static bool notify_lvgl_flush_ready(void *user_ctx);
    static void lvgl_port_tp_read(lv_indev_drv_t * indev, lv_indev_data_t * data);
    static void lvgl_port_lock(int timeout_ms);
    static void lvgl_port_unlock(void);
    static void lvgl_port_task(void *arg);
    static void unit_test(uint32_t *_time);

public:
    Display(HardwareSerial &debugPort);
    ~Display();
    bool init();
};

#endif // 