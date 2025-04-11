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

#define BATTERY_DJI_CELL_NUM    14
#define BMS_CELL_NUM    16

#define DISPLAY_UNIT_TEST       !TRUE

#define NUM_OBJ_VALUE   20
#define NUM_OBJ_TITLE   20
#define NUM_OBJ         20
#define NUM_STYLE       20
#define NUM_FONT        34

LV_IMG_DECLARE(ecodrone_resize);
LV_IMG_DECLARE(symbol_clock);
LV_IMG_DECLARE(product_name);
LV_IMG_DECLARE(charging);
LV_FONT_DECLARE(vni_font);
LV_FONT_DECLARE(vni_number);
LV_FONT_DECLARE(dji_gotham);
LV_FONT_DECLARE(vni_gotham_bold);
LV_FONT_DECLARE(vni_gotham_bold_70);
LV_FONT_DECLARE(vni_gotham_regular);
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

    static void on_create_lv_label(lv_obj_t **obj,
                                    lv_style_t *style,
                                    const lv_font_t *font, 
                                    lv_align_t align,
                                    lv_coord_t x_ofs, 
                                    lv_coord_t y_ofs);
    static void on_change_lv_label_text(lv_obj_t *obj, String str);
    static void on_create_lv_img(lv_obj_t **obj, 
                                const void *src, 
                                int16_t zoom,
                                lv_align_t align, 
                                lv_coord_t x_ofs, 
                                lv_coord_t y_ofs);
    static void on_create_lv_line(lv_obj_t **obj,
                                lv_style_t *style,
                                lv_point_t line_points[],
                                lv_align_t align,
                                lv_coord_t x_ofs, 
                                lv_coord_t y_ofs);

    static lv_obj_t *g_title[NUM_OBJ_TITLE];
    static lv_obj_t *g_value[NUM_OBJ_VALUE];
    static lv_obj_t *g_obj[NUM_OBJ];
    static lv_style_t g_style[NUM_STYLE];
    static lv_style_t g_font[NUM_FONT];

    // BOOT_SCREEN -> logo animation and to screen 2
    struct BOOT_PAGE
    {
        static lv_obj_t *logo;
        static lv_anim_t* animation;
        static void onCreate()
        {
            /* Tạo hình ảnh */
            on_create_lv_img(&logo, &ecodrone_resize, -1, LV_ALIGN_CENTER, 0, 0); 
            lv_anim_t* animation = new lv_anim_t; // Tạo hoạt động mới
            lv_anim_init(animation);
            lv_anim_set_time(animation, 3000); 
            lv_anim_set_exec_cb(animation, onAnimationZoom); // Đặt hàm thực thi
            lv_anim_set_var(animation, logo); // Đặt đối tượng để hoạt động
            lv_anim_set_values(animation, 0, 650); // Từ 50px đến 200px
            lv_anim_set_playback_time(animation, 0); // Không cần phát lại
            lv_anim_set_ready_cb(animation, onDelete); // Đặt hàm callback khi hoàn thành animation
            lv_anim_start(animation); // Bắt đầu hoạt động
            lv_obj_clear_flag(lv_scr_act(),LV_OBJ_FLAG_SCROLLABLE);
        }

        static void onDelete(lv_anim_t* anim)
        {
            lv_anim_del_all();
            vTaskDelay((1500L * configTICK_RATE_HZ) / 1000L);
            lv_obj_del(static_cast<lv_obj_t*>(anim->var));
            choose_page = PAGE::MAIN;
            FIX_FRAME::onCreate();
        }

        static void onAnimationZoom(void * img, int32_t value)
        {
            lv_img_set_zoom(static_cast<lv_obj_t*>(img), value); 
        }
    };

    struct FIX_FRAME
    {
        static lv_obj_t *lb_status_obj;
        static String lb_status_str;
        static lv_obj_t *img_clock_obj;
        static lv_obj_t *lb_clock_obj;
        static lv_style_t font_vni_n;
        static lv_style_t main_panel_style;
        static lv_obj_t *main_panel;
        static bool checkCreate;
        static lv_obj_t *logo_charge;
        static lv_style_t font_vni_g_b;
        static void onCreate()
        {
            if(checkCreate)
            {
                // Top panel
                // logo
                on_create_lv_img(&BOOT_PAGE::logo, &ecodrone_resize, 100, LV_ALIGN_TOP_LEFT, -45, -40);
                // status label
                // on_create_lv_img(&lb_status_obj, &product_name, 500, LV_ALIGN_TOP_MID, 0, ESP_PANEL_LCD_V_RES*0.068*0.8);
                // product name
                on_create_lv_label(&lb_status_obj, &font_vni_g_b, &vni_gotham_bold, LV_ALIGN_TOP_MID, 0, 5);
                lv_label_set_text(lb_status_obj, "ECO CHARGER");
                // clock
                on_create_lv_label(&lb_clock_obj, &font_vni_n, &vni_number, LV_ALIGN_TOP_RIGHT, -10, ESP_PANEL_LCD_V_RES*0.068*0.5);
                lv_label_set_text(lb_clock_obj, "0:00");

                lv_style_init(&main_panel_style);
                lv_style_set_radius(&main_panel_style, 8);
                lv_style_set_border_width(&main_panel_style, 4);
                lv_style_set_border_color(&main_panel_style, lv_color_make(0xB9, 0xB9, 0xB9));
                main_panel = lv_obj_create(lv_scr_act());
                lv_obj_add_style(main_panel, &main_panel_style, 0);
                lv_obj_set_size(main_panel, ESP_PANEL_LCD_H_RES*0.99, ESP_PANEL_LCD_V_RES*0.86);
                lv_obj_align(main_panel, LV_ALIGN_CENTER, 0, ESP_PANEL_LCD_V_RES*0.068);
                lv_obj_clear_flag(main_panel,LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_clear_flag(lv_scr_act(),LV_OBJ_FLAG_SCROLLABLE);
                checkCreate = false;
            }
        }

        static void onUpdate(String clock_str)
        {
            on_change_lv_label_text(lb_clock_obj, clock_str);
        }

        static void onDelete()
        {
            // logo left
            lv_obj_del(BOOT_PAGE::logo);
            // status label
            lv_obj_remove_style_all(lb_status_obj);
            lv_obj_del(lb_status_obj);
            lv_style_reset(&font_vni_g_b);
            lv_style_reset(&font_vni_n);
            // clock system
            lv_obj_del(img_clock_obj);
            lv_obj_del(logo_charge);
            // Main Panel
            lv_style_reset(&main_panel_style);
            lv_obj_remove_style_all(main_panel); lv_obj_del(main_panel);
            checkCreate = true;
        }
    };

    struct MAIN_PAGE
    {
        static uint8_t step;

        enum STEP
        {
            CREATE,
            UPDATE,
            DELETE,
        };
        static lv_point_t line_points[];
        static lv_point_t line_points1[];
        static lv_point_t line_pointsPW1[];
        static lv_point_t line_pointsPW2[];
        static bool checkCreate;

        static void onCreate()
        {
            if(checkCreate)
            {
                // line split
                on_create_lv_line(&g_obj[0], &g_style[0], line_points, LV_ALIGN_BOTTOM_RIGHT, -ESP_PANEL_LCD_H_RES*0.5, -55);
                on_create_lv_line(&g_obj[1], &g_style[1], line_points1, LV_ALIGN_BOTTOM_LEFT, 20, -50);
                // RIGHT
                // name: g_value[0]
                on_create_lv_label(&g_value[0], &g_font[0], &vni_gotham_bold, LV_ALIGN_TOP_RIGHT, -160, 70);
                lv_label_set_text(g_value[0], "N/A");
                
                // percent - bar: g_obj[2] val: g_value[1]
                lv_style_init(&g_style[2]);
                lv_style_set_border_color(&g_style[2], lv_color_make(0xC9, 0xC9, 0xC9));
                lv_style_set_border_width(&g_style[2], 2);
                lv_style_set_pad_all(&g_style[2], 1); /*To make the indicator smaller*/
                lv_style_set_radius(&g_style[2], 10);
                lv_style_init(&g_style[3]);
                lv_style_set_bg_opa(&g_style[3], LV_OPA_COVER);
                lv_style_set_bg_color(&g_style[3], lv_color_make(0x0A, 0xA6, 0x53));
                lv_style_set_radius(&g_style[3], 10);
                g_obj[2] = lv_bar_create(lv_scr_act());
                lv_obj_remove_style_all(g_obj[2]);  /*To have a clean start*/
                lv_obj_add_style(g_obj[2], &g_style[2], 0);
                lv_obj_add_style(g_obj[2], &g_style[3], LV_PART_INDICATOR);
                lv_obj_set_size(g_obj[2], 85, 180);
                lv_obj_align(g_obj[2], LV_ALIGN_TOP_RIGHT, -270, 140);
                lv_bar_set_value(g_obj[2], 0, LV_ANIM_ON);
                lv_obj_clear_flag(g_obj[2],LV_OBJ_FLAG_SCROLLABLE);

                g_obj[3] = lv_line_create(lv_scr_act());
                lv_line_set_points(g_obj[3], line_pointsPW1, 2);
                lv_obj_add_style(g_obj[3], &g_style[4], 0);
                lv_obj_align(g_obj[3], LV_ALIGN_TOP_RIGHT, -292, 135);
                lv_obj_clear_flag(g_obj[3],LV_OBJ_FLAG_SCROLLABLE);

                lv_style_init(&g_font[1]);
                lv_style_set_text_font(&g_font[1], &vni_number); 
                g_value[1] = lv_label_create(g_obj[2]);
                lv_obj_add_style(g_value[1], &g_font[1], 0);  // <--- obj is the label
                lv_obj_center(g_value[1]);
                lv_obj_set_style_text_color(g_value[1], lv_color_make(0x33, 0x33, 0x33), 0);
                lv_label_set_text(g_value[1], "00 %");
                lv_obj_clear_flag(g_value[1],LV_OBJ_FLAG_SCROLLABLE);

                // voltage: g_value[2]
                on_create_lv_label(&g_title[0], &g_font[2], &vni_font, LV_ALIGN_TOP_RIGHT, -30, 130);
                lv_label_set_text(g_title[0], "Điện áp       (V)");

                on_create_lv_label(&g_value[2], &g_font[3], &vni_gotham_bold, LV_ALIGN_TOP_RIGHT, -65, 160);
                lv_obj_set_style_text_color(g_value[2], lv_color_make(0x51, 0x45, 0xAF), 0);
                lv_label_set_text(g_value[2], "00.0");

                // current: g_value[3]
                on_create_lv_label(&g_title[1], &g_font[4], &vni_font, LV_ALIGN_TOP_RIGHT, -30, 220);
                lv_label_set_text(g_title[1], "Dòng điện  (A)");

                on_create_lv_label(&g_value[3], &g_font[5], &vni_gotham_bold, LV_ALIGN_TOP_RIGHT, -65, 250);
                lv_obj_set_style_text_color(g_value[3], lv_color_make(0x51, 0x45, 0xAF), 0);
                lv_label_set_text(g_value[3], "00.0");

                // temperature: g_value[4]
                on_create_lv_label(&g_title[2], &g_font[6], &vni_font, LV_ALIGN_TOP_RIGHT, -30, 310);
                lv_label_set_text(g_title[2], "Nhiệt độ    (°C)");

                on_create_lv_label(&g_value[4], &g_font[7], &vni_gotham_bold, LV_ALIGN_TOP_RIGHT, -65, 340);
                lv_obj_set_style_text_color(g_value[4], lv_color_make(0x51, 0x45, 0xAF), 0);
                lv_label_set_text(g_value[4], "00.0");
                
                // Err: g_value[5]
                on_create_lv_label(&g_title[3], &g_font[8], &vni_font, LV_ALIGN_BOTTOM_RIGHT, -350, -120);
                lv_label_set_text(g_title[3], "E:");
                
                on_create_lv_label(&g_value[5], &g_font[9], &vni_font, LV_ALIGN_BOTTOM_LEFT, 470, -120);
                lv_label_set_text(g_value[5], "N/A");
                
                // Time: g_value[6]
                on_create_lv_label(&g_title[4], &g_font[10], &vni_font, LV_ALIGN_BOTTOM_RIGHT, -350, -70);
                lv_label_set_text(g_title[4], "T:");

                on_create_lv_label(&g_value[6], &g_font[11], &vni_font, LV_ALIGN_BOTTOM_LEFT, 470, -70);
                lv_label_set_text(g_value[6], "0:00");

                // LEFT
                // name: g_value[7]
                on_create_lv_label(&g_value[7], &g_font[12], &vni_gotham_bold, LV_ALIGN_TOP_LEFT, 160, 70);
                lv_label_set_text(g_value[7], "N/A");
                
                // percent - bar: g_obj[4] val: g_value[8]
                lv_style_init(&g_style[5]);
                lv_style_set_border_color(&g_style[5], lv_color_make(0xC9, 0xC9, 0xC9));
                lv_style_set_border_width(&g_style[5], 2);
                lv_style_set_pad_all(&g_style[5], 1); /*To make the indicator smaller*/
                lv_style_set_radius(&g_style[5], 10);
                lv_style_init(&g_style[6]);
                lv_style_set_bg_opa(&g_style[6], LV_OPA_COVER);
                lv_style_set_bg_color(&g_style[6], lv_color_make(0x0A, 0xA6, 0x53));
                lv_style_set_radius(&g_style[6], 10);
                g_obj[4] = lv_bar_create(lv_scr_act());
                lv_obj_remove_style_all(g_obj[4]);  /*To have a clean start*/
                lv_obj_add_style(g_obj[4], &g_style[5], 0);
                lv_obj_add_style(g_obj[4], &g_style[6], LV_PART_INDICATOR);
                lv_obj_set_size(g_obj[4], 85, 180);
                lv_obj_align(g_obj[4], LV_ALIGN_TOP_LEFT, 50, 140);
                lv_bar_set_value(g_obj[4], 0, LV_ANIM_ON);
                lv_obj_clear_flag(g_obj[4],LV_OBJ_FLAG_SCROLLABLE);

                g_obj[5] = lv_line_create(lv_scr_act());
                lv_line_set_points(g_obj[5], line_pointsPW2, 2);
                lv_obj_add_style(g_obj[5], &g_style[7], 0);
                lv_obj_align(g_obj[5], LV_ALIGN_TOP_LEFT, 77, 135);
                lv_obj_clear_flag(g_obj[5],LV_OBJ_FLAG_SCROLLABLE);

                lv_style_init(&g_font[13]);
                lv_style_set_text_font(&g_font[13], &vni_number); 
                g_value[8] = lv_label_create(g_obj[4]);
                lv_obj_add_style(g_value[8], &g_font[13], 0);  // <--- obj is the label
                lv_obj_center(g_value[8]);
                lv_obj_set_style_text_color(g_value[8], lv_color_make(0x33, 0x33, 0x33), 0);
                lv_label_set_text(g_value[8], "00 %");
                lv_obj_clear_flag(g_value[8],LV_OBJ_FLAG_SCROLLABLE);

                // voltage: g_value[9]
                on_create_lv_label(&g_title[5], &g_font[14], &vni_font, LV_ALIGN_TOP_LEFT, 180, 130);
                lv_label_set_text(g_title[5], "Điện áp       (V)");

                on_create_lv_label(&g_value[9], &g_font[15], &vni_gotham_bold, LV_ALIGN_TOP_LEFT, 215, 160);
                lv_obj_set_style_text_color(g_value[9], lv_color_make(0x51, 0x45, 0xAF), 0);
                lv_label_set_text(g_value[9], "00.0");

                // current: g_value[10]
                on_create_lv_label(&g_title[6], &g_font[16], &vni_font, LV_ALIGN_TOP_LEFT, 180, 220);
                lv_label_set_text(g_title[6], "Dòng điện  (A)");

                on_create_lv_label(&g_value[10], &g_font[17], &vni_gotham_bold, LV_ALIGN_TOP_LEFT, 215, 250);
                lv_obj_set_style_text_color(g_value[10], lv_color_make(0x51, 0x45, 0xAF), 0);
                lv_label_set_text(g_value[10], "00.0");

                // temperature: g_value[11]
                on_create_lv_label(&g_title[7], &g_font[18], &vni_font, LV_ALIGN_TOP_LEFT, 180, 310);
                lv_label_set_text(g_title[7], "Nhiệt độ    (°C)");

                on_create_lv_label(&g_value[11], &g_font[19], &vni_gotham_bold, LV_ALIGN_TOP_LEFT, 215, 340);
                lv_obj_set_style_text_color(g_value[11], lv_color_make(0x51, 0x45, 0xAF), 0);
                lv_label_set_text(g_value[11], "00.0");

                // Err: g_value[12]
                on_create_lv_label(&g_title[8], &g_font[20], &vni_font, LV_ALIGN_BOTTOM_LEFT, 30, -120);
                lv_label_set_text(g_title[8], "E:");
                
                on_create_lv_label(&g_value[12], &g_font[21], &vni_font, LV_ALIGN_BOTTOM_LEFT, 70, -120);
                lv_label_set_text(g_value[12], "N/A");
                
                // Time: g_value[13]
                on_create_lv_label(&g_title[9], &g_font[22], &vni_font, LV_ALIGN_BOTTOM_LEFT, 30, -70);
                lv_label_set_text(g_title[9], "T:");

                on_create_lv_label(&g_value[13], &g_font[23], &vni_font, LV_ALIGN_BOTTOM_LEFT, 70, -70);
                lv_label_set_text(g_value[13], "0:00");

                // BOTTOM
                // Num Ins: g_value[14]
                on_create_lv_label(&g_title[10], &g_font[24], &vni_font, LV_ALIGN_BOTTOM_LEFT, 30, -10);
                lv_label_set_text(g_title[10], "N:");

                on_create_lv_label(&g_value[14], &g_font[25], &vni_font, LV_ALIGN_BOTTOM_LEFT, 70, -10);
                lv_label_set_text(g_value[14], "N/A");

                // Vi: g_value[15]
                on_create_lv_label(&g_title[11], &g_font[26], &vni_font, LV_ALIGN_BOTTOM_LEFT, 170, -10);
                lv_label_set_text(g_title[11], "Vi:");

                on_create_lv_label(&g_value[15], &g_font[27], &vni_font, LV_ALIGN_BOTTOM_LEFT, 210, -10);
                lv_label_set_text(g_value[15], "N/A");

                // Ii: g_value[16]
                on_create_lv_label(&g_title[12], &g_font[28], &vni_font, LV_ALIGN_BOTTOM_LEFT, 330, -10);
                lv_label_set_text(g_title[12], "Ii:");

                on_create_lv_label(&g_value[16], &g_font[29], &vni_font, LV_ALIGN_BOTTOM_LEFT, 370, -10);
                lv_label_set_text(g_value[16], "N/A");

                // Vo: g_value[17]
                on_create_lv_label(&g_title[13], &g_font[30], &vni_font, LV_ALIGN_BOTTOM_LEFT, 490, -10);
                lv_label_set_text(g_title[13], "Vo:");

                on_create_lv_label(&g_value[17], &g_font[31], &vni_font, LV_ALIGN_BOTTOM_LEFT, 540, -10);
                lv_label_set_text(g_value[17], "N/A");

                // Io: g_value[18]
                on_create_lv_label(&g_title[14], &g_font[32], &vni_font, LV_ALIGN_BOTTOM_LEFT, 650, -10);
                lv_label_set_text(g_title[14], "Io:");

                on_create_lv_label(&g_value[18], &g_font[33], &vni_font, LV_ALIGN_BOTTOM_LEFT, 690, -10);
                lv_label_set_text(g_value[18], "N/A");

                // charging: g_obj[6] g_obj[7]
                on_create_lv_img(&g_obj[6], &charging, 600, LV_ALIGN_TOP_LEFT, 50, 90);
                on_create_lv_img(&g_obj[7], &charging, 600, LV_ALIGN_TOP_LEFT, 440, 90);
                lv_obj_add_flag(g_obj[6], LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(g_obj[7], LV_OBJ_FLAG_HIDDEN);
                // clear flag
                lv_obj_clear_flag(lv_scr_act(),LV_OBJ_FLAG_SCROLLABLE);
                checkCreate = false;
            }
        }

        static void onUpdate()
        {
            for (uint8_t i = 0; i < 2; i++)
            {
                if (batfr::charging[i]) {
                    if(i == 0) lv_obj_clear_flag(g_obj[7], LV_OBJ_FLAG_HIDDEN);
                    else lv_obj_clear_flag(g_obj[6], LV_OBJ_FLAG_HIDDEN);
                } else {
                    if(i == 0) lv_obj_add_flag(g_obj[7], LV_OBJ_FLAG_HIDDEN);
                    else lv_obj_add_flag(g_obj[6], LV_OBJ_FLAG_HIDDEN);
                }

                // battery Name
                switch (batfr::capacity[i])
                {
                case BATTERY_TYPE::BATTERY_UNKNOWN:
                    if(i == 0) lv_label_set_text(g_value[0], "N/A");
                    else lv_label_set_text(g_value[7], "N/A");
                    break;
                case BATTERY_TYPE::BATTERY_T20P:
                    if(i == 0) lv_label_set_text(g_value[0], "T20P");
                    else lv_label_set_text(g_value[7], "T20P");
                    break;
                case BATTERY_TYPE::BATTERY_T25:
                    if(i == 0) lv_label_set_text(g_value[0], "T25");
                    else lv_label_set_text(g_value[7], "T25");
                    break;
                case BATTERY_TYPE::BATTERY_T30:
                    if(i == 0) lv_label_set_text(g_value[0], "T30");
                    else lv_label_set_text(g_value[7], "T30");
                    break;
                case BATTERY_TYPE::BATTERY_T40:
                    if(i == 0) lv_label_set_text(g_value[0], "T40");
                    else lv_label_set_text(g_value[7], "T40");
                    break;
                case BATTERY_TYPE::BATTERY_T50:
                    if(i == 0) lv_label_set_text(g_value[0], "T50");
                    else lv_label_set_text(g_value[7], "T50");
                    break;
                }
                // voltage
                if(i == 0) on_change_lv_label_text(g_value[2], (batfr::capacity[i] > 0) ? String((float)(batfr::voltage[i])/1000.0, 1): "00.0");
                else on_change_lv_label_text(g_value[9], (batfr::capacity[i] > 0) ? String((float)(batfr::voltage[i])/1000.0, 1): "00.0");
                // current
                if(i == 0) on_change_lv_label_text(g_value[3], (batfr::capacity[i] > 0) ? String((float)(batfr::current[i])/1000.0, 1) : "00.0");
                else on_change_lv_label_text(g_value[10], (batfr::capacity[i] > 0) ? String((float)(batfr::current[i])/1000.0, 1) : "00.0");
                // percent
                if(i == 0) on_change_lv_label_text(g_value[1], (batfr::capacity[i] > 0) ? ((batfr::percent[i] < 10) ? "0" : "" ) + (String)batfr::percent[i]: "00");
                else on_change_lv_label_text(g_value[8], (batfr::capacity[i] > 0) ? ((batfr::percent[i] < 10) ? "0" : "" ) + (String)batfr::percent[i]: "00");
                if(i == 0) lv_bar_set_value(g_obj[2], (batfr::capacity[i] > 0) ? batfr::percent[i] : 0, LV_ANIM_ON);
                else lv_bar_set_value(g_obj[4], (batfr::capacity[i] > 0) ? batfr::percent[i] : 0, LV_ANIM_ON);
                // temperature
                if(i == 0) on_change_lv_label_text(g_value[4], (batfr::capacity[i] > 0) ? String((float)(batfr::temperature[i])/10.0, 1): "00.0");
                else on_change_lv_label_text(g_value[11], (batfr::capacity[i] > 0) ? String((float)(batfr::temperature[i])/10.0, 1): "00.0");
                // timeCharge
                if(i == 0) on_change_lv_label_text(g_value[6], (batfr::capacity[i] > 0) ? (String)batfr::minutes[i] + ":" + ((batfr::seconds[i] < 10) ? "0" : "") +(String)batfr::seconds[i] : "N/A"); 
                else on_change_lv_label_text(g_value[13], (batfr::capacity[i] > 0) ? (String)batfr::minutes[i] + ":" + ((batfr::seconds[i] < 10) ? "0" : "") +(String)batfr::seconds[i] : "N/A"); 
                // error
                if(i == 0) on_change_lv_label_text(g_value[5], (batfr::capacity[i] > 0) ? (String)batfr::countError[i] : "N/A");
                else on_change_lv_label_text(g_value[12], (batfr::capacity[i] > 0) ? (String)batfr::countError[i] : "N/A");
            }
            
            // bottom
            // numins
            on_change_lv_label_text(g_value[14], (String)sysfr::num_ins);
            // vi
            on_change_lv_label_text(g_value[15], String((sysfr::input_voltage[0] + sysfr::input_voltage[1] + sysfr::input_voltage[2])/(float)sysfr::num_ins, 1));
            // ii
            on_change_lv_label_text(g_value[16], String((sysfr::input_current[0] + sysfr::input_current[1] + sysfr::input_current[2]), 1));
            // vo
            on_change_lv_label_text(g_value[17], String((sysfr::output_voltage[0] + sysfr::output_voltage[1] + sysfr::output_voltage[2])/(float)sysfr::num_ins, 1));
            // io
            on_change_lv_label_text(g_value[18], String((sysfr::output_current[0] + sysfr::output_current[1] + sysfr::output_current[2]), 1));
        }

        static void onDelete()
        {
            for(uint8_t i = 0; i < 8; i++) lv_style_reset(&g_style[i]);
            for(uint8_t i = 0; i < 34; i++) lv_style_reset(&g_font[i]);
            for(uint8_t i = 0; i < 15; i++) { lv_obj_remove_style_all(g_title[i]); lv_obj_del(g_title[i]);}
            for(uint8_t i = 0; i < 19; i++) { lv_obj_remove_style_all(g_value[i]); lv_obj_del(g_value[i]);}
            for(uint8_t i = 0; i < 8; i++) { lv_obj_remove_style_all(g_obj[i]); lv_obj_del(g_obj[i]);}
            checkCreate = true;
        }
    };

    struct BATTERY_DJI_PAGE
    {
        static uint8_t step;
        enum STEP
        {
            CREATE,
            UPDATE,
            DELETE,
        };
        static lv_color_t battery_color_percent;
        static lv_style_t cell_style_border[BMS_CELL_NUM];
        static lv_style_t cell_style_bg[BMS_CELL_NUM];
        static lv_obj_t  *cell_bar[BMS_CELL_NUM];
        static lv_obj_t  *cell_lb[BMS_CELL_NUM];

        static void onCreate()
        {
            // battery name
            on_create_lv_label(&g_value[0], &g_font[0], &vni_gotham_bold, LV_ALIGN_TOP_LEFT, 30, 70);
            lv_label_set_text(g_value[0], "N/A");
            // arc percent
            g_obj[0] = lv_arc_create(lv_scr_act());
            lv_obj_set_style_arc_color(g_obj[0], lv_color_make(0xA7, 0xA7, 0xA7), 0); // Arc color
            lv_obj_set_style_arc_color(g_obj[0], lv_color_make(0x16, 0x91, 0x50), LV_PART_INDICATOR);
            lv_obj_set_style_arc_width(g_obj[0], 30, 0); // Arc width
            lv_obj_set_style_arc_width(g_obj[0], 30, LV_PART_INDICATOR); // Arc width'
            lv_obj_set_size(g_obj[0], 220, 220);
            lv_arc_set_rotation(g_obj[0], 270);
            lv_arc_set_bg_angles(g_obj[0], 0, 360);
            lv_obj_remove_style(g_obj[0], NULL, LV_PART_KNOB);   /*Be sure the knob is not displayed*/
            lv_obj_clear_flag(g_obj[0], LV_OBJ_FLAG_CLICKABLE);  /*To not allow adjusting by click*/
            lv_obj_align(g_obj[0], LV_ALIGN_LEFT_MID, 60, -5);
            lv_arc_set_value(g_obj[0], 00);
            lv_obj_clear_flag(g_obj[0],LV_OBJ_FLAG_SCROLLABLE);
            // label percent
            lv_style_init(&g_font[1]);
            lv_style_set_text_font(&g_font[1], &dji_gotham); 
            g_value[1] = lv_label_create(g_obj[0]);
            lv_obj_add_style(g_value[1], &g_font[1], 0);  // <--- obj is the label
            lv_obj_center(g_value[1]);
            lv_obj_set_style_text_color(g_value[1], lv_color_make(0x16, 0x91, 0x50), 0);
            lv_label_set_text(g_value[1], "00");
            lv_obj_clear_flag(g_value[1],LV_OBJ_FLAG_SCROLLABLE);

            lv_style_init(&g_font[2]);
            lv_style_set_text_font(&g_font[2], &vni_number); 
            g_value[2] = lv_label_create(g_obj[0]);
            lv_obj_add_style(g_value[2], &g_font[2], 0);  // <--- obj is the label
            lv_obj_set_style_text_color(g_value[2], lv_color_make(0x16, 0x91, 0x50), 0);
            lv_obj_align(g_value[2], LV_ALIGN_BOTTOM_MID, 0, -35);
            lv_label_set_text(g_value[2], "%");
            lv_obj_clear_flag(g_value[2],LV_OBJ_FLAG_SCROLLABLE);

            // voltage
            on_create_lv_label(&g_title[0], &g_font[3], &vni_font, LV_ALIGN_TOP_RIGHT, -ESP_PANEL_LCD_H_RES*0.345, 70);
            lv_label_set_text(g_title[0], "Điện áp        (V)");

            on_create_lv_label(&g_value[3], &g_font[4], &vni_gotham_bold_70, LV_ALIGN_TOP_RIGHT, -ESP_PANEL_LCD_H_RES*0.36, 110);
            lv_obj_set_style_text_color(g_value[3], lv_color_make(0x51, 0x45, 0xAF), 0);
            lv_label_set_text(g_value[3], "00.0");
            // current
            on_create_lv_label(&g_title[1], &g_font[5], &vni_font, LV_ALIGN_TOP_RIGHT, -30, 70);
            lv_label_set_text(g_title[1], "Dòng điện  (A)");

            on_create_lv_label(&g_value[4], &g_font[6], &vni_gotham_bold_70, LV_ALIGN_TOP_RIGHT, -30, 110);
            lv_obj_set_style_text_color(g_value[4], lv_color_make(0x51, 0x45, 0xAF), 0);
            lv_label_set_text(g_value[4], "00.0");
            // temperature
            on_create_lv_label(&g_title[2], &g_font[7], &vni_font, LV_ALIGN_TOP_RIGHT, -ESP_PANEL_LCD_H_RES*0.345, 180);
            lv_label_set_text(g_title[2], "Nhiệt độ    (°C)");

            on_create_lv_label(&g_value[5], &g_font[8], &vni_gotham_bold_70, LV_ALIGN_TOP_RIGHT, -ESP_PANEL_LCD_H_RES*0.36, 230);
            lv_obj_set_style_text_color(g_value[5], lv_color_make(0x51, 0x45, 0xAF), 0);
            lv_label_set_text(g_value[5], "00.0");
            // numberCharge
            on_create_lv_label(&g_title[3], &g_font[9], &vni_font, LV_ALIGN_TOP_RIGHT, -30, 180);
            lv_label_set_text(g_title[3], "Số lần sạc   ");

            on_create_lv_label(&g_value[6], &g_font[10], &vni_gotham_bold_70, LV_ALIGN_TOP_RIGHT, -30, 230);
            lv_obj_set_style_text_color(g_value[6], lv_color_make(0x51, 0x45, 0xAF), 0);
            lv_label_set_text(g_value[6], "0000");
            // version
            on_create_lv_label(&g_title[4], &g_font[11], &vni_font, LV_ALIGN_BOTTOM_LEFT, 330, -150);
            lv_label_set_text(g_title[4], "Ver:");

            on_create_lv_label(&g_value[7], &g_font[12], &vni_font, LV_ALIGN_BOTTOM_LEFT, 400, -150);
            lv_label_set_text(g_value[7], "N/A");   
            // seri number
            on_create_lv_label(&g_title[5], &g_font[13], &vni_font, LV_ALIGN_BOTTOM_LEFT, 330, -110);
            lv_label_set_text(g_title[5], "S/N:");
            
            on_create_lv_label(&g_value[8], &g_font[14], &vni_font, LV_ALIGN_BOTTOM_LEFT, 400, -110);
            lv_label_set_text(g_value[8], "N/A");
            // Err
            on_create_lv_label(&g_title[6], &g_font[15], &vni_font, LV_ALIGN_TOP_RIGHT, -120, 290);
            lv_label_set_text(g_title[6], "Er:");
            
            on_create_lv_label(&g_value[9], &g_font[16], &vni_font, LV_ALIGN_TOP_RIGHT, -30, 290);
            lv_label_set_text(g_value[9], "N/A");

            // cell
            for (uint8_t i = 0; i < BATTERY_DJI_CELL_NUM; i++)
            {
                lv_style_init(&cell_style_bg[i]);
                lv_style_set_border_color(&cell_style_bg[i], lv_color_make(0xC9, 0xC9, 0xC9));
                lv_style_set_border_width(&cell_style_bg[i], 2);
                lv_style_set_pad_all(&cell_style_bg[i], 1); /*To make the indicator smaller*/
                lv_style_set_radius(&cell_style_bg[i], 3);
                lv_style_set_anim_time(&cell_style_bg[i], 1000);

                lv_style_init(&cell_style_border[i]);
                lv_style_set_bg_opa(&cell_style_border[i], LV_OPA_COVER);
                lv_style_set_bg_color(&cell_style_border[i], lv_color_make(0x0A, 0xA6, 0x53));
                lv_style_set_radius(&cell_style_border[i], 3);

                cell_bar[i] = lv_bar_create(lv_scr_act());
                lv_obj_remove_style_all(cell_bar[i]);  /*To have a clean start*/
                lv_obj_add_style(cell_bar[i], &cell_style_bg[i], 0);
                lv_obj_add_style(cell_bar[i], &cell_style_border[i], LV_PART_INDICATOR);

                lv_obj_set_size(cell_bar[i], 30, 70);
                lv_obj_align(cell_bar[i], LV_ALIGN_BOTTOM_LEFT, 30 + i*54.5, -30);
                lv_bar_set_value(cell_bar[i], 0, LV_ANIM_ON);

                cell_lb[i] = lv_label_create(lv_scr_act());
                lv_label_set_text(cell_lb[i] , "0.00"); // Đặt số ban đầu
                lv_obj_align_to(cell_lb[i] , cell_bar[i], LV_ALIGN_OUT_BOTTOM_MID, 12, 0); // Căn chỉnh nhãn phía dưới thanh bar
                lv_obj_set_style_text_color(cell_lb[i] , lv_color_hex(0x000000), LV_PART_MAIN); // Màu đen
                lv_obj_set_style_text_font(cell_lb[i] , &lv_font_montserrat_18, LV_PART_MAIN);
            
                lv_obj_clear_flag(cell_bar[i],LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_clear_flag(cell_lb[i],LV_OBJ_FLAG_SCROLLABLE);
            }

            // clear flag
            lv_obj_clear_flag(lv_scr_act(),LV_OBJ_FLAG_SCROLLABLE);
        }

        static void onUpdate(uint8_t ins)
        {
            debugPort->printf("ins: %d \n", ins);
            // battery Name
            switch (batfr::capacity[ins])
            {
            case BATTERY_TYPE::BATTERY_UNKNOWN:
                lv_label_set_text(g_value[0], (ins == 0) ? "> N/A" : "< N/A");
                break;
            case BATTERY_TYPE::BATTERY_T20P:
                lv_label_set_text(g_value[0], (ins == 0) ? "> T20P" : "< T20P");
                break;
            case BATTERY_TYPE::BATTERY_T25:
                lv_label_set_text(g_value[0], (ins == 0) ? "> T25" : "< T25");
                break;
            case BATTERY_TYPE::BATTERY_T30:
                lv_label_set_text(g_value[0], (ins == 0) ? "> T30" : "< T30");
                break;
            case BATTERY_TYPE::BATTERY_T40:
                lv_label_set_text(g_value[0], (ins == 0) ? "> T40" : "< T40");
                break;
            case BATTERY_TYPE::BATTERY_T50:
                lv_label_set_text(g_value[0], (ins == 0) ? "> T50" : "< T50");
                break;
            }
            // voltage
            on_change_lv_label_text(g_value[3], (batfr::capacity[ins] > 0) ? String((float)(batfr::voltage[ins])/1000.0, 1): "00.0");
            // current
            on_change_lv_label_text(g_value[4], (batfr::capacity[ins] > 0) ? String((float)(batfr::current[ins])/1000.0, 1) : "00.0");
            // percent
            on_change_lv_label_text(g_value[1], (batfr::capacity[ins] > 0) ? ((batfr::percent[ins] < 10) ? "0" : "" ) + (String)batfr::percent[ins]: "00");
            lv_arc_set_value(g_obj[0], (batfr::capacity[ins] > 0) ? batfr::percent[ins] : 0);
            // temperature
            on_change_lv_label_text(g_value[5], (batfr::capacity[ins] > 0) ? String((float)(batfr::temperature[ins])/10.0, 1): "00.0");
            // numbercharge
            on_change_lv_label_text(g_value[6], (batfr::capacity[ins] > 0) ? String(batfr::numberCharge[ins]): "0000");
            // version
            on_change_lv_label_text(g_value[7], (batfr::capacity[ins] > 0) ? (String)batfr::version[ins][0] + "." + (String)batfr::version[ins][1] + "." + (String)batfr::version[ins][2] + "." + (String)batfr::version[ins][3]: "N/A"); // update sau
            // seri number
            char _tempSN[14]= {};
            for (uint8_t i = 0; i < 14; i++) _tempSN[i] = (char)batfr::seriNumber[ins][i];
            lv_label_set_text(g_value[8], (batfr::capacity[ins] > 0) ? _tempSN : "N/A");
            // error
            on_change_lv_label_text(g_value[9], (batfr::capacity[ins] > 0) ? (String)batfr::countError[ins] : "N/A");
            // cell
            uint16_t _tempCell = 0;
            for (uint8_t i = 0; i < BATTERY_DJI_CELL_NUM; i++)
            { 
                if(batfr::capacity[ins] > 0) _tempCell = map(batfr::cell[ins][i], 3000, 4250, 0, 100);
                else _tempCell= 0;
                // 0 - 10% do
                if(_tempCell <= 10)  lv_style_set_bg_color(&cell_style_border[i], lv_color_hex(0xDF0000));
                // 10-20% vang
                if(_tempCell > 10 && _tempCell <= 20)  lv_style_set_bg_color(&cell_style_border[i], lv_color_hex(0xE2E755));
                // > 20 xanh
                if(_tempCell > 20 && _tempCell <= 100) lv_style_set_bg_color(&cell_style_border[i], lv_color_make(0x0A, 0xA6, 0x53));

                lv_bar_set_value(cell_bar[i], _tempCell, LV_ANIM_ON);
                on_change_lv_label_text(cell_lb[i] , (batfr::capacity[ins] > 0) ? String((float)(batfr::cell[ins][i])/1000.0, 2): "00.0"); // Đặt số ban đầu
            }

            // color
            if(batfr::percent[ins] <= 10)
            {
                battery_color_percent = lv_color_hex(0xDF0000);
            }
            if (batfr::percent[ins] <= 20 && batfr::percent[ins] > 10)
            {
                battery_color_percent = lv_color_hex(0xE2E755);
            }
            if(batfr::percent[ins] > 20)
            {
                battery_color_percent = lv_color_make(0x0A, 0xA6, 0x53);
            }
            lv_obj_set_style_arc_color(g_obj[0], battery_color_percent, LV_PART_INDICATOR);
            lv_obj_set_style_text_color(g_value[1], battery_color_percent, 0);
            lv_obj_set_style_text_color(g_value[2], battery_color_percent, 0);
        }

        static void onDelete()
        {
            for(uint8_t i = 0; i < 17; i++) lv_style_reset(&g_font[i]);
            for(uint8_t i = 0; i < 7; i++) { lv_obj_remove_style_all(g_title[i]); lv_obj_del(g_title[i]);}
            for(uint8_t i = 0; i < 10; i++) { lv_obj_remove_style_all(g_value[i]); lv_obj_del(g_value[i]);}
            for(uint8_t i = 0; i < 1; i++) { lv_obj_remove_style_all(g_obj[i]); lv_obj_del(g_obj[i]);}
            //cell
            for (uint8_t i = 0; i < BATTERY_DJI_CELL_NUM; i++)
            {
                lv_style_reset(&cell_style_bg[i]);
                lv_style_reset(&cell_style_border[i]);
                lv_obj_remove_style_all(cell_bar[i]); lv_obj_del(cell_bar[i]);
                lv_obj_remove_style_all(cell_lb[i]); lv_obj_del(cell_lb[i]);
            }
        }
    };

    struct SYSTEM_PAGE
    {
        static uint8_t step;
        static String s_temp;
        enum STEP
        {
            CREATE,
            UPDATE,
            DELETE,
        };
        static void changeNumins(String &t, uint8_t num, lv_obj_t *_obj)
        {
            t = "Num: " + (String)num;
            lv_label_set_text(_obj, t.c_str());
            t = "";
        }

        static void changeACmodule( String &t,
                                    uint8_t ins, 
                                    float vi, 
                                    float ii, 
                                    float f, 
                                    float ti,
                                    float vo,
                                    float io,
                                    float im,
                                    float to,
                                    float e,
                                    lv_obj_t *_obj)
        {
            t = (String) ins +  ": Vi: " + String(vi, 1) + 
                                "  Ii: " + String(ii, 1) + 
                                "  F: " + String(f, 1) + 
                                "  Ti: " + String(ti, 1) + 
                                " \n    Vo: " + String(vo, 1) + 
                                "  Io: " + String(io, 1) + 
                                "  Im: " + String(im, 1) + 
                                "  To: " + String(to, 1) + 
                                "  E: " + String(e, 1) ;
            lv_label_set_text(_obj, t.c_str());
            t = "";
        }

        static void changeParam(String &t, 
                                String _m1, 
                                String _m2, 
                                String _p1, 
                                String _p2,
                                lv_obj_t *_obj)
        {
            t = _m1 + ": " + _p1 + "           " + _m2 + ": " + _p2;
            lv_label_set_text(_obj, t.c_str());
            t = "";
        }

        static void onCreate()
        {
            // Num ins
            on_create_lv_label(&g_value[0], &g_font[0], &vni_font, LV_ALIGN_TOP_LEFT, 80, 75);
            lv_label_set_text(g_value[0], "Num: N/A");
            // 1
            on_create_lv_label(&g_value[1], &g_font[1], &vni_font, LV_ALIGN_TOP_LEFT, 80, 120);
            lv_label_set_text(g_value[1], "1: Vi: 220.1  Ii: 50.2  F: 49.9  Ti: 26.1 \n    Vo: 50.1  Io: 50.1  Im: 70.8  To: 27.2  E: 97.8");
            // 2
            on_create_lv_label(&g_value[2], &g_font[2], &vni_font, LV_ALIGN_TOP_LEFT, 80, 210);
            lv_label_set_text(g_value[2], "2: Vi: 220.1  Ii: 50.2  F: 49.9  Ti: 26.1 \n    Vo: 50.1  Io: 50.1  Im: 70.8  To: 27.2  E: 97.8");
            // 3
            on_create_lv_label(&g_value[3], &g_font[3], &vni_font, LV_ALIGN_TOP_LEFT, 80, 300);
            lv_label_set_text(g_value[3], "3: Vi: 220.1  Ii: 50.2  F: 49.9  Ti: 26.1 \n    Vo: 50.1  Io: 50.1  Im: 70.8  To: 27.2  E: 97.8");
            // EP
            on_create_lv_label(&g_value[4], &g_font[4], &vni_font, LV_ALIGN_TOP_LEFT, 80, 390);
            lv_label_set_text(g_value[4], "E: 1.0.0           SN: TAC20J4EPT3UO5M");
            // Data
            on_create_lv_label(&g_value[5], &g_font[5], &vni_font, LV_ALIGN_TOP_LEFT, 80, 435);
            lv_label_set_text(g_value[5], "D: 1.0.0           Pa: 1,57,58,43,48");
        }

        static void onUpdate()
        {
            changeNumins(s_temp, sysfr::num_ins, g_value[0]);
            for (uint8_t i = 0; i < 3; i++)
            {
            changeACmodule(     s_temp,
                                1 + i,   
                                sysfr::input_voltage[i],
                                sysfr::input_current[i],
                                sysfr::input_freq[i],
                                sysfr::input_temp[i],
                                sysfr::output_voltage[i],
                                sysfr::output_current[i],
                                sysfr::output_current_max[i],
                                sysfr::output_temp[i],
                                sysfr::efficiency[i],
                                g_value[1 + i]);
            }
            changeParam(s_temp, 
                        "E", 
                        "SN", 
                        sysfr::e_version, 
                        sysfr::e_sn,
                        g_value[4]);
            changeParam(s_temp, 
                        "D", 
                        "Pa", 
                        sysfr::d_version, 
                        sysfr::d_param,
                        g_value[5]);
        }

        static void onDelete()
        {
            for(uint8_t i = 0; i < 6; i++) lv_style_reset(&g_font[i]);
            for(uint8_t i = 0; i < 6; i++) { lv_obj_remove_style_all(g_value[i]); lv_obj_del(g_value[i]);}
        }
    }; 

    static uint8_t choose_page;
    static uint8_t bms_instance;
    static char digit_buffer[10];
public:
    Display(HardwareSerial &debugPort);
    ~Display();

    static uint8_t page;
    static bool page_created;
    static uint8_t state_value;
    
    struct batfr
    {
        static bool active[2];
        static bool charging[2];
        static uint8_t seriNumber[2][14];
        static uint8_t version[2][4];
        static uint16_t voltage[2];
        static int32_t current[2];
        static uint8_t percent[2];
        static uint16_t cell[2][14];
        static int16_t numberCharge[2];
        static uint16_t temperature[2];
        static uint16_t capacity[2];
        static uint8_t countError[2];
        static uint8_t led[2];
        static int seconds[2]; 
        static int minutes[2]; 
        // Hàm để reset toàn bộ biến về 0
        static void reset() {
            for (uint8_t i = 0; i < 2; ++i) {
                active[i] = false;
                charging[i] = false;
                memset(seriNumber[i], 0, sizeof(seriNumber[i]));
                memset(version[i], 0, sizeof(version[i]));
                voltage[i] = 0;
                current[i] = 0;
                percent[i] = 0;
                memset(cell[i], 0, sizeof(cell[i]));
                numberCharge[i] = 0;
                temperature[i] = 0;
                capacity[i] = 0;
                countError[i] = 0;
                led[i] = 0;
                seconds[i] = 0;
                minutes[i] = 0;
            }
        }
    };

    struct sysfr
    {
        static float input_voltage[3]; // 2 
        static float input_freq[3]; // 1
        static float input_current[3]; // 2
        static float input_temp[3]; // 2
        static float efficiency[3]; // 1
        static float output_voltage[3]; // 2
        static float output_current[3]; // 2
        static float output_current_max[3]; // 2
        static float output_temp[3]; // 2
        static uint8_t num_ins;
        static String e_version;
        static String e_sn;
        static String d_version;
        static String d_param;
        static void reset() {
            num_ins = 0;
            memset(input_voltage, 0, sizeof(input_voltage));
            memset(input_freq, 0, sizeof(input_freq));
            memset(input_current, 0, sizeof(input_current));
            memset(input_temp, 0, sizeof(input_temp));
            memset(efficiency, 0, sizeof(efficiency));
            memset(output_voltage, 0, sizeof(output_voltage));
            memset(output_current, 0, sizeof(output_current));
            memset(output_current_max, 0, sizeof(output_current_max));
            memset(output_temp, 0, sizeof(output_temp));
        }
    };

    enum PAGE
    {
        BOOT,
        MAIN,
        BATTERY_DJI,
        BMS1,
        BMS2,
        SYSTEM,
    };

    enum BATTERY_TYPE
    {
        BATTERY_UNKNOWN = 0,
        BATTERY_T30 = 29000,
        BATTERY_T20P = 13000,
        BATTERY_T40 = 30000,
        BATTERY_T25 = 16818,
        BATTERY_T50 = 33022,
    };

    bool init();
    void handleUpdateFirmware(bool option);
};

#endif // 