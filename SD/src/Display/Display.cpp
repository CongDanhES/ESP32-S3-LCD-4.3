#include "Display.hpp"

// Khởi tạo biến tĩnh
HardwareSerial *Display::debugPort = nullptr;
// Định nghĩa các biến
ESP_Panel *Display::panel = nullptr; // Hoặc khởi tạo theo cách bạn cần
SemaphoreHandle_t Display::lvgl_mux = nullptr;
ESP_IOExpander *Display::expander = nullptr;

uint8_t Display::page = 0;
bool Display::page_created = false;
uint8_t Display::state_value = 2;
char Display::digit_buffer[10] = {0};
uint8_t Display::choose_page = 0;
uint8_t Display::bms_instance = 0;
bool Display::batfr::active[2] = {false, false};
bool Display::batfr::charging[2] = {false, false};
uint8_t Display::batfr::seriNumber[2][14] = {{0}, {0}};
uint8_t Display::batfr::version[2][4] = {{0}, {0}};
uint16_t Display::batfr::voltage[2] = {0, 0};
int32_t Display::batfr::current[2] = {0, 0};
uint8_t Display::batfr::percent[2] = {0, 0};
uint16_t Display::batfr::cell[2][14] = {{0}, {0}};
int16_t Display::batfr::numberCharge[2] = {0, 0};
uint16_t Display::batfr::temperature[2] = {0, 0};
uint16_t Display::batfr::capacity[2] = {0, 0};
uint8_t Display::batfr::countError[2] = {0, 0};
uint8_t Display::batfr::led[2] = {0, 0};
int Display::batfr::seconds[2] = {0, 0};
int Display::batfr::minutes[2] = {0, 0};

float Display::sysfr::input_voltage[3] = {0.0};
float Display::sysfr::input_freq[3] = {0.0};
float Display::sysfr::input_current[3] = {0.0};
float Display::sysfr::input_temp[3] = {0.0};
float Display::sysfr::efficiency[3] = {0.0};
float Display::sysfr::output_voltage[3] = {0.0};
float Display::sysfr::output_current[3] = {0.0};
float Display::sysfr::output_current_max[3] = {0.0};
float Display::sysfr::output_temp[3] = {0.0};
uint8_t Display::sysfr::num_ins = 0;
String Display::sysfr::e_version = "";
String Display::sysfr::e_sn = "";
String Display::sysfr::d_version = "";
String Display::sysfr::d_param = "";

// Định nghĩa các thành viên tĩnh
lv_obj_t* Display::g_value[NUM_OBJ_TITLE] = {nullptr};
lv_obj_t* Display::g_title[NUM_OBJ_VALUE] = {nullptr};
lv_obj_t* Display::g_obj[NUM_OBJ] = {nullptr};
lv_style_t Display::g_style[NUM_STYLE];
lv_style_t Display::g_font[NUM_FONT];

lv_obj_t* Display::BOOT_PAGE::logo = nullptr; // Hoặc giá trị khởi tạo khác nếu cần
lv_anim_t* Display::BOOT_PAGE::animation = nullptr; // Hoặc giá trị khởi tạo khác nếu cần

lv_style_t Display::FIX_FRAME::main_panel_style ;
lv_obj_t *Display::FIX_FRAME::main_panel = nullptr;
bool Display::FIX_FRAME::checkCreate = true;
lv_obj_t *Display::FIX_FRAME::lb_status_obj = nullptr;
String Display::FIX_FRAME::lb_status_str = "";
lv_obj_t *Display::FIX_FRAME::img_clock_obj = nullptr;
lv_obj_t *Display::FIX_FRAME::lb_clock_obj = nullptr;
lv_style_t Display::FIX_FRAME::font_vni_n;
lv_style_t Display::FIX_FRAME::font_vni_g_b;
lv_obj_t* Display::FIX_FRAME::logo_charge = nullptr;

uint8_t Display::MAIN_PAGE::step = 0;
lv_point_t Display::MAIN_PAGE::line_points[] = { {0, 0}, {0, (int32_t)(ESP_PANEL_LCD_V_RES*0.73)}};
lv_point_t Display::MAIN_PAGE::line_points1[] = { {0, 0}, {(int32_t)(ESP_PANEL_LCD_H_RES*0.94), 0}};
lv_point_t Display::MAIN_PAGE::line_pointsPW1[] = { {0, 0}, {34, 0}};
lv_point_t Display::MAIN_PAGE::line_pointsPW2[] = { {0, 0}, {34, 0}};
bool Display::MAIN_PAGE::checkCreate = true;

uint8_t Display::BATTERY_DJI_PAGE::step = 0;
lv_color_t Display::BATTERY_DJI_PAGE::battery_color_percent;
lv_style_t Display::BATTERY_DJI_PAGE::cell_style_border[BMS_CELL_NUM] = {};
lv_style_t Display::BATTERY_DJI_PAGE::cell_style_bg[BMS_CELL_NUM] = {};
lv_obj_t  *Display::BATTERY_DJI_PAGE::cell_bar[BMS_CELL_NUM] = {nullptr};
lv_obj_t  *Display::BATTERY_DJI_PAGE::cell_lb[BMS_CELL_NUM] = {nullptr};

uint8_t Display::SYSTEM_PAGE::step = 0;
String Display::SYSTEM_PAGE::s_temp = "";

Display::Display(HardwareSerial &debugPort) 
{
    this->debugPort = &debugPort;
}

Display::~Display(){}

bool Display::init()
{
    panel = new ESP_Panel();

    /* Initialize LVGL core */
    lv_init();

    /* Initialize LVGL buffers */
    static lv_disp_draw_buf_t draw_buf;
    /* Using double buffers is more faster than single buffer */
    /* Using internal SRAM is more fast than PSRAM (Note: Memory allocated using `malloc` may be located in PSRAM.) */
    uint8_t *buf = (uint8_t *)heap_caps_calloc(1, LVGL_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_INTERNAL);
    assert(buf);
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, LVGL_BUF_SIZE);

    /* Initialize the display device */
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = ESP_PANEL_LCD_H_RES;
    disp_drv.ver_res = ESP_PANEL_LCD_V_RES;
    disp_drv.flush_cb = lvgl_port_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

#if ESP_PANEL_USE_LCD_TOUCH && USE_LCD_TOUCH
    /* Initialize the input device */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_port_tp_read;
    lv_indev_drv_register(&indev_drv);
#endif
    /* Initialize bus and device of panel */
    panel->init();
#if ESP_PANEL_LCD_BUS_TYPE != ESP_PANEL_BUS_TYPE_RGB
    /* Register a function to notify LVGL when the panel is ready to flush */
    /* This is useful for refreshing the screen using DMA transfers */
    panel->getLcd()->setCallback(notify_lvgl_flush_ready, &disp_drv);
#endif

    /**
     * These development boards require the use of an IO expander to configure the screen,
     * so it needs to be initialized in advance and registered with the panel for use.
     *
     */
    debugPort->println("Initialize IO expander");
    /* Initialize IO expander */
    // ESP_IOExpander *expander = new ESP_IOExpander_CH422G(I2C_MASTER_NUM, ESP_IO_EXPANDER_I2C_CH422G_ADDRESS_000, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);
    expander = new ESP_IOExpander_CH422G(I2C_MASTER_NUM, ESP_IO_EXPANDER_I2C_CH422G_ADDRESS_000, 9, 8);
    expander->init();
    expander->begin();
    expander->multiPinMode(TP_RST | LCD_BL | LCD_RST | SD_CS | USB_SEL, OUTPUT);
    expander->multiDigitalWrite(TP_RST | LCD_RST, HIGH);

    expander->digitalWrite(SD_CS, LOW);
    expander->digitalWrite(LCD_BL, LOW);
    expander->digitalWrite(USB_SEL, LOW);
    /* Add into panel */
    panel->addIOExpander(expander);

    /* Start panel */
    panel->begin();

    /* Create a task to run the LVGL task periodically */
    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    xTaskCreate(lvgl_port_task, LVGL_TASKNAME, LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, &TaskDisplay_Handler);

    /* Lock the mutex due to the LVGL APIs are not thread-safe */
    lvgl_port_lock(-1);

    /* Release the mutex */
    lvgl_port_unlock();

    debugPort->println("Setup display done");
    return true;
}

#if ESP_PANEL_LCD_BUS_TYPE == ESP_PANEL_BUS_TYPE_RGB
/* Display flushing */
void Display::lvgl_port_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    panel->getLcd()->drawBitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
    lv_disp_flush_ready(disp);
}
#else
/* Display flushing */
void Display::lvgl_port_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    panel->getLcd()->drawBitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
}

bool Display::notify_lvgl_flush_ready(void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}
#endif /* ESP_PANEL_LCD_BUS_TYPE */

#if ESP_PANEL_USE_LCD_TOUCH && USE_LCD_TOUCH
/* Read the touchpad */
void Display::lvgl_port_tp_read(lv_indev_drv_t * indev, lv_indev_data_t * data)
{
    panel->getLcdTouch()->readData();

    bool touched = panel->getLcdTouch()->getTouchState();
    if(!touched) {
        data->state = LV_INDEV_STATE_REL;
    } else {
        TouchPoint point = panel->getLcdTouch()->getPoint();

        data->state = LV_INDEV_STATE_PR;
        /*Set the coordinates*/
        data->point.x = point.x;
        data->point.y = point.y;

        debugPort->printf("Touch point: x %d, y %d\n", point.x, point.y);
    }
}
#endif

void Display::lvgl_port_lock(int timeout_ms)
{
    const TickType_t timeout_ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks);
}

void Display::lvgl_port_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

void Display::lvgl_port_task(void *arg)
{
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0xffffff), LV_PART_MAIN);

    BOOT_PAGE::onCreate();

    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS/4;
    uint32_t lv_time = millis();
    uint32_t lv_time_update = millis();

    unsigned long previousMillis; // Lưu thời gian trước đó
    int lv_sys_seconds; // Biến đếm giây
    int lv_sys_minutes; // Biến đếm phút
    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        lvgl_port_lock(-1);
        task_delay_ms = lv_timer_handler();
        // Release the mutex
        lvgl_port_unlock();

        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS/4) {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS/4;
        } else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }

        switch (choose_page)
        {
        case PAGE::MAIN:
            switch (MAIN_PAGE::step)
            {
            case MAIN_PAGE::STEP::CREATE:
                MAIN_PAGE::onCreate();
                previousMillis = millis();
                MAIN_PAGE::step = MAIN_PAGE::STEP::UPDATE;
                break;
            case MAIN_PAGE::STEP::UPDATE:
                vTaskDelay((50L * configTICK_RATE_HZ) / 1000L);    
                MAIN_PAGE::onUpdate();
                page_created = true;
                if(page%5 == 1) MAIN_PAGE::step = MAIN_PAGE::STEP::DELETE;
                break;
            case MAIN_PAGE::STEP::DELETE:
                MAIN_PAGE::onDelete();
                MAIN_PAGE::step = MAIN_PAGE::STEP::CREATE;
                choose_page = PAGE::BATTERY_DJI;
                page_created = false;
                break;
            }
            break;
        case PAGE::BATTERY_DJI:
            switch (BATTERY_DJI_PAGE::step)
            {
            case BATTERY_DJI_PAGE::STEP::CREATE:
                BATTERY_DJI_PAGE::onCreate();
                BATTERY_DJI_PAGE::step = BATTERY_DJI_PAGE::STEP::UPDATE;
                break;
            case BATTERY_DJI_PAGE::STEP::UPDATE:
                vTaskDelay((50L * configTICK_RATE_HZ) / 1000L); 
                if((page%5 - 1) == 0 || (page%5 - 1) == 1) BATTERY_DJI_PAGE::onUpdate(page%5 - 1);
                page_created = true;
                if(page%5 == 3) BATTERY_DJI_PAGE::step = BATTERY_DJI_PAGE::STEP::DELETE;
                break;
            case BATTERY_DJI_PAGE::STEP::DELETE:
                BATTERY_DJI_PAGE::onDelete();
                BATTERY_DJI_PAGE::step = BATTERY_DJI_PAGE::STEP::CREATE;
                choose_page = PAGE::SYSTEM;
                page_created = false;
                break;
            }
            break;
        case PAGE::SYSTEM:
            switch (SYSTEM_PAGE::step)
            {
            case SYSTEM_PAGE::SYSTEM_PAGE::CREATE:
                SYSTEM_PAGE::onCreate();
                SYSTEM_PAGE::step = SYSTEM_PAGE::STEP::UPDATE;
                break;
            case SYSTEM_PAGE::STEP::UPDATE:
                vTaskDelay((50L * configTICK_RATE_HZ) / 1000L);  
                SYSTEM_PAGE::onUpdate();
                page_created = true;
                if(page%5 == 4) {SYSTEM_PAGE::step = SYSTEM_PAGE::STEP::DELETE; page = 0;}
                break;
            case SYSTEM_PAGE::STEP::DELETE:
                SYSTEM_PAGE::onDelete();
                SYSTEM_PAGE::step = SYSTEM_PAGE::STEP::CREATE;
                choose_page = PAGE::MAIN;
                page_created = false;
                break;
            }
            break;
        }

        if (choose_page > PAGE::BOOT)
        {
            if (millis() - previousMillis >= 1000) 
            {
                previousMillis = millis(); // Cập nhật thời gian trước đó
                lv_sys_seconds++; // Tăng biến đếm giây
                if (lv_sys_seconds >= 60) 
                {
                    lv_sys_seconds = 0; // Đặt lại giây
                    lv_sys_minutes++; // Tăng biến đếm phút
                }
                #if DISPLAY_UNIT_TEST == TRUE
                    if(lv_sys_seconds%20 == 2) MAIN_PAGE::step = MAIN_PAGE::STEP::DELETE;
                    if(lv_sys_seconds%20 == 5) BATTERY_DJI_PAGE::step = BATTERY_DJI_PAGE::STEP::DELETE;
                    if(lv_sys_seconds%20 == 9) BMS1_PAGE::step = BMS1_PAGE::STEP::DELETE;
                    if(lv_sys_seconds%20 == 13) BMS2_PAGE::step = BMS2_PAGE::STEP::DELETE;
                    if(lv_sys_seconds%20 == 17) SYSTEM_PAGE::step = SYSTEM_PAGE::STEP::DELETE;
                #endif
                FIX_FRAME::onUpdate((String)lv_sys_minutes + ":" + ((lv_sys_seconds < 10) ? "0" : "") +(String)lv_sys_seconds);
                
                for (uint8_t i = 0; i < 2; i++)
                {
                    if(batfr::capacity[i] > 0 && batfr::current[i] > 1000 && batfr::voltage[i] > 40000)
                    {
                        batfr::seconds[i]++; 
                        if (batfr::seconds[i] >= 60) 
                        {
                            batfr::seconds[i] = 0; 
                            batfr::minutes[i]++; 
                        }
                        batfr::charging[i] = true;
                    }
                    else
                    {
                        if(batfr::capacity[i] > 0) {}
                        else
                        {
                            batfr::seconds[i] = 0;
                            batfr::minutes[i] = 0; 
                        }
                        
                        batfr::charging[i] = false;
                    }
                }
            }
        }

        if(state_value == 1) 
        {
            if(lv_sys_seconds%10 == 9) page++;
            unit_test(&lv_time);
        }
        else if(state_value == 2)
        {
            batfr::reset();
            sysfr::reset();
            state_value = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

void Display::handleUpdateFirmware(bool option)
{
    if (option)
    {
        vTaskSuspend(TaskDisplay_Handler);
        panel->del();
        expander->del();
    }
    else
    {
        init();
        vTaskResume(TaskDisplay_Handler);
    }
}

void Display::on_create_lv_label(lv_obj_t **obj,
                                lv_style_t *style,
                                const lv_font_t *font, 
                                lv_align_t align,
                                lv_coord_t x_ofs, 
                                lv_coord_t y_ofs)
{
    lv_style_init(style);
    lv_style_set_text_font(style, font); 
    *obj = lv_label_create(lv_scr_act());
    lv_obj_add_style(*obj, style, 0);  // <--- obj is the label
    lv_obj_align(*obj, align, x_ofs, y_ofs);
    // clear flag
    lv_obj_clear_flag(*obj,LV_OBJ_FLAG_SCROLLABLE);
}

void Display::on_change_lv_label_text(lv_obj_t *obj, String str)
{
    char buf[100];
    lv_snprintf(buf, sizeof(buf), "%s", str);
    lv_label_set_text(obj, buf);
}

void Display::on_create_lv_img(lv_obj_t **obj, 
                                const void *src, 
                                int16_t zoom,
                                lv_align_t align, 
                                lv_coord_t x_ofs, 
                                lv_coord_t y_ofs)
{
    *obj = lv_img_create(lv_scr_act());
    lv_img_set_src(*obj, src);
    if(zoom > -1) lv_img_set_zoom(*obj, zoom);
    lv_obj_align(*obj, align, x_ofs, y_ofs);
    // clear flag
    lv_obj_clear_flag(*obj,LV_OBJ_FLAG_SCROLLABLE);
}

void Display::on_create_lv_line(lv_obj_t **obj,
                                lv_style_t *style,
                                lv_point_t line_points[],
                                lv_align_t align,
                                lv_coord_t x_ofs, 
                                lv_coord_t y_ofs)
{
    lv_style_init(style);
    lv_style_set_line_width(style, 3);
    lv_style_set_line_color(style, lv_color_make(0xB9, 0xB9, 0xB9));
    lv_style_set_line_rounded(style, true);

    /*Create a line and apply the new style*/
    *obj = lv_line_create(lv_scr_act());
    lv_line_set_points(*obj, line_points, 2);     /*Set the points*/
    lv_obj_add_style(*obj, style, 0);
    lv_obj_align(*obj, align, x_ofs, y_ofs);
    // clear flag
    lv_obj_clear_flag(*obj,LV_OBJ_FLAG_SCROLLABLE);
}

void Display::unit_test(uint32_t *_time)
{
    if((millis() - *_time) > 1000)
    {
        for (uint8_t k = 0; k < 2; k++)
        {
            switch (random(0, 5))
            {
            case 0:
                batfr::capacity[k] = BATTERY_TYPE::BATTERY_UNKNOWN;
                break;
            case 1:
                batfr::capacity[k] = BATTERY_TYPE::BATTERY_T20P;
                break;
            case 2:
                batfr::capacity[k] = BATTERY_TYPE::BATTERY_T25;
                break;
            case 3:
                batfr::capacity[k] = BATTERY_TYPE::BATTERY_T30;
                break;
            case 4:
                batfr::capacity[k] = BATTERY_TYPE::BATTERY_T40;
                break;
            case 5:
                batfr::capacity[k] = BATTERY_TYPE::BATTERY_T50;
                break;
            }
            batfr::voltage[k] = random(50000, 59000);
            batfr::current[k] = random(-50000, 138000);
            batfr::percent[k] = random(0, 100);
            batfr::temperature[k] = random(250, 800);
            batfr::countError[k] = random(0, 255);
            batfr::numberCharge[k] = random(0, 2000);
            for (uint8_t i = 0; i < 4; i++) batfr::version[k][i] = random(0, 16);   
            for (uint8_t i = 0; i < 14; i++) batfr::seriNumber[k][i] = random(48, 122);  
            for (uint8_t i = 0; i < BATTERY_DJI_CELL_NUM; i++) batfr::cell[k][i] = random(3000, 4200);

        }
        
        sysfr::num_ins = random(0, 4);
        for (uint8_t i = 0; i < 3; i++) 
        {
            sysfr::input_voltage[i] = random(21000, 24000)/100.0;
            sysfr::input_freq[i] = random(4800, 5100)/100.0;
            sysfr::input_current[i] = random(0, 1000)/100.0;
            sysfr::input_temp[i] = random(2000, 4000)/100.0;
            sysfr::efficiency[i] = random(9000, 10000)/100.0;
            sysfr::output_voltage[i] = random(5000, 6000)/100.0;
            sysfr::output_current[i] = random(0, 6000)/100.0;
            sysfr::output_current_max[i] = random(0, 6000)/100.0;
            sysfr::output_temp[i] = random(2000, 4000)/100.0;
        }
        *_time = millis();
    }
}
