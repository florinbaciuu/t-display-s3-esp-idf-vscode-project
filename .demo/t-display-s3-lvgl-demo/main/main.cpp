// comment here
#define BOARD_TFT_DATA0    (39)  // GPIO pin for TFT data line 0
#define BOARD_TFT_DATA1    (40)  // GPIO pin for TFT data line 1
#define BOARD_TFT_DATA2    (41)  // GPIO pin for TFT data line 2
#define BOARD_TFT_DATA3    (42)  // GPIO pin for TFT data line 3
#define BOARD_TFT_DATA4    (45)  // GPIO pin for TFT data line 4
#define BOARD_TFT_DATA5    (46)  // GPIO pin for TFT data line 5
#define BOARD_TFT_DATA6    (47)  // GPIO pin for TFT data line 6
#define BOARD_TFT_DATA7    (48)  // GPIO pin for TFT data line 7
#define BOARD_TFT_RST      (5)   // GPIO pin for TFT reset, set to -1 if not used
#define BOARD_TFT_CS       (6)   // GPIO pin for TFT chip select
#define BOARD_TFT_DC       (7)   // GPIO pin for TFT data/command control
#define BOARD_TFT_WR       (8)   // GPIO pin for TFT write control
#define BOARD_TFT_RD       (9)
#define LCD_WIDTH          (320)               // Width of the LCD in pixels
#define LCD_HEIGHT         (170)               // Height of the LCD in pixels
#define LCD_PIXEL_CLOCK_HZ (10 * 1000 * 1000)  // LCD pixel clock frequency in Hz
#define BOARD_TFT_BL       (38)                // GPIO pin for backlight control
//---------
#define PWR_ON_PIN (15)
#define PWR_EN_PIN (PWR_ON_PIN)
//---------

/*********************
 *    LVGL DEFINES
 *********************/

#define TICK_INCREMENTATION 5  // in ms(milliseconds) must be equal incrementation with delay
#define LV_TIMER_INCREMENT  TICK_INCREMENTATION * 1000
#define LV_DELAY            TICK_INCREMENTATION
#define LV_NO_OOP           esp_rom_delay_us(100);

//-----------------------------------------------------------
#define LV_TICK_SOURCE_TIMER    0
#define LV_TICK_SOURCE_TASK     1
#define LV_TICK_SOURCE_CALLBACK 2
#ifndef LV_TICK_SOURCE
#    define LV_TICK_SOURCE (LV_TICK_SOURCE_TASK)
#endif /* #ifndef LV_TICK_SOURCE */
//---------
//---------
/*Where flush_ready must to go : in display_flush or in io_trans_done_cb*/
////#define flush_ready_in_disp_flush // nu e asa bun
#define flush_ready_in_io_trans_done  // mult mai bine asa deoarece se da flush ready in momentul cand display face trans_done
//---------

//-----------------------------------------------------------
//---------

//-----------------------------------------------------------

/* BUFFER MODE */
#define BUFFER_20LINES     1
#define BUFFER_40LINES     2
#define BUFFER_60LINES     3  // merge
#define BUFFER_DEVIDED4    4
#define BUFFER_FULL        5            // merge super ok
#define BUFFER_MODE        BUFFER_FULL  // selecteaza modul de buffer , defaut este BUFFER_FULL
#define DOUBLE_BUFFER_MODE true
//---------
/* BUFFER MEMORY TYPE AND DMA */
#define BUFFER_INTERNAL 1
#define BUFFER_SPIRAM   2
#define BUFFER_MEM      BUFFER_SPIRAM
#if (BUFFER_MEM == BUFFER_INTERNAL)
#    define DMA_ON (true)
#endif /* #if (BUFFER_MEM == BUFFER_INTERNAL) */
//---------

//-----------------------------------------------------------
/* RENDER MODE */
// Aici e mod mai special pt ca se transmite direct functiei....
#define RENDER_MODE_PARTIAL (LV_DISPLAY_RENDER_MODE_PARTIAL)
#define RENDER_MODE_FULL    (LV_DISPLAY_RENDER_MODE_FULL)
#define RENDER_MODE_DIRECT  (LV_DISPLAY_RENDER_MODE_DIRECT)

#define RENDER_MODE (RENDER_MODE_PARTIAL)
//--------------------- --------------------------------------

//---------

/*********************
 *      INCLUDES
 *********************/
extern "C" {
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "esp_bootloader_desc.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lvgl.h"
#include <lv_conf.h>

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"  // Sau driverul real folosit de tines
}
/**********************
 *   GLOBAL VARIABLES
 **********************/
RTC_DATA_ATTR static uint32_t boot_count      = 0;
esp_lcd_panel_io_handle_t     touch_io_handle = NULL;
esp_lcd_panel_io_handle_t     lcd_io_handle   = NULL;
esp_lcd_i80_bus_handle_t      i80_bus         = NULL;
esp_lcd_panel_handle_t        panel_handle    = NULL;
//---------

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
//---------
void power_latch_init() {
    gpio_config_t io_conf = {.pin_bit_mask = 1ULL << PWR_ON_PIN,
        .mode                              = GPIO_MODE_OUTPUT,
        .pull_up_en                        = GPIO_PULLUP_DISABLE,
        .pull_down_en                      = GPIO_PULLDOWN_DISABLE,
        .intr_type                         = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t) PWR_ON_PIN, 1);  // ⚡ ține placa aprinsă
}
//---------
void gfx_set_backlight(uint32_t mode) {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BOARD_TFT_BL,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t) BOARD_TFT_BL, mode);
}
//---------

// #define LVGL_BENCH_TEST

#ifdef LVGL_BENCH_TEST
// --- stats flush (ISR-safe) ---
volatile uint64_t g_flush_tstart_us = 0;
volatile uint32_t g_flush_bytes     = 0;
volatile uint32_t g_flush_last_us   = 0;
volatile uint64_t g_flush_total_us  = 0;
volatile uint32_t g_flush_count     = 0;

// pentru log la 1s (din task, nu din ISR)
static uint32_t g_log_last_tick = 0;

#endif /* #if LVGL_BENCH_TEST */

/**********************
 *   LVGL VARIABLES
 **********************/
uint32_t      bufSize;           // Dimensiunea buffer-ului
lv_color_t*   disp_draw_buf;     // Buffer LVGL
lv_color_t*   disp_draw_buf_II;  // Buffer LVGL secundar
lv_display_t* disp;              // Display LVGL

/**********************
 *   LVGL FUNCTIONS
 **********************/

/* Display flushing function callback */
void lv_disp_flush(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
#ifdef LVGL_BENCH_TEST
    // dimensiune reală a zonei în bytes
    g_flush_bytes     = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1) * sizeof(lv_color_t);
    g_flush_tstart_us = esp_timer_get_time();  // ISR-safe
#endif                                         /* #if LVGL_BENCH_TEST */
    esp_lcd_panel_draw_bitmap(
        panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, (const void*) px_map);
#ifdef flush_ready_in_disp_flush
    lv_disp_flush_ready(disp);
#endif /* #ifdef (flush_ready_in_disp_flush) */
}
//---------
// ############  GLOBALE LVGL ##########//
lv_obj_t* tabview = NULL;

void lvgl_ui_function(void) {
    // Creăm containerul de taburi
    tabview = lv_tabview_create(lv_screen_active());
    lv_tabview_set_tab_bar_size(tabview, 40);            // Setăm înălțimea tab-urilor
    lv_obj_set_size(tabview, LV_PCT(100), LV_PCT(100));  // Setăm dimensiunea tabview-ului
    lv_obj_set_flex_flow(tabview, LV_FLEX_FLOW_COLUMN);  // Setăm flex flow pentru tabview
    lv_obj_set_flex_grow(tabview, 1);                    // Permitem tabview-ului să ocupe tot spațiul disponibil
    lv_dir_t dir = LV_DIR_TOP;                           // Poziționăm tab-urile în partea de sus
    lv_tabview_set_tab_bar_position(
        tabview, dir);  // Funcția nu există în LVGL, deci comentăm această linie

    /*Adăugăm 3 taburi*/
    lv_obj_t* tab1 = lv_tabview_add_tab(tabview, "Tab 1");
    lv_obj_t* tab2 = lv_tabview_add_tab(tabview, "Tab 2");
    lv_obj_t* tab3 = lv_tabview_add_tab(tabview, "Tab 3");

    // TAB 1

    // TAB 2

    // TAB 3
}
//---------

//---------
bool panel_io_trans_done_callback(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t* edata, void* user_ctx) {
    // old rau.
    // if (disp != NULL) {
    // } else {
    //     esp_rom_printf("[lv_trans_done_cb] - disp este NULL!\n");
    // }
    // return false;  // false înseamnă: nu mai face nimic după
#ifdef LVGL_BENCH_TEST
    // calcule rapide, ISR-safe
    uint32_t end_us     = (uint32_t) esp_timer_get_time();
    uint32_t elapsed_us = end_us - (uint32_t) g_flush_tstart_us;

    g_flush_last_us     = elapsed_us;
    g_flush_total_us += elapsed_us;
    g_flush_count++;
#endif /* #ifdef LVGL_BENCH_TEST */
#ifdef flush_ready_in_io_trans_done
    lv_display_t* d = (lv_display_t*) user_ctx;
    if (d)
        lv_disp_flush_ready(d);  // <— mutat aici
#endif                           /* #ifdef (flush_ready_in_io_trans_done) */
    return false;                // nu mai face nimic după
}
//---------
#if LV_TICK_SOURCE == LV_TICK_SOURCE_CALLBACK
uint32_t lv_get_rtos_tick_count_callback(void) {
    return xTaskGetTickCount();
}  // Callback pentru a obține numărul de tick-uri RTOS
#endif /* #if LV_TICK_SOURCE == LV_TICK_SOURCE_CALLBACK */
//--------------------------------------

/*
███████ ██████  ███████ ███████ ██████ ████████  ██████  ███████ 
██      ██   ██ ██      ██      ██   ██   ██    ██    ██ ██      
█████   ██████  █████   █████   ██████    ██    ██    ██ ███████ 
██      ██   ██ ██      ██      ██   ██   ██    ██    ██      ██ 
██      ██   ██ ███████ ███████ ██   ██   ██     ██████  ███████ 
*/

/*********************
 *  rtos variables
 *********************/
TaskHandle_t xHandle_lv_main_task;
TaskHandle_t xHandle_lv_main_tick_task;
TaskHandle_t xHandle_chechButton0State;

// -------------------------------

// lbgl semafor principal (draw)
SemaphoreHandle_t s_lvgl_mutex;

bool s_lvgl_port_init_locking(void) {
    s_lvgl_mutex = xSemaphoreCreateRecursiveMutex();
    return (s_lvgl_mutex != NULL);
}

bool s_lvgl_lock(uint32_t timeout_ms) {
    if (!s_lvgl_mutex)
        return false;
    return xSemaphoreTakeRecursive(s_lvgl_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void s_lvgl_unlock(void) {
    if (s_lvgl_mutex)
        xSemaphoreGiveRecursive(s_lvgl_mutex);
}

// -------------------------------

/************************************************** */

#if LV_TICK_SOURCE == LV_TICK_SOURCE_TIMER

static void lv_tick_timer_callback_func(void* arg) {
    lv_tick_inc((TICK_INCREMENTATION));
}

static void lv_tick_start_timer(void) {
    const esp_timer_create_args_t timer_args = {
        .callback              = &lv_tick_timer_callback_func,
        .dispatch_method       = ESP_TIMER_TASK,
        .name                  = "lv_tick",
        .skip_unhandled_events = true};
    esp_timer_handle_t tick_timer;

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, LV_TIMER_INCREMENT));
}

void lv_main_task(void* parameter) {
    xHandle_lv_main_task   = xTaskGetCurrentTaskHandle();
    static TickType_t tick = 0;
    tick                   = xTaskGetTickCount();
    while (true) {
        if (s_lvgl_lock(portMAX_DELAY)) {
            lv_timer_handler();
        }
        vTaskDelayUntil(&tick, pdMS_TO_TICKS(LV_DELAY));
    }
}

#elif LV_TICK_SOURCE == LV_TICK_SOURCE_TASK

/********************************************** */
/*                   TASK                       */
/********************************************** */
void lv_main_tick_task(void* parameter) {
    static TickType_t tick = 0;
    tick                   = xTaskGetTickCount();
    while (true) {
        lv_tick_inc(TICK_INCREMENTATION);  // Incrementeaza tick-urile LVGL in ms
        xTaskDelayUntil(&tick, pdMS_TO_TICKS(LV_DELAY)); // Delay precis mult mai rapid asa
    }
}

/********************************************** */
/*                   TASK                       */
/********************************************** */
void lv_main_task(void* parameter) {
    xHandle_lv_main_task   = xTaskGetCurrentTaskHandle();
    static TickType_t tick = 0;
    tick                   = xTaskGetTickCount();  // Inițializare corectă
    while (true) {
        if (s_lvgl_lock(portMAX_DELAY)) {
            lv_timer_handler();
            s_lvgl_unlock();
        }
        xTaskDelayUntil(&tick, pdMS_TO_TICKS(LV_DELAY));  // delay doar aici
    }
}

#endif /* #if LV_TICK_SOURCE == LV_TICK_SOURCE_TIMER */

/********************************************** */
/*                   TASK                       */
/********************************************** */
static void IRAM_ATTR chechButton0State_isr_handler(void* arg) {
    // NOTĂ: NU face log sau delay aici
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR((TaskHandle_t) arg, 0x01, eSetBits, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}
//---------
void chechButton0State(void* parameter) {
    (void) parameter;
    xHandle_chechButton0State = xTaskGetCurrentTaskHandle();
    uint32_t notificationValue;

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << 0,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        //.intr_type    = GPIO_INTR_ANYEDGE, // si la apasare si la eliberare
        .intr_type = GPIO_INTR_NEGEDGE,  // doar la apasare
        //.intr_type    = GPIO_INTR_POSEDGE, // doar la eliberare
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    gpio_isr_handler_add((gpio_num_t) 0, chechButton0State_isr_handler, (void*) xHandle_chechButton0State);

    static uint8_t current_tab = 0;

    while (true) {
        xTaskNotifyWait(0x00, 0xFFFFFFFF, &notificationValue, portMAX_DELAY); // asteapta notificarea din ISR
        if (notificationValue & 0x01) {
            ESP_LOGW("BUTTON", "Button ACTIVAT pe GPIO0");

            current_tab++;
            if (current_tab > 2)
                current_tab = 0;

            // schimbă tab-ul LVGL
            if (s_lvgl_lock(30)) {  // protejează LVGL cu mutex-ul tău
                lv_tabview_set_act(tabview, current_tab, LV_ANIM_ON);
                s_lvgl_unlock();
            }
        }
        vTaskDelay(200);
    }
}
/****************************/

//--------------------------------------

/*
███    ███  █████  ██ ███    ██ 
████  ████ ██   ██ ██ ████   ██ 
██ ████ ██ ███████ ██ ██ ██  ██ 
██  ██  ██ ██   ██ ██ ██  ██ ██ 
██      ██ ██   ██ ██ ██   ████ 
  * This is the main entry point of the application.
  * It initializes the hardware, sets up the display, and starts the LVGL tasks.
  * The application will run indefinitely until the device is powered off or reset.
*/
extern "C" void app_main(void) {
    power_latch_init();  // Inițializare latch pentru alimentare
    gfx_set_backlight(1);
    esp_log_level_set("*", ESP_LOG_INFO);

    gpio_reset_pin((gpio_num_t) BOARD_TFT_RD);
    gpio_set_direction((gpio_num_t) BOARD_TFT_RD, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t) BOARD_TFT_RD, 1);

    printf("\n");

    lv_init();

#if LV_TICK_SOURCE == LV_TICK_SOURCE_CALLBACK
    // Next function comment because create problems with lvgl timers and esp32 timers
    lv_tick_set_cb(lv_get_rtos_tick_count_callback);
#endif /* #if LV_TICK_SOURCE == LV_TICK_SOURCE_CALLBACK */

    disp = lv_display_create(
        (int32_t) LCD_WIDTH,
        (int32_t) LCD_HEIGHT);

    esp_lcd_i80_bus_config_t lcd_bus_config = {.dc_gpio_num = BOARD_TFT_DC,
        .wr_gpio_num                                        = BOARD_TFT_WR,
        .clk_src                                            = LCD_CLK_SRC_DEFAULT,
        .data_gpio_nums                                     = {
            BOARD_TFT_DATA0,
            BOARD_TFT_DATA1,
            BOARD_TFT_DATA2,
            BOARD_TFT_DATA3,
            BOARD_TFT_DATA4,
            BOARD_TFT_DATA5,
            BOARD_TFT_DATA6,
            BOARD_TFT_DATA7,
        },
        .bus_width          = 8,
        .max_transfer_bytes = LCD_WIDTH * LCD_HEIGHT * sizeof(uint16_t),
        .psram_trans_align  = 64,
        .sram_trans_align   = 4};
    // esp_lcd_new_i80_bus(&lcd_bus_config, &i80_bus);
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&lcd_bus_config, &i80_bus));

    esp_lcd_panel_io_i80_config_t lcd_io_config = {
        .cs_gpio_num         = BOARD_TFT_CS,
        .pclk_hz             = 10000000,
        .trans_queue_depth   = 10,
        .on_color_trans_done = panel_io_trans_done_callback,
        //.user_ctx            = NULL,
        .user_ctx       = disp,
        .lcd_cmd_bits   = 8,
        .lcd_param_bits = 8,
        .dc_levels      = {
                 .dc_idle_level  = 0,
                 .dc_cmd_level   = 0,
                 .dc_dummy_level = 0,
                 .dc_data_level  = 1,
        },
        .flags = {
            .cs_active_high     = 0,
            .reverse_color_bits = 0,
            .swap_color_bytes   = 0,
            .pclk_active_neg    = 0,
            .pclk_idle_low      = 0,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &lcd_io_config, &lcd_io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BOARD_TFT_RST,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian    = LCD_RGB_DATA_ENDIAN_LITTLE,  // .swap_color_bytes   = 1,
        .bits_per_pixel = 16,
        .flags          = {
                     .reset_active_high = 0,
        },
        .vendor_config = NULL,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(lcd_io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 0, 35));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

#if (BUFFER_MODE == BUFFER_FULL)
    bufSize = ((LCD_WIDTH * LCD_HEIGHT) * lv_color_format_get_size(lv_display_get_color_format(disp)));
#elif (BUFFER_MODE == BUFFER_60LINES)
    bufSize = ((LCD_WIDTH * 60) * lv_color_format_get_size(lv_display_get_color_format(disp)));
#elif (BUFFER_MODE == BUFFER_40LINES)
    bufSize = ((LCD_WIDTH * 40) * lv_color_format_get_size(lv_display_get_color_format(disp)));
#elif (BUFFER_MODE == BUFFER_20LINES)
    bufSize = ((LCD_WIDTH * 20) * lv_color_format_get_size(lv_display_get_color_format(disp)));
#elif (BUFFER_MODE == BUFFER_DEVIDED4)
    bufSize = ((LCD_WIDTH * LCD_HEIGHT) * lv_color_format_get_size(lv_display_get_color_format(disp)) / 4);
#endif
#if (BUFFER_MEM == BUFFER_SPIRAM)
#    if (DOUBLE_BUFFER_MODE == 1)
    disp_draw_buf    = (lv_color_t*) heap_caps_malloc(bufSize, MALLOC_CAP_SPIRAM);
    disp_draw_buf_II = (lv_color_t*) heap_caps_malloc(bufSize, MALLOC_CAP_SPIRAM);
    ESP_LOGI("LVGL", "LVGL buffers created in SPIRAM");
#    else
    disp_draw_buf = (lv_color_t*) heap_caps_malloc(bufSize, MALLOC_CAP_SPIRAM);
#    endif
#elif (BUFFER_MEM == BUFFER_INTERNAL)
#    if (DMA_ON == 1)
#        if (DOUBLE_BUFFER_MODE == 1)
    disp_draw_buf    = (lv_color_t*) heap_caps_malloc(bufSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    disp_draw_buf_II = (lv_color_t*) heap_caps_malloc(bufSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    ESP_LOGI("LVGL", "LVGL buffers created in SPIRAM");
#        else
    disp_draw_buf = (lv_color_t*) heap_caps_malloc(bufSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
#        endif
#    else
#        if (DOUBLE_BUFFER_MODE == 1)
    disp_draw_buf    = (lv_color_t*) heap_caps_malloc(bufSize, MALLOC_CAP_INTERNAL);
    disp_draw_buf_II = (lv_color_t*) heap_caps_malloc(bufSize, MALLOC_CAP_INTERNAL);
    ESP_LOGI("LVGL", "LVGL buffers created in SPIRAM");
#        else
    disp_draw_buf = (lv_color_t*) heap_caps_malloc(bufSize, MALLOC_CAP_INTERNAL);
#        endif
#    endif
#endif /* #if (BUFFER_MEM == BUFFER_SPIRAM) */

    // --- memset pentru curățare frame-uri ---
    // Asta îți garantează că primul frame e complet „negru”

    if (disp_draw_buf) {
        memset(disp_draw_buf, 0, bufSize);
    }
    if (disp_draw_buf_II) {
        memset(disp_draw_buf_II, 0, bufSize);
    }

    if (!disp_draw_buf) {  // VERIFICA DACA PRIMUL BUFFER ESTE CREAT
        ESP_LOGE("LVGL", "LVGL disp_draw_buf allocate failed!");
    }
#if (DOUBLE_BUFFER_MODE == 1)
    if (!disp_draw_buf_II) {  // VERIFICA DACA AL DOILEA BUFFER ESTE CREAT
        ESP_LOGE("LVGL", "LVGL disp_draw_buf_II allocate failed!");
    }
#endif

#if (DOUBLE_BUFFER_MODE == 1)
    lv_display_set_buffers(
        disp, disp_draw_buf, disp_draw_buf_II, bufSize, (lv_display_render_mode_t) RENDER_MODE);
    ESP_LOGI("LVGL", "LVGL buffers set");
#else
    lv_display_set_buffers(disp, disp_draw_buf, NULL, bufSize, (lv_display_render_mode_t) RENDER_MODE);
#endif

    lv_display_set_resolution(disp, LCD_WIDTH, LCD_HEIGHT);           // Seteaza rezolutia software
    lv_display_set_physical_resolution(disp, LCD_WIDTH, LCD_HEIGHT);  // Actualizeaza rezolutia reala
    lv_display_set_rotation(disp, (lv_display_rotation_t) 0);         // Seteaza rotatia lvgl
    lv_display_set_render_mode(disp,
        (lv_display_render_mode_t) RENDER_MODE);  // Seteaza (lv_display_render_mode_t)
    lv_display_set_antialiasing(disp, true);      // Antialiasing DA sau NU
    ESP_LOGI("LVGL", "LVGL display settings done");

    lv_display_set_flush_cb(disp, lv_disp_flush);  // Set the flush callback which will be called to
                                                   // copy the rendered image to the display.
    ESP_LOGI("LVGL", "LVGL display flush callback set");

    vTaskDelay(500);

    s_lvgl_port_init_locking();
    esp_rom_delay_us(100);
    s_lvgl_lock(0);
    // lvgl code here
    lvgl_ui_function();
    s_lvgl_unlock();
    esp_rom_delay_us(100);

    xTaskCreatePinnedToCore(lv_main_task,        // Functia task-ului
        (const char*) "LVGL Main Task",          // Numele task-ului
        (uint32_t) (4096 + 4096),                // Dimensiunea stack-ului
        (NULL),                                  // Parametri (daca exista)
        (UBaseType_t) configMAX_PRIORITIES - 4,  // Prioritatea task-ului // 3
        &xHandle_lv_main_task,                   // Handle-ul task-ului
        ((1))                                    // Nucleul pe care ruleaza task-ul
    );

#if LV_TICK_SOURCE == LV_TICK_SOURCE_TIMER
    lv_tick_start_timer();
#elif LV_TICK_SOURCE == LV_TICK_SOURCE_TASK
    xTaskCreatePinnedToCore(lv_main_tick_task,   // Functia care ruleaza task-ul
        (const char*) "LVGL Tick Task",          // Numele task-ului
        (uint32_t) (2048 + 1024),                // Dimensiunea stack-ului
        (NULL),                                  // Parametri
        (UBaseType_t) configMAX_PRIORITIES - 2,  // Prioritatea task-ului // 1
        &xHandle_lv_main_tick_task,              // Handle-ul task-ului
        ((1))                                    // Nucleul pe care ruleaza (ESP32 e dual-core)
    );
#endif /* #if LV_TICK_SOURCE == LV_TICK_SOURCE_TIMER */

    vTaskDelay(500);
    printf("HAH\n");

#ifdef LVGL_BENCH_TEST
    esp_rom_delay_us(100);
    xTaskCreatePinnedToCore(lv_bench_task, "lvBench", 4096, NULL, tskIDLE_PRIORITY + 1, NULL, 1);
#endif /* #ifdef LVGL_BENCH_TEST */

    xTaskCreatePinnedToCore(chechButton0State,   // Functia care ruleaza task-ul
        (const char*) "v_check_0_pin_state",     // Numele task-ului
        (uint32_t) (4096),                       // Dimensiunea stack-ului
        (NULL),                                  // Parametri
        (UBaseType_t) configMAX_PRIORITIES - 7,  // Prioritatea task-ului // 6
        &xHandle_chechButton0State,              // Handle-ul task-ului
        ((1))                                    // Nucleul pe care ruleaza (ESP32 e dual-core)
    );

    printf("T\n");
    esp_rom_delay_us(100);
    printf("Aici aplicatia ar trebui sa returneze.Meh\n");
    vTaskDelay(100);
    printf("Aici aplicatia ar trebui sa returneze.Meh\n");
    printf("Si totusi am observat ca nu returneaza imediat .. ci mai astepata putin .\n");
}
