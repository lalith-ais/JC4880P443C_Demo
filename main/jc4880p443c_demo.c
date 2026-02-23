/*
 * SPDX-FileCopyrightText: 2024 Your Name
 *
 * SPDX-License-Identifier: MIT
 * 
 * JC4880P443C Display Demo - ST7701 480x800 MIPI-DSI on ESP32-P4
 * 
 * This is a clean, standalone implementation for the AliExpress
 * JC4880P443C display module.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_st7701.h"
#include "esp_ldo_regulator.h"
#include "lvgl.h"
#include "jc4880p443c.h"

static const char *TAG = "JC4880P443C_DEMO";

// ============================================================================
// Hardware Configuration
// ============================================================================

// Display resolution from component
uint16_t DISPLAY_H_RES, DISPLAY_V_RES;

// Pin assignments - VERIFY THESE WITH YOUR MODULE!
#define PIN_LCD_RST                  (GPIO_NUM_5)    // LCD reset pin
#define PIN_LCD_BACKLIGHT            (GPIO_NUM_23)   // Backlight control
#define PIN_MIPI_PHY_PWR_LDO_CHAN    (3)             // LDO channel for MIPI PHY
#define PIN_MIPI_PHY_PWR_VOLTAGE_MV  (2500)          // 2.5V for MIPI PHY

// Backlight PWM configuration
#define BACKLIGHT_LEDC_CH            (0)
#define BACKLIGHT_LEDC_TIMER         (LEDC_TIMER_1)
#define BACKLIGHT_LEDC_MODE          (LEDC_LOW_SPEED_MODE)
#define BACKLIGHT_PWM_FREQ_HZ        (20000)
#define BACKLIGHT_PWM_RESOLUTION     (LEDC_TIMER_10_BIT)

// LVGL configuration
#define LVGL_TICK_PERIOD_MS          (2)
#define LVGL_TASK_MAX_DELAY_MS       (500)
#define LVGL_TASK_MIN_DELAY_MS       (1)
#define LVGL_TASK_STACK_SIZE         (6 * 1024)
#define LVGL_TASK_PRIORITY           (4)
#define LVGL_TASK_CORE               1
#define LVGL_BUFFER_SIZE             (480 * 50)  // 50 lines buffer

// ============================================================================
// Backlight Control
// ============================================================================

static esp_err_t backlight_init(void)
{
    ESP_LOGI(TAG, "Initializing backlight PWM");
    
    ledc_timer_config_t ledc_timer = {
        .speed_mode = BACKLIGHT_LEDC_MODE,
        .duty_resolution = BACKLIGHT_PWM_RESOLUTION,
        .timer_num = BACKLIGHT_LEDC_TIMER,
        .freq_hz = BACKLIGHT_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&ledc_timer), TAG, "LEDC timer config failed");
    
    ledc_channel_config_t ledc_channel = {
        .gpio_num = PIN_LCD_BACKLIGHT,
        .speed_mode = BACKLIGHT_LEDC_MODE,
        .channel = BACKLIGHT_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BACKLIGHT_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ledc_channel), TAG, "LEDC channel config failed");
    
    return ESP_OK;
}

static esp_err_t backlight_set(int brightness_percent)
{
    if (brightness_percent > 100) brightness_percent = 100;
    if (brightness_percent < 0) brightness_percent = 0;
    
    uint32_t duty_cycle = (1023 * brightness_percent) / 100;
    ESP_RETURN_ON_ERROR(ledc_set_duty(BACKLIGHT_LEDC_MODE, BACKLIGHT_LEDC_CH, duty_cycle), TAG, "LEDC set duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(BACKLIGHT_LEDC_MODE, BACKLIGHT_LEDC_CH), TAG, "LEDC update duty failed");
    
    ESP_LOGI(TAG, "Backlight set to %d%%", brightness_percent);
    return ESP_OK;
}

// ============================================================================
// MIPI DSI PHY Power
// ============================================================================

static esp_err_t mipi_phy_power_init(void)
{
    static esp_ldo_channel_handle_t phy_pwr_chan = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = PIN_MIPI_PHY_PWR_LDO_CHAN,
        .voltage_mv = PIN_MIPI_PHY_PWR_VOLTAGE_MV,
    };
    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&ldo_cfg, &phy_pwr_chan), TAG, "LDO channel acquire failed");
    ESP_LOGI(TAG, "MIPI DSI PHY powered on");
    return ESP_OK;
}

// ============================================================================
// Display Initialization
// ============================================================================

static esp_err_t display_init(esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io)
{
    esp_err_t ret = ESP_OK;
    esp_lcd_dsi_bus_handle_t dsi_bus = NULL;
    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_handle_t panel = NULL;
    
    // Get JC4880P443C configuration
    const st7701_lcd_init_cmd_t *init_cmds;
    uint16_t init_cmds_size;
    jc4880p443c_get_init_cmds(&init_cmds, &init_cmds_size);
    
    uint32_t lane_bit_rate;
    uint8_t num_lanes;
    jc4880p443c_get_dsi_config(&lane_bit_rate, &num_lanes);
    
    uint32_t pclk_mhz;
    uint16_t hbp, hfp, vbp, vfp;
    jc4880p443c_get_timing(&pclk_mhz, &hbp, &hfp, &vbp, &vfp);
    
    jc4880p443c_get_resolution(&DISPLAY_H_RES, &DISPLAY_V_RES);
    
    ESP_LOGI(TAG, "Initializing display: %dx%d", DISPLAY_H_RES, DISPLAY_V_RES);
    ESP_LOGI(TAG, "DSI: %d lanes at %ld Mbps", num_lanes, lane_bit_rate);
    ESP_LOGI(TAG, "Timing: pclk=%ldMHz, HBP=%d, HFP=%d, VBP=%d, VFP=%d", 
             pclk_mhz, hbp, hfp, vbp, vfp);
    
    // Initialize backlight and MIPI PHY
    ESP_GOTO_ON_ERROR(backlight_init(), err, TAG, "Backlight init failed");
    ESP_GOTO_ON_ERROR(mipi_phy_power_init(), err, TAG, "MIPI PHY power init failed");
    
    // Create MIPI DSI bus
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id = 0,
        .num_data_lanes = num_lanes,
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = lane_bit_rate,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_dsi_bus(&bus_config, &dsi_bus), err, TAG, "DSI bus creation failed");
    
    // Create DBI IO for commands
    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = 0,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_dbi(dsi_bus, &dbi_config, &io), err, TAG, "DBI IO creation failed");
    
    // Configure DPI video timing
    esp_lcd_dpi_panel_config_t dpi_config = {
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = pclk_mhz,
        .virtual_channel = 0,
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565,
        .num_fbs = 2,
        .video_timing = {
            .h_size = DISPLAY_H_RES,
            .v_size = DISPLAY_V_RES,
            .hsync_pulse_width = 12,
            .hsync_back_porch = hbp,
            .hsync_front_porch = hfp,
            .vsync_pulse_width = 2,
            .vsync_back_porch = vbp,
            .vsync_front_porch = vfp,
        },
        .flags.use_dma2d = true,
    };
    
    // ST7701 vendor configuration with JC4880P443C init commands
    st7701_vendor_config_t vendor_config = {
        .init_cmds = init_cmds,
        .init_cmds_size = init_cmds_size,
        .mipi_config = {
            .dsi_bus = dsi_bus,
            .dpi_config = &dpi_config,
        },
        .flags = {
            .use_mipi_interface = 1,
        },
    };
    
    // Panel device configuration
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_config,
    };
    
    // Create ST7701 panel
    ESP_LOGI(TAG, "Creating ST7701 panel with JC4880P443C init sequence");
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7701(io, &panel_config, &panel), err, TAG, "ST7701 panel creation failed");
    
    // Reset and initialize the panel
    ESP_GOTO_ON_ERROR(esp_lcd_panel_reset(panel), err, TAG, "Panel reset failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(panel), err, TAG, "Panel init failed");
    
    // Turn on display
    ESP_GOTO_ON_ERROR(esp_lcd_panel_disp_on_off(panel, true), err, TAG, "Display on failed");
    
    *ret_panel = panel;
    *ret_io = io;
    
    ESP_LOGI(TAG, "Display initialized successfully");
    return ESP_OK;

err:
    if (panel) esp_lcd_panel_del(panel);
    if (io) esp_lcd_panel_io_del(io);
    if (dsi_bus) esp_lcd_del_dsi_bus(dsi_bus);
    return ret;
}

// ============================================================================
// LVGL Port
// ============================================================================

static void lvgl_tick_increment(void *arg)
{
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static esp_err_t lvgl_tick_init(void)
{
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvgl_tick_increment,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    return esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000);
}

static void lvgl_flush_callback(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map);
    lv_display_flush_ready(disp);
}

static void lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "LVGL task started");
    
    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        task_delay_ms = lv_timer_handler();
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

// ============================================================================
// LVGL Port - Updated for LVGL 9.5.0
// ============================================================================

static lv_display_t *lvgl_display_init(esp_lcd_panel_handle_t panel)
{
    lv_init();
    ESP_ERROR_CHECK(lvgl_tick_init());
    
    // Create display
    lv_display_t *disp = lv_display_create(DISPLAY_H_RES, DISPLAY_V_RES);
    assert(disp);
    
    // Allocate draw buffers
    void *buf1 = heap_caps_malloc(LVGL_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    void *buf2 = heap_caps_malloc(LVGL_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 && buf2);
    
    // LVGL 9.5.0 uses lv_draw_buf_t for buffers
    lv_draw_buf_t *draw_buf1 = lv_draw_buf_create(LVGL_BUFFER_SIZE, 1, 
                                                   LV_COLOR_FORMAT_RGB565, 
                                                   LV_STRIDE_AUTO);
    lv_draw_buf_t *draw_buf2 = lv_draw_buf_create(LVGL_BUFFER_SIZE, 1, 
                                                   LV_COLOR_FORMAT_RGB565, 
                                                   LV_STRIDE_AUTO);
    
    // Alternative simpler method for LVGL 9.5.0
    lv_display_set_buffers(disp, buf1, buf2, LVGL_BUFFER_SIZE * sizeof(lv_color_t), 
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    
    lv_display_set_flush_cb(disp, lvgl_flush_callback);
    lv_display_set_user_data(disp, panel);
    
    return disp;
}

// ============================================================================
// Demo UI
// ============================================================================

static void create_demo_ui(lv_display_t *disp)
{
    lv_obj_t *scr = lv_display_get_screen_active(disp);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x1a1a2e), 0);
    
    // Title
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "JC4880P443C\n480x800 ST7701 Display");
    lv_obj_set_style_text_color(title, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 30);
    
    // Color test rectangles
    lv_obj_t *rect_r = lv_obj_create(scr);
    lv_obj_set_size(rect_r, 100, 100);
    lv_obj_set_style_bg_color(rect_r, lv_color_hex(0xff0000), 0);
    lv_obj_align(rect_r, LV_ALIGN_LEFT_MID, 30, 0);
    
    lv_obj_t *rect_g = lv_obj_create(scr);
    lv_obj_set_size(rect_g, 100, 100);
    lv_obj_set_style_bg_color(rect_g, lv_color_hex(0x00ff00), 0);
    lv_obj_align(rect_g, LV_ALIGN_CENTER, 0, 0);
    
    lv_obj_t *rect_b = lv_obj_create(scr);
    lv_obj_set_size(rect_b, 100, 100);
    lv_obj_set_style_bg_color(rect_b, lv_color_hex(0x0000ff), 0);
    lv_obj_align(rect_b, LV_ALIGN_RIGHT_MID, -30, 0);
    
    // Info label
    lv_obj_t *info = lv_label_create(scr);
    lv_label_set_text(info, "Clean implementation\nValidated init sequence\nLVGL 9 ready");
    lv_obj_set_style_text_color(info, lv_color_hex(0x88ff88), 0);
    lv_obj_align(info, LV_ALIGN_BOTTOM_MID, 0, -30);
}

// ============================================================================
// Main Entry Point
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "JC4880P443C Display Demo - ST7701 480x800");
    ESP_LOGI(TAG, "Clean implementation - No hacked BSP code");
    ESP_LOGI(TAG, "========================================");
    
    esp_lcd_panel_handle_t panel = NULL;
    esp_lcd_panel_io_handle_t io = NULL;
    
    esp_err_t ret = display_init(&panel, &io);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Display initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_ERROR_CHECK(backlight_set(100));
    
    lv_display_t *disp = lvgl_display_init(panel);
    assert(disp);
    
    create_demo_ui(disp);
    
    xTaskCreatePinnedToCore(
        lvgl_port_task,
        "lvgl",
        LVGL_TASK_STACK_SIZE,
        NULL,
        LVGL_TASK_PRIORITY,
        NULL,
        LVGL_TASK_CORE
    );
    
    ESP_LOGI(TAG, "System ready! Display running at %dx%d", DISPLAY_H_RES, DISPLAY_V_RES);
}
