#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_interface.h"
#include "esp_log.h"

#include "qmsd_lcd_panel_rgb.h"
#include "qmsd_board.h"
#include "sdkconfig.h"

#include "screen_driver.h"

esp_lcd_panel_handle_t g_rgb_panel_handle = NULL;

#ifndef CONFIG_QMSD_RGB_BOUNCE_BUFFER_LINES
#define CONFIG_QMSD_RGB_BOUNCE_BUFFER_LINES 16
#endif

#if QMSD_SCREEN_HIGHT % CONFIG_QMSD_RGB_BOUNCE_BUFFER_LINES
#error "CONFIG_QMSD_RGB_BOUNCE_BUFFER_LINES must evenly divide QMSD_SCREEN_HIGHT"
#endif

#if CONFIG_QMSD_RGB_PANEL_PHASE_TELEMETRY
static const char *TAG = "lcd_panel.rgb";

#ifdef CONFIG_LCD_RGB_RESTART_IN_VSYNC
#define RGB_CFG_RESTART_IN_VSYNC 1
#else
#define RGB_CFG_RESTART_IN_VSYNC 0
#endif

#ifdef CONFIG_QMSD_RGB_RESTART_ON_PHASE_DESYNC
#define RGB_CFG_RESTART_ON_PHASE_DESYNC 1
#else
#define RGB_CFG_RESTART_ON_PHASE_DESYNC 0
#endif

#ifdef CONFIG_QMSD_SCREEN_20_MHZ_CLK
#define RGB_CFG_SCREEN_20_MHZ 1
#else
#define RGB_CFG_SCREEN_20_MHZ 0
#endif

#ifdef CONFIG_QMSD_GUI_DIRECT_MODE
#define RGB_CFG_GUI_DIRECT_MODE 1
#else
#define RGB_CFG_GUI_DIRECT_MODE 0
#endif

#ifdef CONFIG_QMSD_GUI_FULL_REFRESH
#define RGB_CFG_GUI_FULL_REFRESH 1
#else
#define RGB_CFG_GUI_FULL_REFRESH 0
#endif
#endif

static esp_err_t qmsd_screen_init(const scr_controller_config_t *lcd_conf);
static esp_err_t qmsd_screen_drawbitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *bitmap);
static esp_err_t qmsd_screen_draw_pixel(uint16_t x, uint16_t y, uint16_t color);
#if CONFIG_QMSD_RGB_PANEL_PHASE_TELEMETRY
static void qmsd_screen_log_rgb_config(const qmsd_lcd_rgb_panel_config_t *panel_config);
#endif

scr_driver_t qmsd_lcd_rgb_driver = {
    .init = qmsd_screen_init,
    .deinit = NULL,
    .set_direction = NULL,
    .set_window = NULL,
    .write_ram_data = NULL,
    .draw_pixel = qmsd_screen_draw_pixel,
    .draw_bitmap = qmsd_screen_drawbitmap,
    .get_info = NULL,
};

static esp_err_t qmsd_screen_init(const scr_controller_config_t *lcd_conf) {
    (void)lcd_conf;
    qmsd_lcd_rgb_panel_config_t panel_config = {
        .data_width = 16,
        .disp_gpio_num = -1,
        .pclk_gpio_num = LCD_PCLK_GPIO,
        .vsync_gpio_num = LCD_VSYNC_GPIO,
        .hsync_gpio_num = LCD_HSYNC_GPIO,
        .de_gpio_num = LCD_DE_GPIO,
        .data_gpio_nums = {
            LCD_D0_GPIO,
            LCD_D1_GPIO,
            LCD_D2_GPIO,
            LCD_D3_GPIO,
            LCD_D4_GPIO,
            LCD_D5_GPIO,
            LCD_D6_GPIO,
            LCD_D7_GPIO,
            LCD_D8_GPIO,
            LCD_D9_GPIO,
            LCD_D10_GPIO,
            LCD_D11_GPIO,
            LCD_D12_GPIO,
            LCD_D13_GPIO,
            LCD_D14_GPIO,
            LCD_D15_GPIO,
        },
        .bounce_buffer_size_px = QMSD_RGB_CLK_FREQ > 15000000
                                      ? QMSD_SCREEN_WIDTH * CONFIG_QMSD_RGB_BOUNCE_BUFFER_LINES
                                      : 0,
        .clk_src = USER_RGB_CLK_SRC_PLL240M,
        .timings = {
            .pclk_hz = QMSD_RGB_CLK_FREQ,
            .h_res = QMSD_SCREEN_WIDTH,
            .v_res = QMSD_SCREEN_HIGHT,
            .hsync_pulse_width = QMSD_RGB_HSYNC_PULSE_WIDTH,
            .hsync_back_porch = QMSD_RGB_HSYNC_BACK_PORCH,
            .hsync_front_porch = QMSD_RGB_HSYNC_FRONT_PORCH,
            .vsync_pulse_width = QMSD_RGB_VSYNC_PULSE_WIDTH,
            .vsync_back_porch = QMSD_RGB_VSYNC_BACK_PORCH,
            .vsync_front_porch = QMSD_RGB_VSYNC_FRONT_PORCH,
        },

        .flags = {
            .fb_in_psram = 1,
            .double_fb = 1,
            .avoid_te = 1,
        },
    };

    extern void qmsd_rgb_spi_init();
    qmsd_rgb_spi_init();

    qmsd_lcd_new_rgb_panel(&panel_config, &g_rgb_panel_handle);
    g_rgb_panel_handle->reset(g_rgb_panel_handle);
    esp_err_t ret = g_rgb_panel_handle->init(g_rgb_panel_handle);
#if CONFIG_QMSD_RGB_PANEL_PHASE_TELEMETRY
    qmsd_screen_log_rgb_config(&panel_config);
#endif
    return ret;
}

static esp_err_t qmsd_screen_draw_pixel(uint16_t x, uint16_t y, uint16_t color) {
    return g_rgb_panel_handle->draw_bitmap(g_rgb_panel_handle, x, y, x + 1, y + 1, &color);
}

static esp_err_t qmsd_screen_drawbitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
    g_rgb_panel_handle->draw_bitmap(g_rgb_panel_handle, x, y, x + w, y + h, (void *)bitmap);
    return ESP_OK;
}

#if CONFIG_QMSD_RGB_PANEL_PHASE_TELEMETRY
static void qmsd_screen_log_rgb_config(const qmsd_lcd_rgb_panel_config_t *panel_config)
{
    if (!panel_config) {
        return;
    }

    size_t bits_per_pixel = panel_config->bits_per_pixel
                                ? panel_config->bits_per_pixel
                                : panel_config->data_width;
    size_t bytes_per_pixel = bits_per_pixel / 8;
    size_t frame_px = panel_config->timings.h_res * panel_config->timings.v_res;
    size_t bounce_bytes = panel_config->bounce_buffer_size_px * bytes_per_pixel;
    size_t expected_eof_since = panel_config->bounce_buffer_size_px
                                    ? frame_px / panel_config->bounce_buffer_size_px
                                    : 0;
    const char *fb_location = panel_config->flags.fb_in_psram ? "psram" : "internal";

    ESP_LOGI(TAG,
             "display-config board=ZX3D95CE01S_TR_4848 "
             "timing_20mhz=%d pclk_hz=%lu h_res=%lu v_res=%lu hsync_pw=%lu "
             "hsync_bp=%lu hsync_fp=%lu vsync_pw=%lu vsync_bp=%lu vsync_fp=%lu "
             "pclk_active_neg=%u pclk_idle_high=%u data_width=%zu bpp=%zu "
             "fb_count=%u fb_location=%s double_fb=%u direct_mode=%d full_refresh=%d "
             "avoid_te=%u bounce_lines=%d bounce_px=%zu bounce_bytes=%zu expected_eof_since=%zu "
             "restart_in_vsync=%d restart_on_phase_desync=%d phase_telemetry=%d",
             RGB_CFG_SCREEN_20_MHZ,
             (unsigned long)panel_config->timings.pclk_hz,
             (unsigned long)panel_config->timings.h_res,
             (unsigned long)panel_config->timings.v_res,
             (unsigned long)panel_config->timings.hsync_pulse_width,
             (unsigned long)panel_config->timings.hsync_back_porch,
             (unsigned long)panel_config->timings.hsync_front_porch,
             (unsigned long)panel_config->timings.vsync_pulse_width,
             (unsigned long)panel_config->timings.vsync_back_porch,
             (unsigned long)panel_config->timings.vsync_front_porch,
             (unsigned)panel_config->timings.flags.pclk_active_neg,
             (unsigned)panel_config->timings.flags.pclk_idle_high,
             panel_config->data_width,
             bits_per_pixel,
             panel_config->flags.double_fb ? 2U : 1U,
             fb_location,
             (unsigned)panel_config->flags.double_fb,
             RGB_CFG_GUI_DIRECT_MODE,
             RGB_CFG_GUI_FULL_REFRESH,
             (unsigned)panel_config->flags.avoid_te,
             CONFIG_QMSD_RGB_BOUNCE_BUFFER_LINES,
             panel_config->bounce_buffer_size_px,
             bounce_bytes,
             expected_eof_since,
             RGB_CFG_RESTART_IN_VSYNC,
             RGB_CFG_RESTART_ON_PHASE_DESYNC,
             CONFIG_QMSD_RGB_PANEL_PHASE_TELEMETRY);
}
#endif
