/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <stdarg.h>
#include <sys/cdefs.h>
#include <inttypes.h>
#include <sys/param.h>
#include <string.h>
#include "esp_lcd_types.h"
#include "soc/soc_caps.h"
#include "qmsd_utils.h"

#if SOC_LCD_RGB_SUPPORTED

#ifndef QMSD_GUI_RENDER_TELEMETRY_ENABLED
#define QMSD_GUI_RENDER_TELEMETRY_ENABLED 0
#endif

#if CONFIG_LCD_ENABLE_DEBUG_LOG
// The local log level must be defined before including esp_log.h
// Set the maximum log level for this source file
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_pm.h"
#include "esp_lcd_panel_interface.h"
#include "qmsd_lcd_panel_rgb.h"
#include "esp_lcd_panel_ops.h"
#include "esp_rom_gpio.h"
#include "esp_private/esp_clk.h"
#include "hal/dma_types.h"
#include "hal/gpio_hal.h"
#include "esp_private/gdma.h"
#include "driver/gpio.h"
#if ESP_IDF_VERSION_MAJOR >= 5
#include "esp_private/periph_ctrl.h"
#if CONFIG_SPIRAM
#include "esp_psram.h"
#endif
#else
#include "driver/periph_ctrl.h"
#if CONFIG_SPIRAM
#include "spiram.h"
#endif
#endif
#include "esp_lcd_common.h"
#include "esp_timer.h"
#if ESP_IDF_VERSION_MAJOR >= 6
#include "hal/lcd_periph.h"
#else
#include "soc/lcd_periph.h"
#endif
#include "hal/lcd_hal.h"
#include "hal/lcd_ll.h"
#include "hal/gdma_ll.h"
#include "rom/cache.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 2)
#include "esp_private/esp_clk_tree_common.h"
#include "esp_private/gdma_link.h"
#include "esp_private/gdma.h"
#include "esp_private/esp_dma_utils.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/gpio.h"
#if ESP_IDF_VERSION_MAJOR < 6
#define lcd_periph_signals lcd_periph_rgb_signals
#endif
#endif

#if ESP_IDF_VERSION_MAJOR >= 6
#define QMSD_LCD_PERIPH(panel_id) soc_lcd_rgb_signals[(panel_id)]
#else
#define QMSD_LCD_PERIPH(panel_id) lcd_periph_signals.panels[(panel_id)]
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
#define lcd_ll_set_data_width(x, y) lcd_ll_set_dma_read_stride(x, y)
#if defined(SOC_GDMA_TRIG_PERIPH_LCD0_BUS) && (SOC_GDMA_TRIG_PERIPH_LCD0_BUS == SOC_GDMA_BUS_AHB)
#if ESP_IDF_VERSION_MAJOR >= 6
#define LCD_GDMA_NEW_CHANNEL(config, ret_chan) gdma_new_ahb_channel((config), (ret_chan), NULL)
#else
#define LCD_GDMA_NEW_CHANNEL(config, ret_chan) gdma_new_ahb_channel((config), (ret_chan))
#endif
#define LCD_GDMA_DESCRIPTOR_ALIGN 4
#elif defined(SOC_GDMA_TRIG_PERIPH_LCD0_BUS) && (SOC_GDMA_TRIG_PERIPH_LCD0_BUS == SOC_GDMA_BUS_AXI)
#if ESP_IDF_VERSION_MAJOR >= 6
#define LCD_GDMA_NEW_CHANNEL(config, ret_chan) gdma_new_axi_channel((config), (ret_chan), NULL)
#else
#define LCD_GDMA_NEW_CHANNEL(config, ret_chan) gdma_new_axi_channel((config), (ret_chan))
#endif
#define LCD_GDMA_DESCRIPTOR_ALIGN 8
#endif
#endif

#ifndef LCD_LL_EVENT_RGB
#define LCD_LL_EVENT_RGB LCD_LL_EVENT_VSYNC_END
#endif

#if ESP_IDF_VERSION_MAJOR >= 6
#define QMSD_GPIO_FUNC_SEL(gpio_num) gpio_reset_pin((gpio_num_t)(gpio_num))
#else
#define QMSD_GPIO_FUNC_SEL(gpio_num) gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[(gpio_num)], PIN_FUNC_GPIO)
#endif

#if CONFIG_LCD_RGB_ISR_IRAM_SAFE
#define LCD_RGB_INTR_ALLOC_FLAGS     (ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_INTRDISABLED)
#else
#define LCD_RGB_INTR_ALLOC_FLAGS     ESP_INTR_FLAG_INTRDISABLED
#endif
#define RGB_LCD_PANEL_MAX_FB_NUM         3 // maximum supported frame buffer number
#define LCD_RGB_PHASE_BASELINE_SAMPLES   16
#define LCD_RGB_PHASE_BASELINE_POSITIONS 4

static const char *TAG = "lcd_panel.rgb";

#if !CONFIG_LCD_RGB_ISR_IRAM_SAFE
void qmsd_gui_record_vsync(int64_t timestamp_us) __attribute__((weak));
#endif

typedef struct esp_rgb_panel_t esp_rgb_panel_t;

typedef struct {
    uint32_t frame_px;
    uint32_t chunk_px;
    uint32_t vsync_total;
    uint32_t eof_total;
    uint32_t frame_wrap_total;
    uint32_t start_total;
    uint32_t restart_total;
    uint32_t eof_since_vsync;
    uint32_t wraps_since_vsync;
    uint32_t last_pos_px;
    uint32_t last_delta_px;
    uint32_t max_delta_px;
    uint32_t last_eofs_since_vsync;
    uint32_t min_eofs_since_vsync;
    uint32_t max_eofs_since_vsync;
    uint32_t last_wraps_since_vsync;
    uint32_t anomalies;
    uint32_t eof_anomalies;
    uint32_t baseline_resets;
    uint32_t baseline_overflow;
    uint32_t baseline_eof_min;
    uint32_t baseline_eof_max;
    uint32_t swap_wait_calls;
    uint32_t swap_wait_total_us;
    uint32_t swap_wait_max_us;
    uint32_t swap_wait_vsync_cross;
    uint32_t swap_wait_wrap_cross;
    uint32_t swap_wait_timeout;
    uint32_t baseline_pos_px[LCD_RGB_PHASE_BASELINE_POSITIONS];
    uint8_t baseline_pos_count;
    uint8_t baseline_samples;
    bool baseline_valid;
} lcd_rgb_phase_monitor_t;

static esp_err_t rgb_panel_del(esp_lcd_panel_t *panel);
static esp_err_t rgb_panel_reset(esp_lcd_panel_t *panel);
static esp_err_t rgb_panel_init(esp_lcd_panel_t *panel);
static esp_err_t rgb_panel_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t rgb_panel_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t rgb_panel_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t rgb_panel_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t rgb_panel_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
#if ESP_IDF_VERSION_MAJOR >= 5
static esp_err_t rgb_panel_disp_on_off(esp_lcd_panel_t *panel, bool off);
#endif
static uint32_t qmsd_lcd_hal_cal_pclk_freq(lcd_hal_context_t *hal, uint32_t src_freq_hz, uint32_t expect_pclk_freq_hz, int lcd_clk_flags);
static esp_err_t lcd_rgb_panel_select_clock_src(esp_rgb_panel_t *panel, user_rgb_clock_source_t clk_src);
static esp_err_t lcd_rgb_panel_create_trans_link(esp_rgb_panel_t *panel);
static esp_err_t lcd_rgb_panel_configure_gpio(esp_rgb_panel_t *panel, const qmsd_lcd_rgb_panel_config_t *panel_config);
static void lcd_rgb_panel_start_transmission(esp_rgb_panel_t *rgb_panel);
static void lcd_default_isr_handler(void *args);
static void lcd_rgb_panel_phase_init(esp_rgb_panel_t *panel);
static void lcd_rgb_panel_phase_record_eof(esp_rgb_panel_t *panel, int before_pos_px, int after_pos_px);
static void lcd_rgb_panel_phase_record_vsync(esp_rgb_panel_t *panel);
static void lcd_rgb_panel_phase_record_start(esp_rgb_panel_t *panel);
static void lcd_rgb_panel_phase_record_swap_wait(esp_rgb_panel_t *panel,
                                                 int64_t start_us,
                                                 uint32_t start_vsync,
                                                 uint32_t start_wrap,
                                                 BaseType_t result);
#if CONFIG_LCD_RGB_RESTART_IN_VSYNC
static void lcd_rgb_panel_phase_record_restart(esp_rgb_panel_t *panel);
#endif

struct esp_rgb_panel_t {
    esp_lcd_panel_t base;  // Base class of generic lcd panel
    int panel_id;          // LCD panel ID
    lcd_hal_context_t hal; // Hal layer object
    size_t data_width;     // Number of data lines
    size_t bits_per_pixel; // Color depth, in bpp
    size_t sram_trans_align;  // Alignment for framebuffer that allocated in SRAM
    size_t psram_trans_align; // Alignment for framebuffer that allocated in PSRAM
    int disp_gpio_num;     // Display control GPIO, which is used to perform action like "disp_off"
    intr_handle_t intr;    // LCD peripheral interrupt handle
    esp_pm_lock_handle_t pm_lock; // Power management lock
    size_t num_dma_nodes;  // Number of DMA descriptors that used to carry the frame buffer
    uint8_t *fbs[2];       // Frame buffers
    uint8_t cur_fb_index;  // Current frame buffer index (0 or 1)
    uint8_t cur_fb_index_hope;
    size_t fb_size;        // Size of frame buffer
    int data_gpio_nums[SOC_LCD_RGB_DATA_WIDTH]; // GPIOs used for data lines, we keep these GPIOs for action like "invert_color"
    uint32_t src_clk_hz;   // Peripheral source clock resolution
    qmsd_lcd_rgb_timing_t timings;   // RGB timing parameters (e.g. pclk, sync pulse, porch width)
    size_t bb_size;                 // If not-zero, the driver uses two bounce buffers allocated from internal memory
    int bounce_pos_px;              // Position in whatever source material is used for the bounce buffer, in pixels
    uint8_t *bounce_buffer[2];      // Pointer to the bounce buffers
    lcd_rgb_phase_monitor_t phase;
    gdma_channel_handle_t dma_chan; // DMA channel handle
    qmsd_lcd_rgb_panel_vsync_cb_t on_vsync; // VSYNC event callback
    qmsd_lcd_rgb_panel_bounce_buf_fill_cb_t on_bounce_empty; // callback used to fill a bounce buffer rather than copying from the frame buffer
    void *user_ctx;                 // Reserved user's data of callback functions
    int x_gap;                      // Extra gap in x coordinate, it's used when calculate the flush window
    int y_gap;                      // Extra gap in y coordinate, it's used when calculate the flush window
    SemaphoreHandle_t flush_ready;
    SemaphoreHandle_t swap_ready;
    portMUX_TYPE spinlock;          // to protect panel specific resource from concurrent access (e.g. between task and ISR)
    struct {
        uint32_t disp_en_level: 1;       // The level which can turn on the screen by `disp_gpio_num`
        uint32_t stream_mode: 1;         // If set, the LCD transfers data continuously, otherwise, it stops refreshing the LCD when transaction done
        uint32_t no_fb: 1;               // No frame buffer allocated in the driver
        uint32_t fb_in_psram: 1;         // Whether the frame buffer is in PSRAM
        uint32_t need_update_pclk: 1;    // Whether to update the PCLK before start a new transaction
        uint32_t bb_invalidate_cache: 1; // Whether to do cache invalidation in bounce buffer mode
        uint32_t avoid_te: 1;
    } flags;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 2)
    uint8_t bb_eof_count;
    gdma_link_list_handle_t dma_bb_link; // DMA link list for bounce buffer
    gdma_link_list_handle_t dma_fb_links[RGB_LCD_PANEL_MAX_FB_NUM]; // DMA link lists for multiple frame buffers
#if CONFIG_LCD_RGB_RESTART_IN_VSYNC
    gdma_link_list_handle_t dma_restart_link; // DMA link list used to restart the transfer
#endif
#else
    dma_descriptor_t *dma_links[2];    // fbs[0] <-> dma_links[0], fbs[1] <-> dma_links[1]
    dma_descriptor_t dma_restart_node; // DMA descriptor used to restart the transfer
    dma_descriptor_t dma_nodes[];      // DMA descriptors pool
#endif
};

#if QMSD_GUI_RENDER_TELEMETRY_ENABLED
static esp_rgb_panel_t *s_phase_monitor_panel = NULL;

static IRAM_ATTR uint32_t lcd_rgb_panel_phase_distance_px(uint32_t a, uint32_t b, uint32_t frame_px)
{
    uint32_t delta = a > b ? a - b : b - a;
    if (frame_px > 0 && delta > frame_px / 2) {
        delta = frame_px - delta;
    }
    return delta;
}

static IRAM_ATTR uint32_t lcd_rgb_panel_phase_tolerance_px(const esp_rgb_panel_t *panel)
{
    uint32_t line_px = panel && panel->timings.h_res ? panel->timings.h_res : 480;
    return line_px * 2U;
}

static IRAM_ATTR bool lcd_rgb_panel_phase_matches_baseline(const esp_rgb_panel_t *panel,
                                                           uint32_t pos_px,
                                                           uint32_t *out_delta_px)
{
    const lcd_rgb_phase_monitor_t *phase = &panel->phase;
    uint32_t best_delta = phase->frame_px;
    uint32_t tolerance_px = lcd_rgb_panel_phase_tolerance_px(panel);

    for (uint8_t i = 0; i < phase->baseline_pos_count; i++) {
        uint32_t delta = lcd_rgb_panel_phase_distance_px(pos_px,
                                                         phase->baseline_pos_px[i],
                                                         phase->frame_px);
        if (delta < best_delta) {
            best_delta = delta;
        }
        if (delta <= tolerance_px) {
            if (out_delta_px) {
                *out_delta_px = delta;
            }
            return true;
        }
    }

    if (out_delta_px) {
        *out_delta_px = best_delta;
    }
    return false;
}

static IRAM_ATTR void lcd_rgb_panel_phase_add_baseline_pos(esp_rgb_panel_t *panel, uint32_t pos_px)
{
    lcd_rgb_phase_monitor_t *phase = &panel->phase;
    uint32_t delta = 0;
    if (phase->baseline_pos_count > 0 &&
        lcd_rgb_panel_phase_matches_baseline(panel, pos_px, &delta)) {
        return;
    }

    if (phase->baseline_pos_count < LCD_RGB_PHASE_BASELINE_POSITIONS) {
        phase->baseline_pos_px[phase->baseline_pos_count++] = pos_px;
    } else {
        phase->baseline_overflow++;
    }
}

static void lcd_rgb_panel_phase_init(esp_rgb_panel_t *panel)
{
    if (!panel) {
        return;
    }

    memset(&panel->phase, 0, sizeof(panel->phase));
    if (!panel->bb_size || !panel->bits_per_pixel) {
        return;
    }

    int bytes_per_pixel = panel->bits_per_pixel / 8;
    if (bytes_per_pixel <= 0) {
        return;
    }

    panel->phase.frame_px = (uint32_t)(panel->fb_size / (size_t)bytes_per_pixel);
    panel->phase.chunk_px = (uint32_t)(panel->bb_size / (size_t)bytes_per_pixel);
}

static IRAM_ATTR void lcd_rgb_panel_phase_record_eof(esp_rgb_panel_t *panel,
                                                     int before_pos_px,
                                                     int after_pos_px)
{
    if (!panel || !panel->phase.frame_px || !panel->phase.chunk_px) {
        return;
    }

    panel->phase.eof_total++;
    panel->phase.eof_since_vsync++;
    if (after_pos_px < before_pos_px || after_pos_px == 0) {
        panel->phase.frame_wrap_total++;
        panel->phase.wraps_since_vsync++;
    }
}

static IRAM_ATTR void lcd_rgb_panel_phase_record_start(esp_rgb_panel_t *panel)
{
    if (!panel || !panel->phase.frame_px || !panel->phase.chunk_px) {
        return;
    }

    panel->phase.start_total++;
}

static void lcd_rgb_panel_phase_record_swap_wait(esp_rgb_panel_t *panel,
                                                 int64_t start_us,
                                                 uint32_t start_vsync,
                                                 uint32_t start_wrap,
                                                 BaseType_t result)
{
    if (!panel || !panel->phase.frame_px || start_us <= 0) {
        return;
    }

    int64_t now_us = esp_timer_get_time();
    if (now_us < start_us) {
        now_us = start_us;
    }
    uint32_t elapsed_us = (uint32_t)(now_us - start_us);

    lcd_rgb_phase_monitor_t *phase = &panel->phase;
    phase->swap_wait_calls++;
    phase->swap_wait_total_us += elapsed_us;
    if (elapsed_us > phase->swap_wait_max_us) {
        phase->swap_wait_max_us = elapsed_us;
    }
    if (phase->vsync_total != start_vsync) {
        phase->swap_wait_vsync_cross++;
    }
    if (phase->frame_wrap_total != start_wrap) {
        phase->swap_wait_wrap_cross++;
    }
    if (result != pdTRUE) {
        phase->swap_wait_timeout++;
    }
}

#if CONFIG_LCD_RGB_RESTART_IN_VSYNC
static IRAM_ATTR void lcd_rgb_panel_phase_record_restart(esp_rgb_panel_t *panel)
{
    if (!panel || !panel->phase.frame_px || !panel->phase.chunk_px) {
        return;
    }

    panel->phase.restart_total++;
    panel->phase.baseline_resets++;
}
#endif

static IRAM_ATTR void lcd_rgb_panel_phase_record_vsync(esp_rgb_panel_t *panel)
{
    if (!panel || !panel->phase.frame_px || !panel->phase.chunk_px) {
        return;
    }

    lcd_rgb_phase_monitor_t *phase = &panel->phase;
    uint32_t pos_px = panel->bounce_pos_px >= 0 ? (uint32_t)panel->bounce_pos_px : 0;
    uint32_t eofs_since_vsync = phase->eof_since_vsync;
    uint32_t wraps_since_vsync = phase->wraps_since_vsync;

    phase->eof_since_vsync = 0;
    phase->wraps_since_vsync = 0;
    phase->vsync_total++;
    phase->last_pos_px = pos_px;
    phase->last_eofs_since_vsync = eofs_since_vsync;
    phase->last_wraps_since_vsync = wraps_since_vsync;
    if (phase->min_eofs_since_vsync == 0 || eofs_since_vsync < phase->min_eofs_since_vsync) {
        phase->min_eofs_since_vsync = eofs_since_vsync;
    }
    if (eofs_since_vsync > phase->max_eofs_since_vsync) {
        phase->max_eofs_since_vsync = eofs_since_vsync;
    }

    uint32_t delta_px = 0;
    if (!phase->baseline_valid) {
        lcd_rgb_panel_phase_add_baseline_pos(panel, pos_px);
        if (phase->baseline_eof_min == 0 || eofs_since_vsync < phase->baseline_eof_min) {
            phase->baseline_eof_min = eofs_since_vsync;
        }
        if (eofs_since_vsync > phase->baseline_eof_max) {
            phase->baseline_eof_max = eofs_since_vsync;
        }
        phase->baseline_samples++;
        phase->last_delta_px = 0;
        if (phase->baseline_samples >= LCD_RGB_PHASE_BASELINE_SAMPLES) {
            phase->baseline_valid = true;
        }
        return;
    }

    if (!lcd_rgb_panel_phase_matches_baseline(panel, pos_px, &delta_px)) {
        phase->anomalies++;
    }
    if (eofs_since_vsync < phase->baseline_eof_min || eofs_since_vsync > phase->baseline_eof_max) {
        phase->eof_anomalies++;
    }
    phase->last_delta_px = delta_px;
    if (delta_px > phase->max_delta_px) {
        phase->max_delta_px = delta_px;
    }
}

void qmsd_lcd_rgb_panel_phase_stats_log(void)
{
    esp_rgb_panel_t *panel = s_phase_monitor_panel;
    if (!panel || !panel->bb_size || !panel->phase.frame_px || !panel->phase.chunk_px) {
        return;
    }

    lcd_rgb_phase_monitor_t *phase = &panel->phase;
    static uint32_t last_vsync_total;
    static uint32_t last_eof_total;
    static uint32_t last_anomalies;
    static uint32_t last_eof_anomalies;
    static uint32_t last_frame_wrap_total;
    static uint32_t last_start_total;
    static uint32_t last_restart_total;
    static uint32_t last_swap_wait_calls;
    static uint32_t last_swap_wait_total_us;
    static uint32_t last_swap_wait_vsync_cross;
    static uint32_t last_swap_wait_wrap_cross;
    static uint32_t last_swap_wait_timeout;

    uint32_t vsync_total = phase->vsync_total;
    uint32_t eof_total = phase->eof_total;
    uint32_t anomalies = phase->anomalies;
    uint32_t eof_anomalies = phase->eof_anomalies;
    uint32_t frame_wrap_total = phase->frame_wrap_total;
    uint32_t start_total = phase->start_total;
    uint32_t restart_total = phase->restart_total;
    uint32_t swap_wait_calls = phase->swap_wait_calls;
    uint32_t swap_wait_total_us = phase->swap_wait_total_us;
    uint32_t swap_wait_vsync_cross = phase->swap_wait_vsync_cross;
    uint32_t swap_wait_wrap_cross = phase->swap_wait_wrap_cross;
    uint32_t swap_wait_timeout = phase->swap_wait_timeout;
    uint32_t phase_vsync = vsync_total - last_vsync_total;
    uint32_t phase_eof = eof_total - last_eof_total;
    uint32_t phase_desync = anomalies - last_anomalies;
    uint32_t phase_eof_desync = eof_anomalies - last_eof_anomalies;
    uint32_t phase_wraps = frame_wrap_total - last_frame_wrap_total;
    uint32_t phase_starts = start_total - last_start_total;
    uint32_t phase_restarts = restart_total - last_restart_total;
    uint32_t phase_swap_wait_calls = swap_wait_calls - last_swap_wait_calls;
    uint32_t phase_swap_wait_total_us = swap_wait_total_us - last_swap_wait_total_us;
    uint32_t phase_swap_wait_vsync_cross = swap_wait_vsync_cross - last_swap_wait_vsync_cross;
    uint32_t phase_swap_wait_wrap_cross = swap_wait_wrap_cross - last_swap_wait_wrap_cross;
    uint32_t phase_swap_wait_timeout = swap_wait_timeout - last_swap_wait_timeout;
    uint32_t phase_swap_wait_avg_us = phase_swap_wait_calls
                                          ? phase_swap_wait_total_us / phase_swap_wait_calls
                                          : 0;

    last_vsync_total = vsync_total;
    last_eof_total = eof_total;
    last_anomalies = anomalies;
    last_eof_anomalies = eof_anomalies;
    last_frame_wrap_total = frame_wrap_total;
    last_start_total = start_total;
    last_restart_total = restart_total;
    last_swap_wait_calls = swap_wait_calls;
    last_swap_wait_total_us = swap_wait_total_us;
    last_swap_wait_vsync_cross = swap_wait_vsync_cross;
    last_swap_wait_wrap_cross = swap_wait_wrap_cross;
    last_swap_wait_timeout = swap_wait_timeout;

    uint32_t base0 = phase->baseline_pos_count > 0 ? phase->baseline_pos_px[0] : 0;
    uint32_t base1 = phase->baseline_pos_count > 1 ? phase->baseline_pos_px[1] : 0;
    uint32_t base2 = phase->baseline_pos_count > 2 ? phase->baseline_pos_px[2] : 0;
    uint32_t base3 = phase->baseline_pos_count > 3 ? phase->baseline_pos_px[3] : 0;

    ESP_LOGI(TAG,
             "phase vsync=%lu phase_vsync=%lu eof=%lu phase_eof=%lu pos_px=%lu base_count=%u base_px=%lu,%lu,%lu,%lu base_valid=%u base_eof=%lu..%lu desync=%lu phase_desync=%lu eof_desync=%lu phase_eof_desync=%lu delta_px=%lu max_delta_px=%lu eof_since=%lu eof_range=%lu..%lu wraps=%lu phase_wraps=%lu last_wraps=%lu starts=%lu phase_starts=%lu restarts=%lu phase_restarts=%lu swap=%lu swap_avg_us=%lu swap_max_us=%lu swap_vsync=%lu swap_wrap=%lu swap_timeout=%lu chunk_px=%lu frame_px=%lu fb=%u hope=%u overflow=%lu resets=%lu",
             (unsigned long)vsync_total,
             (unsigned long)phase_vsync,
             (unsigned long)eof_total,
             (unsigned long)phase_eof,
             (unsigned long)phase->last_pos_px,
             (unsigned)phase->baseline_pos_count,
             (unsigned long)base0,
             (unsigned long)base1,
             (unsigned long)base2,
             (unsigned long)base3,
             phase->baseline_valid ? 1U : 0U,
             (unsigned long)phase->baseline_eof_min,
             (unsigned long)phase->baseline_eof_max,
             (unsigned long)anomalies,
             (unsigned long)phase_desync,
             (unsigned long)eof_anomalies,
             (unsigned long)phase_eof_desync,
             (unsigned long)phase->last_delta_px,
             (unsigned long)phase->max_delta_px,
             (unsigned long)phase->last_eofs_since_vsync,
             (unsigned long)phase->min_eofs_since_vsync,
             (unsigned long)phase->max_eofs_since_vsync,
             (unsigned long)frame_wrap_total,
             (unsigned long)phase_wraps,
             (unsigned long)phase->last_wraps_since_vsync,
             (unsigned long)start_total,
             (unsigned long)phase_starts,
             (unsigned long)restart_total,
             (unsigned long)phase_restarts,
             (unsigned long)phase_swap_wait_calls,
             (unsigned long)phase_swap_wait_avg_us,
             (unsigned long)phase->swap_wait_max_us,
             (unsigned long)phase_swap_wait_vsync_cross,
             (unsigned long)phase_swap_wait_wrap_cross,
             (unsigned long)phase_swap_wait_timeout,
             (unsigned long)phase->chunk_px,
             (unsigned long)phase->frame_px,
             (unsigned)panel->cur_fb_index,
             (unsigned)panel->cur_fb_index_hope,
             (unsigned long)phase->baseline_overflow,
             (unsigned long)phase->baseline_resets);

    if (phase_desync > 0 || phase_eof_desync > 0 ||
        phase_restarts > 0 || phase_swap_wait_timeout > 0) {
        ESP_LOGW(TAG,
                 "phase-risk desync=%lu eof_desync=%lu restarts=%lu swap_timeout=%lu pos_px=%lu base_px=%lu,%lu,%lu,%lu delta_px=%lu eof_since=%lu eof_baseline=%lu..%lu wraps=%lu swap_vsync=%lu swap_wrap=%lu",
                 (unsigned long)phase_desync,
                 (unsigned long)phase_eof_desync,
                 (unsigned long)phase_restarts,
                 (unsigned long)phase_swap_wait_timeout,
                 (unsigned long)phase->last_pos_px,
                 (unsigned long)base0,
                 (unsigned long)base1,
                 (unsigned long)base2,
                 (unsigned long)base3,
                 (unsigned long)phase->last_delta_px,
                 (unsigned long)phase->last_eofs_since_vsync,
                 (unsigned long)phase->baseline_eof_min,
                 (unsigned long)phase->baseline_eof_max,
                 (unsigned long)phase->last_wraps_since_vsync,
                 (unsigned long)phase_swap_wait_vsync_cross,
                 (unsigned long)phase_swap_wait_wrap_cross);
    }
}
#else
static inline void lcd_rgb_panel_phase_init(esp_rgb_panel_t *panel)
{
    (void)panel;
}

static inline IRAM_ATTR void lcd_rgb_panel_phase_record_eof(esp_rgb_panel_t *panel,
                                                            int before_pos_px,
                                                            int after_pos_px)
{
    (void)panel;
    (void)before_pos_px;
    (void)after_pos_px;
}

static inline IRAM_ATTR void lcd_rgb_panel_phase_record_start(esp_rgb_panel_t *panel)
{
    (void)panel;
}

static inline void lcd_rgb_panel_phase_record_swap_wait(esp_rgb_panel_t *panel,
                                                        int64_t start_us,
                                                        uint32_t start_vsync,
                                                        uint32_t start_wrap,
                                                        BaseType_t result)
{
    (void)panel;
    (void)start_us;
    (void)start_vsync;
    (void)start_wrap;
    (void)result;
}

#if CONFIG_LCD_RGB_RESTART_IN_VSYNC
static inline IRAM_ATTR void lcd_rgb_panel_phase_record_restart(esp_rgb_panel_t *panel)
{
    (void)panel;
}
#endif

static inline IRAM_ATTR void lcd_rgb_panel_phase_record_vsync(esp_rgb_panel_t *panel)
{
    (void)panel;
}

void qmsd_lcd_rgb_panel_phase_stats_log(void)
{
}
#endif

static esp_err_t lcd_rgb_panel_alloc_frame_buffers(const qmsd_lcd_rgb_panel_config_t *rgb_panel_config, esp_rgb_panel_t *rgb_panel)
{
    bool fb_in_psram = false;
    size_t psram_trans_align = rgb_panel_config->psram_trans_align ? rgb_panel_config->psram_trans_align : 64;
    size_t sram_trans_align = rgb_panel_config->sram_trans_align ? rgb_panel_config->sram_trans_align : 4;
    rgb_panel->psram_trans_align = psram_trans_align;
    rgb_panel->sram_trans_align = sram_trans_align;

    // alloc frame buffer
    if (!rgb_panel_config->flags.no_fb) {
        // fb_in_psram is only an option, if there's no PSRAM on board, we fallback to alloc from SRAM
        if (rgb_panel_config->flags.fb_in_psram) {
            fb_in_psram = true;
        }
        for (int i = 0; i < (rgb_panel_config->flags.double_fb ? 2 : 1); i++) {
            if (fb_in_psram) {
                // the low level malloc function will help check the validation of alignment
                rgb_panel->fbs[i] = heap_caps_aligned_calloc(psram_trans_align, 1, rgb_panel->fb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            } else {
                rgb_panel->fbs[i] = heap_caps_aligned_calloc(sram_trans_align, 1, rgb_panel->fb_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
            }
            ESP_RETURN_ON_FALSE(rgb_panel->fbs[i], ESP_ERR_NO_MEM, TAG, "no mem for frame buffer");
        }
    }

    // alloc bounce buffer
    if (rgb_panel->bb_size) {
        for (int i = 0; i < 2; i++) {
            // bounce buffer must come from SRAM
            rgb_panel->bounce_buffer[i] = heap_caps_aligned_calloc(sram_trans_align, 1, rgb_panel->bb_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
            ESP_RETURN_ON_FALSE(rgb_panel->bounce_buffer[i], ESP_ERR_NO_MEM, TAG, "no mem for bounce buffer");
        }
    }
    rgb_panel->cur_fb_index = 0;
    rgb_panel->flags.fb_in_psram = fb_in_psram;

    return ESP_OK;
}

static esp_err_t lcd_rgb_panel_destory(esp_rgb_panel_t *rgb_panel)
{
    lcd_ll_enable_clock(rgb_panel->hal.dev, false);
    if (rgb_panel->panel_id >= 0) {
#if ESP_IDF_VERSION_MAJOR >= 6
        PERIPH_RCC_RELEASE_ATOMIC(QMSD_LCD_PERIPH(rgb_panel->panel_id).module, ref_count) {
            if (ref_count == 0) {
                lcd_ll_enable_bus_clock(rgb_panel->panel_id, false);
            }
        }
#else
        periph_module_disable(QMSD_LCD_PERIPH(rgb_panel->panel_id).module);
#endif
        lcd_com_remove_device(LCD_COM_DEVICE_TYPE_RGB, rgb_panel->panel_id);
    }
    if (rgb_panel->fbs[0]) {
        free(rgb_panel->fbs[0]);
    }
    if (rgb_panel->fbs[1]) {
        free(rgb_panel->fbs[1]);
    }
    if (rgb_panel->bounce_buffer[0]) {
        free(rgb_panel->bounce_buffer[0]);
    }
    if (rgb_panel->bounce_buffer[1]) {
        free(rgb_panel->bounce_buffer[1]);
    }
    if (rgb_panel->dma_chan) {
        gdma_disconnect(rgb_panel->dma_chan);
        gdma_del_channel(rgb_panel->dma_chan);
    }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 2)
    for (size_t i = 0; i < RGB_LCD_PANEL_MAX_FB_NUM; i++) {
        if (rgb_panel->dma_fb_links[i]) {
            gdma_del_link_list(rgb_panel->dma_fb_links[i]);
        }
    }
    if (rgb_panel->dma_bb_link) {
        gdma_del_link_list(rgb_panel->dma_bb_link);
    }
#if CONFIG_LCD_RGB_RESTART_IN_VSYNC
    if (rgb_panel->dma_restart_link) {
        gdma_del_link_list(rgb_panel->dma_restart_link);
    }
#endif
#endif
    if (rgb_panel->intr) {
        esp_intr_free(rgb_panel->intr);
    }
    if (rgb_panel->pm_lock) {
        esp_pm_lock_release(rgb_panel->pm_lock);
        esp_pm_lock_delete(rgb_panel->pm_lock);
    }
    free(rgb_panel);
    return ESP_OK;
}

esp_err_t qmsd_lcd_new_rgb_panel(const qmsd_lcd_rgb_panel_config_t *rgb_panel_config, esp_lcd_panel_handle_t *ret_panel)
{
#if CONFIG_LCD_ENABLE_DEBUG_LOG
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
#endif
    esp_err_t ret = ESP_OK;
    esp_rgb_panel_t *rgb_panel = NULL;
    ESP_GOTO_ON_FALSE(rgb_panel_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid parameter");
    ESP_GOTO_ON_FALSE(rgb_panel_config->data_width == 16 || rgb_panel_config->data_width == 8,
                      ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported data width %d", rgb_panel_config->data_width);
    ESP_GOTO_ON_FALSE(!(rgb_panel_config->flags.double_fb && rgb_panel_config->flags.no_fb),
                      ESP_ERR_INVALID_ARG, err, TAG, "invalid frame buffer number");
    ESP_GOTO_ON_FALSE(!(rgb_panel_config->flags.no_fb && rgb_panel_config->bounce_buffer_size_px == 0),
                      ESP_ERR_INVALID_ARG, err, TAG, "must set bounce buffer if there's no frame buffer");
    ESP_GOTO_ON_FALSE(!(rgb_panel_config->flags.refresh_on_demand && rgb_panel_config->bounce_buffer_size_px),
                      ESP_ERR_INVALID_ARG, err, TAG, "refresh on demand is not supported under bounce buffer mode");
#if CONFIG_LCD_RGB_ISR_IRAM_SAFE
    ESP_GOTO_ON_FALSE(rgb_panel_config->bounce_buffer_size_px == 0,
                      ESP_ERR_INVALID_ARG, err, TAG, "bounce buffer mode is not IRAM Safe");
#endif

    // bpp defaults to the number of data lines, but for serial RGB interface, they're not equal
    size_t bits_per_pixel = rgb_panel_config->data_width;
    if (rgb_panel_config->bits_per_pixel) { // override bpp if it's set
        bits_per_pixel = rgb_panel_config->bits_per_pixel;
    }
    // calculate buffer size
    size_t fb_size = rgb_panel_config->timings.h_res * rgb_panel_config->timings.v_res * bits_per_pixel / 8;
    size_t bb_size = rgb_panel_config->bounce_buffer_size_px * bits_per_pixel / 8;
    if (bb_size) {
        // we want the bounce can always end in the second buffer
        ESP_GOTO_ON_FALSE(fb_size % (2 * bb_size) == 0, ESP_ERR_INVALID_ARG, err, TAG,
                          "fb size must be even multiple of bounce buffer size, fb_size: %d, bb_size: %d", fb_size, bb_size);
    }

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 2)
    rgb_panel = heap_caps_calloc(1, sizeof(esp_rgb_panel_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    rgb_panel->bb_eof_count = 0;
    ESP_GOTO_ON_FALSE(rgb_panel, ESP_ERR_NO_MEM, err, TAG, "no mem for rgb panel");
#else
    // calculate the number of DMA descriptors
    size_t num_dma_nodes = 0;
    if (bb_size) {
        // in bounce buffer mode, DMA is used to convey the bounce buffer, not the frame buffer.
        // frame buffer is copied to bounce buffer by CPU
        num_dma_nodes = (bb_size + DMA_DESCRIPTOR_BUFFER_MAX_SIZE - 1) / DMA_DESCRIPTOR_BUFFER_MAX_SIZE;
    } else {
        // Not bounce buffer mode, DMA descriptors need to fit the entire frame buffer
        num_dma_nodes = (fb_size + DMA_DESCRIPTOR_BUFFER_MAX_SIZE - 1) / DMA_DESCRIPTOR_BUFFER_MAX_SIZE;
    }

    // DMA descriptors must be placed in internal SRAM (requested by DMA)
    // multiply 2 because of double frame buffer mode (two frame buffer) and bounce buffer mode (two bounce buffer)
    rgb_panel = heap_caps_calloc(1, sizeof(esp_rgb_panel_t) + num_dma_nodes * sizeof(dma_descriptor_t) * 2,
                                 MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    ESP_GOTO_ON_FALSE(rgb_panel, ESP_ERR_NO_MEM, err, TAG, "no mem for rgb panel");
    rgb_panel->num_dma_nodes = num_dma_nodes;
#endif
    rgb_panel->fb_size = fb_size;
    rgb_panel->bb_size = bb_size;
    rgb_panel->bits_per_pixel = bits_per_pixel;
    rgb_panel->panel_id = -1;
    // register to platform
    int panel_id = lcd_com_register_device(LCD_COM_DEVICE_TYPE_RGB, rgb_panel);
    ESP_GOTO_ON_FALSE(panel_id >= 0, ESP_ERR_NOT_FOUND, err, TAG, "no free rgb panel slot");
    rgb_panel->panel_id = panel_id;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
    // enable APB to access LCD registers
    PERIPH_RCC_ACQUIRE_ATOMIC(QMSD_LCD_PERIPH(panel_id).module, ref_count) {
        if (ref_count == 0) {
            lcd_ll_enable_bus_clock(panel_id, true);
            lcd_ll_reset_register(panel_id);
        }
    }
#else
    periph_module_enable(QMSD_LCD_PERIPH(panel_id).module);
    periph_module_reset(QMSD_LCD_PERIPH(panel_id).module);
#endif
    // allocate frame buffers + bounce buffers
    ESP_GOTO_ON_ERROR(lcd_rgb_panel_alloc_frame_buffers(rgb_panel_config, rgb_panel), err, TAG, "alloc frame buffers failed");

    // initialize HAL layer, so we can call LL APIs later
    lcd_hal_init(&rgb_panel->hal, panel_id);
    // enable clock gating
    lcd_ll_enable_clock(rgb_panel->hal.dev, true);
    // set clock source
    ret = lcd_rgb_panel_select_clock_src(rgb_panel, rgb_panel_config->clk_src);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "set source clock failed");
    // install interrupt service, (LCD peripheral shares the interrupt source with Camera by different mask)
    int isr_flags = LCD_RGB_INTR_ALLOC_FLAGS | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_LOWMED;
    ret = esp_intr_alloc_intrstatus(QMSD_LCD_PERIPH(panel_id).irq_id, isr_flags,
                                    (uint32_t)lcd_ll_get_interrupt_status_reg(rgb_panel->hal.dev),
                                    LCD_LL_EVENT_RGB, lcd_default_isr_handler, rgb_panel, &rgb_panel->intr);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "install interrupt failed");
    PERIPH_RCC_ATOMIC() {
        lcd_ll_enable_interrupt(rgb_panel->hal.dev, LCD_LL_EVENT_RGB, false); // disable all interrupts
    }
    lcd_ll_clear_interrupt_status(rgb_panel->hal.dev, UINT32_MAX); // clear pending interrupt

    // install DMA service
    rgb_panel->flags.stream_mode = !rgb_panel_config->flags.refresh_on_demand;
    ret = lcd_rgb_panel_create_trans_link(rgb_panel);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "install DMA failed");
    // configure GPIO
    ret = lcd_rgb_panel_configure_gpio(rgb_panel, rgb_panel_config);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "configure GPIO failed");
    // fill other rgb panel runtime parameters
    memcpy(rgb_panel->data_gpio_nums, rgb_panel_config->data_gpio_nums, sizeof(rgb_panel->data_gpio_nums));
    rgb_panel->timings = rgb_panel_config->timings;
    rgb_panel->data_width = rgb_panel_config->data_width;
    rgb_panel->disp_gpio_num = rgb_panel_config->disp_gpio_num;
    rgb_panel->flags.disp_en_level = !rgb_panel_config->flags.disp_active_low;
    rgb_panel->flags.no_fb = rgb_panel_config->flags.no_fb;
    rgb_panel->flags.bb_invalidate_cache = rgb_panel_config->flags.bb_invalidate_cache;
    rgb_panel->flags.avoid_te = rgb_panel_config->flags.avoid_te;
    rgb_panel->spinlock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    lcd_rgb_panel_phase_init(rgb_panel);
#if QMSD_GUI_RENDER_TELEMETRY_ENABLED
    if (rgb_panel->bb_size) {
        s_phase_monitor_panel = rgb_panel;
    }
#endif
    // fill function table
    rgb_panel->base.del = rgb_panel_del;
    rgb_panel->base.reset = rgb_panel_reset;
    rgb_panel->base.init = rgb_panel_init;
    rgb_panel->base.draw_bitmap = rgb_panel_draw_bitmap;
#if ESP_IDF_VERSION_MAJOR >= 5
    rgb_panel->base.disp_on_off = rgb_panel_disp_on_off;
#endif
    rgb_panel->base.invert_color = rgb_panel_invert_color;
    rgb_panel->base.mirror = rgb_panel_mirror;
    rgb_panel->base.swap_xy = rgb_panel_swap_xy;
    rgb_panel->base.set_gap = rgb_panel_set_gap;
    rgb_panel->flush_ready = xSemaphoreCreateBinary();
    rgb_panel->swap_ready = xSemaphoreCreateBinary();
    // return base class
    *ret_panel = &(rgb_panel->base);
    ESP_LOGD(TAG, "new rgb panel(%d) @%p, fb0 @%p, fb1 @%p, fb_size=%zu, bb0 @%p, bb1 @%p, bb_size=%zu",
             rgb_panel->panel_id, rgb_panel, rgb_panel->fbs[0], rgb_panel->fbs[1], rgb_panel->fb_size,
             rgb_panel->bounce_buffer[0], rgb_panel->bounce_buffer[1], rgb_panel->bb_size);
    return ESP_OK;

err:
    if (rgb_panel) {
        lcd_rgb_panel_destory(rgb_panel);
    }
    return ret;
}

esp_err_t qmsd_lcd_rgb_panel_register_event_callbacks(esp_lcd_panel_handle_t panel, const qmsd_lcd_rgb_panel_event_callbacks_t *callbacks, void *user_ctx)
{
    ESP_RETURN_ON_FALSE(panel && callbacks, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    esp_rgb_panel_t *rgb_panel = __containerof(panel, esp_rgb_panel_t, base);
#if CONFIG_LCD_RGB_ISR_IRAM_SAFE
    if (callbacks->on_vsync) {
        ESP_RETURN_ON_FALSE(esp_ptr_in_iram(callbacks->on_vsync), ESP_ERR_INVALID_ARG, TAG, "on_vsync callback not in IRAM");
    }
    if (callbacks->on_bounce_empty) {
        ESP_RETURN_ON_FALSE(esp_ptr_in_iram(callbacks->on_bounce_empty), ESP_ERR_INVALID_ARG, TAG, "on_bounce_empty callback not in IRAM");
    }
    if (user_ctx) {
        ESP_RETURN_ON_FALSE(esp_ptr_internal(user_ctx), ESP_ERR_INVALID_ARG, TAG, "user context not in internal RAM");
    }
#endif // CONFIG_LCD_RGB_ISR_IRAM_SAFE
    rgb_panel->on_vsync = callbacks->on_vsync;
    rgb_panel->on_bounce_empty = callbacks->on_bounce_empty;
    rgb_panel->user_ctx = user_ctx;
    return ESP_OK;
}

esp_err_t qmsd_lcd_rgb_panel_set_pclk(esp_lcd_panel_handle_t panel, uint32_t freq_hz)
{
    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    esp_rgb_panel_t *rgb_panel = __containerof(panel, esp_rgb_panel_t, base);
    // the pclk frequency will be updated in the `LCD_LL_EVENT_VSYNC_END` event handler
    portENTER_CRITICAL(&rgb_panel->spinlock);
    rgb_panel->flags.need_update_pclk = true;
    rgb_panel->timings.pclk_hz = freq_hz;
    portEXIT_CRITICAL(&rgb_panel->spinlock);
    return ESP_OK;
}

esp_err_t qmsd_lcd_rgb_panel_get_frame_buffer(esp_lcd_panel_handle_t panel, uint32_t fb_num, void **fb0, ...)
{
    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(fb_num && fb_num <= 2, ESP_ERR_INVALID_ARG, TAG, "invalid frame buffer number");
    esp_rgb_panel_t *rgb_panel = __containerof(panel, esp_rgb_panel_t, base);
    void **fb_itor = fb0;
    va_list args;
    va_start(args, fb0);
    for (int i = 0; i < fb_num; i++) {
        if (fb_itor) {
            *fb_itor = rgb_panel->fbs[i];
            fb_itor = va_arg(args, void **);
        }
    }
    va_end(args);
    return ESP_OK;
}

esp_err_t qmsd_lcd_rgb_panel_get_idle_frame_buffer(esp_lcd_panel_handle_t panel,  uint8_t **buffer) {
    esp_rgb_panel_t *rgb_panel = __containerof(panel, esp_rgb_panel_t, base);
    if (rgb_panel->cur_fb_index == 0) {
        *buffer = rgb_panel->fbs[1];
    } else {
        *buffer = rgb_panel->fbs[0];
    }
    return ESP_OK;
}

esp_err_t qmsd_lcd_rgb_panel_get_running_frame_buffer(esp_lcd_panel_handle_t panel,  uint8_t **buffer) {
    esp_rgb_panel_t *rgb_panel = __containerof(panel, esp_rgb_panel_t, base);
    if (rgb_panel->cur_fb_index == 0) {
        *buffer = rgb_panel->fbs[0];
    } else {
        *buffer = rgb_panel->fbs[1];
    }
    return ESP_OK;
}

esp_err_t qmsd_lcd_rgb_panel_refresh(esp_lcd_panel_handle_t panel)
{
    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    esp_rgb_panel_t *rgb_panel = __containerof(panel, esp_rgb_panel_t, base);
    ESP_RETURN_ON_FALSE(!rgb_panel->flags.stream_mode, ESP_ERR_INVALID_STATE, TAG, "refresh on demand is not enabled");
    lcd_rgb_panel_start_transmission(rgb_panel);
    return ESP_OK;
}

static esp_err_t rgb_panel_del(esp_lcd_panel_t *panel)
{
    esp_rgb_panel_t *rgb_panel = __containerof(panel, esp_rgb_panel_t, base);
    int panel_id = rgb_panel->panel_id;
    ESP_RETURN_ON_ERROR(lcd_rgb_panel_destory(rgb_panel), TAG, "destroy rgb panel(%d) failed", panel_id);
    ESP_LOGD(TAG, "del rgb panel(%d)", panel_id);
    return ESP_OK;
}

static esp_err_t rgb_panel_reset(esp_lcd_panel_t *panel)
{
    esp_rgb_panel_t *rgb_panel = __containerof(panel, esp_rgb_panel_t, base);
    lcd_ll_fifo_reset(rgb_panel->hal.dev);
    lcd_ll_reset(rgb_panel->hal.dev);
    return ESP_OK;
}

static esp_err_t rgb_panel_init(esp_lcd_panel_t *panel)
{
    esp_err_t ret = ESP_OK;
    esp_rgb_panel_t *rgb_panel = __containerof(panel, esp_rgb_panel_t, base);

    // set pixel clock frequency
    rgb_panel->timings.pclk_hz = qmsd_lcd_hal_cal_pclk_freq(&rgb_panel->hal, rgb_panel->src_clk_hz, rgb_panel->timings.pclk_hz, 0);
    // pixel clock phase and polarity
    lcd_ll_set_clock_idle_level(rgb_panel->hal.dev, rgb_panel->timings.flags.pclk_idle_high);
    lcd_ll_set_pixel_clock_edge(rgb_panel->hal.dev, rgb_panel->timings.flags.pclk_active_neg);
    // enable RGB mode and set data width
    lcd_ll_enable_rgb_mode(rgb_panel->hal.dev, true);
    lcd_ll_set_data_width(rgb_panel->hal.dev, rgb_panel->data_width);
    lcd_ll_set_phase_cycles(rgb_panel->hal.dev, 0, 0, 1); // enable data phase only
    // number of data cycles is controlled by DMA buffer size
    lcd_ll_enable_output_always_on(rgb_panel->hal.dev, true);
    // configure HSYNC, VSYNC, DE signal idle state level
    lcd_ll_set_idle_level(rgb_panel->hal.dev, !rgb_panel->timings.flags.hsync_idle_low,
                          !rgb_panel->timings.flags.vsync_idle_low, rgb_panel->timings.flags.de_idle_high);
    // configure blank region timing
    lcd_ll_set_blank_cycles(rgb_panel->hal.dev, 1, 1); // RGB panel always has a front and back blank (porch region)
    lcd_ll_set_horizontal_timing(rgb_panel->hal.dev, rgb_panel->timings.hsync_pulse_width,
                                 rgb_panel->timings.hsync_back_porch, rgb_panel->timings.h_res * rgb_panel->bits_per_pixel / rgb_panel->data_width,
                                 rgb_panel->timings.hsync_front_porch);
    lcd_ll_set_vertical_timing(rgb_panel->hal.dev, rgb_panel->timings.vsync_pulse_width,
                               rgb_panel->timings.vsync_back_porch, rgb_panel->timings.v_res,
                               rgb_panel->timings.vsync_front_porch);
    // output hsync even in porch region
    lcd_ll_enable_output_hsync_in_porch_region(rgb_panel->hal.dev, true);
    // generate the hsync at the very beginning of line
    lcd_ll_set_hsync_position(rgb_panel->hal.dev, 0);
    // send next frame automatically in stream mode
    lcd_ll_enable_auto_next_frame(rgb_panel->hal.dev, rgb_panel->flags.stream_mode);
    // trigger interrupt on the end of frame
    PERIPH_RCC_ATOMIC() {
        lcd_ll_enable_interrupt(rgb_panel->hal.dev, LCD_LL_EVENT_RGB, true);
    }
    // enable intr
    esp_intr_enable(rgb_panel->intr);
    // start transmission
    if (rgb_panel->flags.stream_mode) {
        lcd_rgb_panel_start_transmission(rgb_panel);
    }
    ESP_LOGD(TAG, "rgb panel(%d) start, pclk=%" PRIu32 "Hz", rgb_panel->panel_id, rgb_panel->timings.pclk_hz);
    return ret;
}

static esp_err_t rgb_panel_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    esp_rgb_panel_t *rgb_panel = __containerof(panel, esp_rgb_panel_t, base);
    ESP_RETURN_ON_FALSE(!rgb_panel->flags.no_fb, ESP_ERR_NOT_SUPPORTED, TAG, "no frame buffer installed");
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");
    bool do_copy = false;

    // avoid bounce mode te
    if (rgb_panel->bb_size) {
        if (color_data == rgb_panel->fbs[0]) {
            rgb_panel->cur_fb_index_hope = 0;
        } else if (color_data == rgb_panel->fbs[1]) {
            rgb_panel->cur_fb_index_hope = 1;
        } else {
            goto normal;
        }

        if (rgb_panel->flags.avoid_te) {
            xSemaphoreTake(rgb_panel->swap_ready, 0);
#if QMSD_GUI_RENDER_TELEMETRY_ENABLED
            uint32_t wait_start_vsync = rgb_panel->phase.vsync_total;
            uint32_t wait_start_wrap = rgb_panel->phase.frame_wrap_total;
            int64_t wait_start_us = esp_timer_get_time();
            BaseType_t wait_result = xSemaphoreTake(rgb_panel->swap_ready, portMAX_DELAY);
            lcd_rgb_panel_phase_record_swap_wait(rgb_panel,
                                                 wait_start_us,
                                                 wait_start_vsync,
                                                 wait_start_wrap,
                                                 wait_result);
#else
            xSemaphoreTake(rgb_panel->swap_ready, portMAX_DELAY);
#endif
        }

        return ESP_OK;
    }

normal:
    // check if we need to copy the draw buffer (pointed by the color_data) to the driver's frame buffer
    if (color_data == rgb_panel->fbs[0]) {
        rgb_panel->cur_fb_index = 0;
    } else if (color_data == rgb_panel->fbs[1]) {
        rgb_panel->cur_fb_index = 1;
    } else {
        // we do the copy only if the color_data is different from either frame buffer
        do_copy = true;
    }

    // adjust the flush window by adding extra gap
    x_start += rgb_panel->x_gap;
    y_start += rgb_panel->y_gap;
    x_end += rgb_panel->x_gap;
    y_end += rgb_panel->y_gap;
    // round the boundary
    x_start = MIN(x_start, rgb_panel->timings.h_res);
    x_end = MIN(x_end, rgb_panel->timings.h_res);
    y_start = MIN(y_start, rgb_panel->timings.v_res);
    y_end = MIN(y_end, rgb_panel->timings.v_res);

    int bytes_per_pixel = rgb_panel->bits_per_pixel / 8;
    int pixels_per_line = rgb_panel->timings.h_res;
    uint32_t bytes_per_line = bytes_per_pixel * pixels_per_line;
    uint8_t *fb = rgb_panel->fbs[rgb_panel->cur_fb_index];

    if (do_copy) {
        // copy the UI draw buffer into internal frame buffer
        const uint8_t *from = (const uint8_t *)color_data;
        uint32_t copy_bytes_per_line = (x_end - x_start) * bytes_per_pixel;
        uint8_t *to = fb + (y_start * pixels_per_line + x_start) * bytes_per_pixel;
        for (int y = y_start; y < y_end; y++) {
            memcpy(to, from, copy_bytes_per_line);
            to += bytes_per_line;
            from += copy_bytes_per_line;
        }
    }

    if (rgb_panel->flags.fb_in_psram && !rgb_panel->bb_size) {
        // CPU writes data to PSRAM through DCache, data in PSRAM might not get updated, so write back
        // Note that if we use a bounce buffer, the data gets read by the CPU as well so no need to write back
        uint32_t bytes_to_flush = (y_end - y_start) * bytes_per_line;
        Cache_WriteBack_Addr((uint32_t)(fb + y_start * bytes_per_line), bytes_to_flush);
    }

    if (!rgb_panel->bb_size) {
        if (rgb_panel->flags.stream_mode) {
            int64_t time_start = esp_timer_get_time();
            // the DMA will convey the new frame buffer next time
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 2)
            for (int i = 0; i < 2; i++) {
                // Note, because of DMA prefetch, there's possibility that the old frame buffer might be sent out again
                // it's hard to know the time when the new frame buffer starts
                gdma_link_concat(rgb_panel->dma_fb_links[i], -1, rgb_panel->dma_fb_links[rgb_panel->cur_fb_index], 0);
            }
#else
            rgb_panel->dma_nodes[rgb_panel->num_dma_nodes - 1].next = rgb_panel->dma_links[rgb_panel->cur_fb_index];
            rgb_panel->dma_nodes[rgb_panel->num_dma_nodes * 2 - 1].next = rgb_panel->dma_links[rgb_panel->cur_fb_index];
#endif

            if (rgb_panel->flags.avoid_te) {
                xSemaphoreTake(rgb_panel->flush_ready, 0);
                xSemaphoreTake(rgb_panel->flush_ready, portMAX_DELAY);
                if (esp_timer_get_time() - time_start < 1000) {
                    xSemaphoreTake(rgb_panel->flush_ready, portMAX_DELAY);
                }
            }
        }
    }
    return ESP_OK;
}

static esp_err_t rgb_panel_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    esp_rgb_panel_t *rgb_panel = __containerof(panel, esp_rgb_panel_t, base);
    int panel_id = rgb_panel->panel_id;
    // inverting the data line by GPIO matrix
    for (int i = 0; i < rgb_panel->data_width; i++) {
        esp_rom_gpio_connect_out_signal(rgb_panel->data_gpio_nums[i], QMSD_LCD_PERIPH(panel_id).data_sigs[i],
                                        invert_color_data, false);
    }
    return ESP_OK;
}

static esp_err_t rgb_panel_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t rgb_panel_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t rgb_panel_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    esp_rgb_panel_t *rgb_panel = __containerof(panel, esp_rgb_panel_t, base);
    rgb_panel->x_gap = x_gap;
    rgb_panel->y_gap = y_gap;
    return ESP_OK;
}

#if ESP_IDF_VERSION_MAJOR >= 5
static esp_err_t rgb_panel_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    esp_rgb_panel_t *rgb_panel = __containerof(panel, esp_rgb_panel_t, base);
    if (rgb_panel->disp_gpio_num < 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (!on_off) { // turn off screen
        gpio_set_level(rgb_panel->disp_gpio_num, !rgb_panel->flags.disp_en_level);
    } else { // turn on screen
        gpio_set_level(rgb_panel->disp_gpio_num, rgb_panel->flags.disp_en_level);
    }
    return ESP_OK;
}
#endif

static esp_err_t lcd_rgb_panel_configure_gpio(esp_rgb_panel_t *panel, const qmsd_lcd_rgb_panel_config_t *panel_config)
{
    int panel_id = panel->panel_id;
    // check validation of GPIO number
    bool valid_gpio = (panel_config->pclk_gpio_num >= 0);
    if (panel_config->de_gpio_num < 0) {
        // Hsync and Vsync are required in HV mode
        valid_gpio = valid_gpio && (panel_config->hsync_gpio_num >= 0) && (panel_config->vsync_gpio_num >= 0);
    }
    for (size_t i = 0; i < panel_config->data_width; i++) {
        valid_gpio = valid_gpio && (panel_config->data_gpio_nums[i] >= 0);
    }
    if (!valid_gpio) {
        return ESP_ERR_INVALID_ARG;
    }
    // connect peripheral signals via GPIO matrix
    for (size_t i = 0; i < panel_config->data_width; i++) {
        QMSD_GPIO_FUNC_SEL(panel_config->data_gpio_nums[i]);
        gpio_set_direction(panel_config->data_gpio_nums[i], GPIO_MODE_OUTPUT);
        esp_rom_gpio_pad_set_drv(panel_config->data_gpio_nums[i], 0);
        esp_rom_gpio_connect_out_signal(panel_config->data_gpio_nums[i],
                                        QMSD_LCD_PERIPH(panel_id).data_sigs[i], false, false);
    }
    if (panel_config->hsync_gpio_num >= 0) {
        QMSD_GPIO_FUNC_SEL(panel_config->hsync_gpio_num);
        gpio_set_direction(panel_config->hsync_gpio_num, GPIO_MODE_OUTPUT);
        esp_rom_gpio_pad_set_drv(panel_config->hsync_gpio_num, 0);
        esp_rom_gpio_connect_out_signal(panel_config->hsync_gpio_num,
                                        QMSD_LCD_PERIPH(panel_id).hsync_sig, false, false);
    }
    if (panel_config->vsync_gpio_num >= 0) {
        QMSD_GPIO_FUNC_SEL(panel_config->vsync_gpio_num);
        gpio_set_direction(panel_config->vsync_gpio_num, GPIO_MODE_OUTPUT);
        esp_rom_gpio_pad_set_drv(panel_config->vsync_gpio_num, 0);
        esp_rom_gpio_connect_out_signal(panel_config->vsync_gpio_num,
                                        QMSD_LCD_PERIPH(panel_id).vsync_sig, false, false);
    }
    QMSD_GPIO_FUNC_SEL(panel_config->pclk_gpio_num);
    gpio_set_direction(panel_config->pclk_gpio_num, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_set_drv(panel_config->pclk_gpio_num, 0);
    esp_rom_gpio_connect_out_signal(panel_config->pclk_gpio_num,
                                    QMSD_LCD_PERIPH(panel_id).pclk_sig, false, false);
    // DE signal might not be necessary for some RGB LCD
    if (panel_config->de_gpio_num >= 0) {
        QMSD_GPIO_FUNC_SEL(panel_config->de_gpio_num);
        gpio_set_direction(panel_config->de_gpio_num, GPIO_MODE_OUTPUT);
        esp_rom_gpio_pad_set_drv(panel_config->de_gpio_num, 0);
        esp_rom_gpio_connect_out_signal(panel_config->de_gpio_num,
                                        QMSD_LCD_PERIPH(panel_id).de_sig, false, false);
    }
    // disp enable GPIO is optional
    if (panel_config->disp_gpio_num >= 0) {
        QMSD_GPIO_FUNC_SEL(panel_config->disp_gpio_num);
        gpio_set_direction(panel_config->disp_gpio_num, GPIO_MODE_OUTPUT);
        esp_rom_gpio_pad_set_drv(panel_config->disp_gpio_num, 0);
        esp_rom_gpio_connect_out_signal(panel_config->disp_gpio_num, SIG_GPIO_OUT_IDX, false, false);
    }
    return ESP_OK;
}

static IRAM_ATTR bool lcd_rgb_panel_fill_bounce_buffer(esp_rgb_panel_t *panel, uint8_t *buffer)
{
    bool need_yield = false;
    int bytes_per_pixel = panel->bits_per_pixel / 8;
    if (panel->flags.no_fb) {
        if (panel->on_bounce_empty) {
            // We don't have a frame buffer here; we need to call a callback to refill the bounce buffer
            need_yield = panel->on_bounce_empty(&panel->base, buffer, panel->bounce_pos_px, panel->bb_size, panel->user_ctx);
        }
    } else {
        // We do have frame buffer; copy from there.
        // Note: if the cache is diabled, and accessing the PSRAM by DCACHE will crash.
        memcpy(buffer, &panel->fbs[panel->cur_fb_index][panel->bounce_pos_px * bytes_per_pixel], panel->bb_size);
        if (panel->flags.bb_invalidate_cache) {
            // We don't need the bytes we copied from the psram anymore
            // Make sure that if anything happened to have changed (because the line already was in cache) we write the data back.
            Cache_WriteBack_Addr((uint32_t)&panel->fbs[panel->cur_fb_index][panel->bounce_pos_px * bytes_per_pixel], panel->bb_size);
            // Invalidate the data.
            // Note: possible race: perhaps something on the other core can squeeze a write between this and the writeback,
            // in which case that data gets discarded.
            Cache_Invalidate_Addr((uint32_t)&panel->fbs[panel->cur_fb_index][panel->bounce_pos_px * bytes_per_pixel], panel->bb_size);
        }
    }
    panel->bounce_pos_px += panel->bb_size / bytes_per_pixel;
    // If the bounce pos is larger than the frame buffer size, wrap around so the next isr starts pre-loading the next frame.
    if (panel->bounce_pos_px >= panel->fb_size / bytes_per_pixel) {
        panel->bounce_pos_px = 0;
        panel->cur_fb_index = panel->cur_fb_index_hope;
        BaseType_t high_task_awoken = pdFALSE;
        xSemaphoreGiveFromISR(panel->swap_ready, &high_task_awoken);
        need_yield |= high_task_awoken;
    }
    if (!panel->flags.no_fb) {
        // Preload the next bit of buffer from psram
        Cache_Start_DCache_Preload((uint32_t)&panel->fbs[panel->cur_fb_index][panel->bounce_pos_px * bytes_per_pixel],
                                   panel->bb_size, 0);
    }
    return need_yield;
}

// This is called in bounce buffer mode, when one bounce buffer has been fully sent to the LCD peripheral.
static IRAM_ATTR bool lcd_rgb_panel_eof_handler(gdma_channel_handle_t dma_chan, gdma_event_data_t *event_data, void *user_data)
{
    esp_rgb_panel_t *panel = (esp_rgb_panel_t *)user_data;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 2)
    int bb = panel->bb_eof_count % 2;
    panel->bb_eof_count++;
#else
    dma_descriptor_t *desc = (dma_descriptor_t *)event_data->tx_eof_desc_addr;
    // Figure out which bounce buffer to write to.
    // Note: what we receive is the *last* descriptor of this bounce buffer.
    int bb = (desc == &panel->dma_nodes[panel->num_dma_nodes - 1]) ? 0 : 1;
#endif
    int before_pos_px = panel->bounce_pos_px;
    bool need_yield = lcd_rgb_panel_fill_bounce_buffer(panel, panel->bounce_buffer[bb]);
    lcd_rgb_panel_phase_record_eof(panel, before_pos_px, panel->bounce_pos_px);
    return need_yield;
}

// If we restart GDMA, many pixels already have been transferred to the LCD peripheral.
// Looks like that has 16 pixels of FIFO plus one holding register.
#define LCD_FIFO_PRESERVE_SIZE_PX (GDMA_LL_L2FIFO_BASE_SIZE + 1)

static esp_err_t lcd_rgb_panel_create_trans_link(esp_rgb_panel_t *panel)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 2)
#if CONFIG_LCD_RGB_RESTART_IN_VSYNC
    size_t bytes_per_pixel = panel->bits_per_pixel / 8;
    size_t restart_skip_bytes = LCD_FIFO_PRESERVE_SIZE_PX * bytes_per_pixel;
#endif
    if (panel->bb_size) {
        size_t buffer_alignment = panel->sram_trans_align;
        size_t num_dma_nodes_per_bounce_buffer = esp_dma_calculate_node_count(panel->bb_size, buffer_alignment, LCD_DMA_DESCRIPTOR_BUFFER_MAX_SIZE);
        gdma_link_list_config_t link_cfg = {
#if ESP_IDF_VERSION_MAJOR < 6
            .buffer_alignment = buffer_alignment,
#endif
            .item_alignment = LCD_GDMA_DESCRIPTOR_ALIGN,
            .num_items = num_dma_nodes_per_bounce_buffer * 2,
            .flags = {
                .check_owner = true,
            }
        };
        ESP_RETURN_ON_ERROR(gdma_new_link_list(&link_cfg, &panel->dma_bb_link), TAG, "create bounce buffer DMA link failed");
        // mount bounce buffers to the DMA link list
        gdma_buffer_mount_config_t mount_cfgs[2] = {0};
        for (int i = 0; i < 2; i++) {
            mount_cfgs[i].buffer = panel->bounce_buffer[i];
#if ESP_IDF_VERSION_MAJOR >= 6
            mount_cfgs[i].buffer_alignment = buffer_alignment;
#endif
            mount_cfgs[i].length = panel->bb_size;
            mount_cfgs[i].flags.mark_eof = true;  // we use the DMA EOF interrupt to copy the frame buffer (partially) to the bounce buffer
        }
        ESP_RETURN_ON_ERROR(gdma_link_mount_buffers(panel->dma_bb_link, 0, mount_cfgs, 2, NULL), TAG, "mount DMA bounce buffers failed");
#if CONFIG_LCD_RGB_RESTART_IN_VSYNC
        // Match the actual first mounted node; IDF 6 shortens node lengths for buffer alignment.
        size_t restart_length = gdma_link_get_length(panel->dma_bb_link, 0);
        ESP_RETURN_ON_FALSE(restart_skip_bytes < restart_length, ESP_ERR_INVALID_ARG, TAG, "restart skip is too large for bounce buffer");
        gdma_link_list_config_t restart_link_cfg = {
#if ESP_IDF_VERSION_MAJOR < 6
            .buffer_alignment = buffer_alignment,
#endif
            .item_alignment = LCD_GDMA_DESCRIPTOR_ALIGN,
            .num_items = 1,
            .flags = {
                .check_owner = true,
            },
        };
        ESP_RETURN_ON_ERROR(gdma_new_link_list(&restart_link_cfg, &panel->dma_restart_link), TAG, "create DMA restart link failed");
        gdma_buffer_mount_config_t restart_mount_cfg = {
            .buffer = panel->bounce_buffer[0] + restart_skip_bytes,
            .length = restart_length - restart_skip_bytes,
#if ESP_IDF_VERSION_MAJOR >= 6
            .buffer_alignment = buffer_alignment,
#endif
            .flags = {
                .bypass_buffer_align_check = true,
            },
        };
        ESP_RETURN_ON_ERROR(gdma_link_mount_buffers(panel->dma_restart_link, 0, &restart_mount_cfg, 1, NULL), TAG, "mount DMA restart buffer failed");
        gdma_link_concat(panel->dma_restart_link, 0, panel->dma_bb_link, 1);
#endif
    } else {
        size_t buffer_alignment = panel->flags.fb_in_psram ? panel->psram_trans_align : panel->sram_trans_align;
        size_t num_dma_nodes = esp_dma_calculate_node_count(panel->fb_size, buffer_alignment, LCD_DMA_DESCRIPTOR_BUFFER_MAX_SIZE);
        gdma_link_list_config_t link_cfg = {
#if ESP_IDF_VERSION_MAJOR < 6
            .buffer_alignment = buffer_alignment,
#endif
            .item_alignment = LCD_GDMA_DESCRIPTOR_ALIGN,
            .num_items = num_dma_nodes,
            .flags = {
                .check_owner = true,
            },
        };
        gdma_buffer_mount_config_t mount_cfg = {
            .length = panel->fb_size,
#if ESP_IDF_VERSION_MAJOR >= 6
            .buffer_alignment = buffer_alignment,
#endif
            .flags = {
                .mark_final = panel->flags.stream_mode ? false : true,
                .mark_eof = true,
            },
        };
        for (size_t i = 0; i < 2; i++) {
            ESP_RETURN_ON_ERROR(gdma_new_link_list(&link_cfg, &panel->dma_fb_links[i]), TAG, "create frame buffer DMA link failed");
            // mount bounce buffers to the DMA link list
            mount_cfg.buffer = panel->fbs[i];
            ESP_RETURN_ON_ERROR(gdma_link_mount_buffers(panel->dma_fb_links[i], 0, &mount_cfg, 1, NULL), TAG, "mount DMA frame buffer failed");
        }
#if CONFIG_LCD_RGB_RESTART_IN_VSYNC
        // Match the actual first mounted node; IDF 6 shortens node lengths for buffer alignment.
        size_t restart_length = gdma_link_get_length(panel->dma_fb_links[0], 0);
        ESP_RETURN_ON_FALSE(restart_skip_bytes < restart_length, ESP_ERR_INVALID_ARG, TAG, "restart skip is too large for frame buffer");
        gdma_link_list_config_t restart_link_cfg = {
#if ESP_IDF_VERSION_MAJOR < 6
            .buffer_alignment = buffer_alignment,
#endif
            .item_alignment = LCD_GDMA_DESCRIPTOR_ALIGN,
            .num_items = 1,
            .flags = {
                .check_owner = true,
            },
        };
        ESP_RETURN_ON_ERROR(gdma_new_link_list(&restart_link_cfg, &panel->dma_restart_link), TAG, "create DMA restart link failed");
        gdma_buffer_mount_config_t restart_mount_cfg = {
            .buffer = panel->fbs[0] + restart_skip_bytes,
            .length = restart_length - restart_skip_bytes,
#if ESP_IDF_VERSION_MAJOR >= 6
            .buffer_alignment = buffer_alignment,
#endif
            .flags = {
                .bypass_buffer_align_check = true,
            },
        };
        ESP_RETURN_ON_ERROR(gdma_link_mount_buffers(panel->dma_restart_link, 0, &restart_mount_cfg, 1, NULL), TAG, "mount DMA restart buffer failed");
        gdma_link_concat(panel->dma_restart_link, 0, panel->dma_fb_links[0], 1);
#endif
    }

#else
    panel->dma_links[0] = &panel->dma_nodes[0];
    panel->dma_links[1] = &panel->dma_nodes[panel->num_dma_nodes];
    // chain DMA descriptors
    for (int i = 0; i < panel->num_dma_nodes * 2; i++) {
        panel->dma_nodes[i].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_CPU;
        panel->dma_nodes[i].next = &panel->dma_nodes[i + 1];
    }

    if (panel->bb_size) {
        // loop end back to start
        panel->dma_nodes[panel->num_dma_nodes * 2 - 1].next = &panel->dma_nodes[0];
        // mount the bounce buffers to the DMA descriptors
        lcd_com_mount_dma_data(panel->dma_links[0], panel->bounce_buffer[0], panel->bb_size);
        lcd_com_mount_dma_data(panel->dma_links[1], panel->bounce_buffer[1], panel->bb_size);
    } else {
        if (panel->flags.stream_mode) {
            // circle DMA descriptors chain for each frame buffer
            panel->dma_nodes[panel->num_dma_nodes - 1].next = &panel->dma_nodes[0];
            panel->dma_nodes[panel->num_dma_nodes * 2 - 1].next = &panel->dma_nodes[panel->num_dma_nodes];
        } else {
            // one-off DMA descriptors chain
            panel->dma_nodes[panel->num_dma_nodes - 1].next = NULL;
            panel->dma_nodes[panel->num_dma_nodes * 2 - 1].next = NULL;
        }
        // mount the frame buffer to the DMA descriptors
        lcd_com_mount_dma_data(panel->dma_links[0], panel->fbs[0], panel->fb_size);
        if (panel->fbs[1]) {
            lcd_com_mount_dma_data(panel->dma_links[1], panel->fbs[1], panel->fb_size);
        }
    }
#endif
#if CONFIG_LCD_RGB_RESTART_IN_VSYNC && (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 3, 2))
    // On restart, the data sent to the LCD peripheral needs to start LCD_FIFO_PRESERVE_SIZE_PX pixels after the FB start
    // so we use a dedicated DMA node to restart the DMA transaction
    memcpy(&panel->dma_restart_node, &panel->dma_nodes[0], sizeof(panel->dma_restart_node));
    int restart_skip_bytes = LCD_FIFO_PRESERVE_SIZE_PX * (panel->bits_per_pixel / 8);
    uint8_t *p = (uint8_t *)panel->dma_restart_node.buffer;
    panel->dma_restart_node.buffer = &p[restart_skip_bytes];
    panel->dma_restart_node.dw0.length -= restart_skip_bytes;
    panel->dma_restart_node.dw0.size -= restart_skip_bytes;
#endif
    // alloc DMA channel and connect to LCD peripheral
#if ESP_IDF_VERSION_MAJOR >= 6
    gdma_channel_alloc_config_t dma_chan_config = {};
#else
    gdma_channel_alloc_config_t dma_chan_config = {
        .direction = GDMA_CHANNEL_DIRECTION_TX,
    };
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
    ESP_RETURN_ON_ERROR(LCD_GDMA_NEW_CHANNEL(&dma_chan_config, &panel->dma_chan), TAG, "alloc DMA channel failed");
    gdma_connect(panel->dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));

    // configure DMA transfer parameters
    gdma_transfer_config_t trans_cfg = {
        .max_data_burst_size = 64,
        .access_ext_mem = true, // frame buffer was allocated from external memory
    };
    ESP_RETURN_ON_ERROR(gdma_config_transfer(panel->dma_chan, &trans_cfg), TAG, "config DMA transfer failed");
#else
    // alloc DMA channel and connect to LCD peripheral
    ESP_RETURN_ON_ERROR(gdma_new_channel(&dma_chan_config, &panel->dma_chan), TAG, "alloc DMA channel failed");
    gdma_connect(panel->dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));
    gdma_transfer_ability_t ability = {
        .psram_trans_align = panel->psram_trans_align,
        .sram_trans_align = panel->sram_trans_align,
    };
    gdma_set_transfer_ability(panel->dma_chan, &ability);
#endif

    // we need to refill the bounce buffer in the DMA EOF interrupt, so only register the callback for bounce buffer mode
    if (panel->bb_size) {
        gdma_tx_event_callbacks_t cbs = {
            .on_trans_eof = lcd_rgb_panel_eof_handler,
        };
        gdma_register_tx_event_callbacks(panel->dma_chan, &cbs, panel);
    }

    return ESP_OK;
}

#if CONFIG_LCD_RGB_RESTART_IN_VSYNC
static IRAM_ATTR void lcd_rgb_panel_restart_transmission_in_isr(esp_rgb_panel_t *panel)
{
    lcd_rgb_panel_phase_record_restart(panel);
    int bytes_per_pixel = panel->bits_per_pixel / 8;
    int bb_size_px = bytes_per_pixel > 0 ? panel->bb_size / bytes_per_pixel : 0;
    if (panel->bb_size) {
        // Catch de-synced frame buffer and reset if needed.
        if (panel->bounce_pos_px > bb_size_px * 2) {
            panel->bounce_pos_px = 0;
        }
        // Pre-fill bounce buffer 0, if the EOF ISR didn't do that already
        if (panel->bounce_pos_px < bb_size_px) {
            lcd_rgb_panel_fill_bounce_buffer(panel, panel->bounce_buffer[0]);
        }
    }

    lcd_ll_fifo_reset(panel->hal.dev);
    gdma_reset(panel->dma_chan);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 2)
    gdma_start(panel->dma_chan, gdma_link_get_head_addr(panel->dma_restart_link));
#else
    // restart the DMA by a special DMA node
    gdma_start(panel->dma_chan, (intptr_t)&panel->dma_restart_node);
#endif

    if (panel->bb_size) {
        // Fill 2nd bounce buffer while 1st is being sent out, if needed.
        if (panel->bounce_pos_px < bb_size_px * 2) {
            lcd_rgb_panel_fill_bounce_buffer(panel, panel->bounce_buffer[1]);
        }
    }
}
#endif

static void lcd_rgb_panel_start_transmission(esp_rgb_panel_t *rgb_panel)
{
    lcd_rgb_panel_phase_record_start(rgb_panel);
    // reset FIFO of DMA and LCD, incase there remains old frame data
    gdma_reset(rgb_panel->dma_chan);
    lcd_ll_stop(rgb_panel->hal.dev);
    lcd_ll_fifo_reset(rgb_panel->hal.dev);

    // pre-fill bounce buffers if needed
    if (rgb_panel->bb_size) {
        rgb_panel->bounce_pos_px = 0;
        lcd_rgb_panel_fill_bounce_buffer(rgb_panel, rgb_panel->bounce_buffer[0]);
        lcd_rgb_panel_fill_bounce_buffer(rgb_panel, rgb_panel->bounce_buffer[1]);
    }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 2)
    if (rgb_panel->bb_size) {
        gdma_start(rgb_panel->dma_chan, gdma_link_get_head_addr(rgb_panel->dma_bb_link));
    }  else {
        gdma_start(rgb_panel->dma_chan, gdma_link_get_head_addr(rgb_panel->dma_fb_links[rgb_panel->cur_fb_index]));
    }
#else
    // the start of DMA should be prior to the start of LCD engine
    gdma_start(rgb_panel->dma_chan, (intptr_t)rgb_panel->dma_links[rgb_panel->cur_fb_index]);
#endif
    // delay 1us is sufficient for DMA to pass data to LCD FIFO
    // in fact, this is only needed when LCD pixel clock is set too high
    esp_rom_delay_us(1);
    // start LCD engine
    lcd_ll_start(rgb_panel->hal.dev);
}

IRAM_ATTR static void lcd_rgb_panel_try_update_pclk(esp_rgb_panel_t *rgb_panel)
{
    portENTER_CRITICAL_ISR(&rgb_panel->spinlock);
    if (unlikely(rgb_panel->flags.need_update_pclk)) {
        rgb_panel->flags.need_update_pclk = false;
        rgb_panel->timings.pclk_hz = qmsd_lcd_hal_cal_pclk_freq(&rgb_panel->hal, rgb_panel->src_clk_hz, rgb_panel->timings.pclk_hz, 0);
    }
    portEXIT_CRITICAL_ISR(&rgb_panel->spinlock);
}

IRAM_ATTR static void lcd_default_isr_handler(void *args)
{
    esp_rgb_panel_t *rgb_panel = (esp_rgb_panel_t *)args;
    bool need_yield = false;

    uint32_t intr_status = lcd_ll_get_interrupt_status(rgb_panel->hal.dev);
    lcd_ll_clear_interrupt_status(rgb_panel->hal.dev, intr_status);
    if (intr_status & LCD_LL_EVENT_VSYNC_END) {
        lcd_rgb_panel_phase_record_vsync(rgb_panel);
#if !CONFIG_LCD_RGB_ISR_IRAM_SAFE
        if (qmsd_gui_record_vsync) {
            qmsd_gui_record_vsync(esp_timer_get_time());
        }
#endif
        // call user registered callback
        if (rgb_panel->on_vsync) {
            if (rgb_panel->on_vsync(&rgb_panel->base, NULL, rgb_panel->user_ctx)) {
                need_yield = true;
            }
        }

        if (rgb_panel->flags.avoid_te) {
            BaseType_t high_task_awoken = pdFALSE;
            xSemaphoreGiveFromISR(rgb_panel->flush_ready, &high_task_awoken);
            need_yield |= high_task_awoken;
        }

        // check whether to update the PCLK frequency, it should be safe to update the PCLK frequency in the VSYNC interrupt
        lcd_rgb_panel_try_update_pclk(rgb_panel);

        if (rgb_panel->flags.stream_mode) {
#if CONFIG_LCD_RGB_RESTART_IN_VSYNC
            // reset the GDMA channel every VBlank to stop permanent desyncs from happening.
            // Note that this fix can lead to single-frame desyncs itself, as in: if this interrupt
            // is late enough, the display will shift as the LCD controller already read out the
            // first data bytes, and resetting DMA will re-send those. However, the single-frame
            // desync this leads to is preferable to the permanent desync that could otherwise
            // happen. It's also not super-likely as this interrupt has the entirety of the VBlank
            // time to reset DMA.
            lcd_rgb_panel_restart_transmission_in_isr(rgb_panel);
#endif
        }

    }

    if (need_yield) {
        portYIELD_FROM_ISR();
    }
}

#if ESP_IDF_VERSION_MAJOR >= 5
static esp_err_t lcd_rgb_panel_select_clock_src(esp_rgb_panel_t *panel, user_rgb_clock_source_t clk_src)
{
    esp_err_t ret = ESP_OK;
    switch (clk_src) {
    case USER_RGB_CLK_SRC_PLL240M:
        panel->src_clk_hz = 240000000;
        panel->hal.dev->lcd_clock.lcd_clk_sel = 2;
        break;
    case USER_RGB_CLK_SRC_PLL160M:
        panel->src_clk_hz = 160000000;
        panel->hal.dev->lcd_clock.lcd_clk_sel = 3;
        break;
    case USER_RGB_CLK_SRC_XTAL:
        panel->src_clk_hz = esp_clk_xtal_freq();
        panel->hal.dev->lcd_clock.lcd_clk_sel = 1;
        break;
    default:
        ESP_RETURN_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, TAG, "unsupported clock source: %d", clk_src);
        break;
    }

    if (clk_src == USER_RGB_CLK_SRC_PLL160M || clk_src == USER_RGB_CLK_SRC_PLL240M) {
#if CONFIG_PM_ENABLE
        ret = esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "rgb_panel", &panel->pm_lock);
        ESP_RETURN_ON_ERROR(ret, TAG, "create ESP_PM_APB_FREQ_MAX lock failed");
        // hold the lock during the whole lifecycle of RGB panel
        esp_pm_lock_acquire(panel->pm_lock);
        ESP_LOGD(TAG, "installed ESP_PM_APB_FREQ_MAX lock and hold the lock during the whole panel lifecycle");
#endif
    }
    return ret;
}

static uint32_t qmsd_lcd_hal_cal_pclk_freq(lcd_hal_context_t *hal, uint32_t src_freq_hz, uint32_t expect_pclk_freq_hz, int lcd_clk_flags)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
    hal_utils_clk_div_t lcd_clk_div = {};
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 1)
    uint32_t pclk_hz = lcd_hal_cal_pclk_freq(hal, src_freq_hz, expect_pclk_freq_hz, &lcd_clk_div);
#else
    uint32_t pclk_hz = lcd_hal_cal_pclk_freq(hal, src_freq_hz, expect_pclk_freq_hz, 0, &lcd_clk_div);
#endif
    LCD_CLOCK_SRC_ATOMIC() {
        lcd_ll_set_group_clock_coeff(hal->dev, lcd_clk_div.integer, lcd_clk_div.denominator, lcd_clk_div.numerator);
    }
#else
    uint32_t pclk_hz = lcd_hal_cal_pclk_freq(hal, src_freq_hz, expect_pclk_freq_hz, lcd_clk_flags);
#endif
    return pclk_hz;
}

#else
static uint32_t qmsd_lcd_hal_cal_pclk_freq(lcd_hal_context_t *hal, uint32_t src_freq_hz, uint32_t expect_pclk_freq_hz, int lcd_clk_flags)
{
    uint32_t pclk_prescale = src_freq_hz / expect_pclk_freq_hz;
    lcd_ll_set_pixel_clock_prescale(hal->dev, pclk_prescale);
    return src_freq_hz / pclk_prescale;
}

static inline void user_rgb_ll_set_group_clock_src(lcd_cam_dev_t *dev, user_rgb_clock_source_t src, int div_num, int div_a, int div_b) {
    // lcd_clk = module_clock_src / (div_num + div_b / div_a)
    HAL_ASSERT(div_num >= 2);
    HAL_FORCE_MODIFY_U32_REG_FIELD(dev->lcd_clock, lcd_clkm_div_num, div_num);
    dev->lcd_clock.lcd_clkm_div_a = div_a;
    dev->lcd_clock.lcd_clkm_div_b = div_b;
    switch (src) {
        case USER_RGB_CLK_SRC_PLL160M:
            dev->lcd_clock.lcd_clk_sel = 3;
            break;
        case USER_RGB_CLK_SRC_PLL240M:
            dev->lcd_clock.lcd_clk_sel = 2;
            break;
        case USER_RGB_CLK_SRC_XTAL:
            dev->lcd_clock.lcd_clk_sel = 1;
            break;
        default:
            HAL_ASSERT(false && "unsupported clock source");
            break;
    }
}

static esp_err_t lcd_rgb_panel_select_clock_src(esp_rgb_panel_t *panel, user_rgb_clock_source_t clk_src)
{
    esp_err_t ret = ESP_OK;
    user_rgb_ll_set_group_clock_src(panel->hal.dev, clk_src, LCD_PERIPH_CLOCK_PRE_SCALE, 1, 0);
    switch (clk_src) {
        case USER_RGB_CLK_SRC_PLL160M:
            panel->src_clk_hz = 160000000 / LCD_PERIPH_CLOCK_PRE_SCALE;
#if CONFIG_PM_ENABLE
            ret = esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "rgb_panel", &panel->pm_lock);
            ESP_RETURN_ON_ERROR(ret, TAG, "create ESP_PM_APB_FREQ_MAX lock failed");
            // hold the lock during the whole lifecycle of RGB panel
            esp_pm_lock_acquire(panel->pm_lock);
            ESP_LOGD(TAG, "installed ESP_PM_APB_FREQ_MAX lock and hold the lock during the whole panel lifecycle");
#endif
            break;
        case USER_RGB_CLK_SRC_PLL240M:
            panel->src_clk_hz = 240000000 / LCD_PERIPH_CLOCK_PRE_SCALE;
            break;
        case USER_RGB_CLK_SRC_XTAL:
            panel->src_clk_hz = rtc_clk_xtal_freq_get() * 1000000;
            break;
        default:
            ESP_RETURN_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, TAG,  "unsupported clock source: %d", clk_src);
            break;
    }
    return ret;
}
#endif

#endif // SOC_LCD_RGB_SUPPORTED
