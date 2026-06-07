#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "qmsd_gui.h"
#include "qmsd_utils.h"
#include "lvgl.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

// Forward declare to avoid adding a build-time dependency edge from qmsd_gui -> qmsd_touch.
uint32_t touch_samples_waiting(void);

#define QMSD_GUI_INDEV_READ_PERIOD_MS 10
#define QMSD_GUI_STATS_PERIOD_US 1000000
#define QMSD_GUI_SLOW_HANDLER_US 16000
#define QMSD_GUI_VERY_SLOW_HANDLER_US 25000
#define QMSD_GUI_INV_PRESSURE_AREAS 32
#define QMSD_GUI_DIRECT_RENDER_VSYNC_WAIT_TIMEOUT_MS 25
#define QMSD_GUI_DIRECT_RENDER_PHASE_WINDOW_US 2500

static const char *TAG = "QMSD_GUI";

static qmsd_gui_config_t* g_lvgl_config;
static QueueHandle_t g_image_queue;
static SemaphoreHandle_t g_gui_semaphore = NULL;
static TaskHandle_t s_gui_update_task_handle = NULL;
static volatile bool s_render_stats_active = false;
static lv_area_t s_direct_flush_area;
static bool s_direct_flush_area_valid = false;
static uint32_t s_handler_flush_event_total_us = 0;

typedef struct {
    uint32_t count;
    uint32_t interval_count;
    uint64_t interval_total_us;
    uint32_t interval_min_us;
    uint32_t interval_max_us;
    uint32_t jitter_count;
    uint64_t jitter_total_us;
    uint32_t jitter_max_us;
    int64_t last_us;
    uint32_t last_interval_us;
} qmsd_gui_vsync_stats_t;

static portMUX_TYPE s_vsync_stats_mux = portMUX_INITIALIZER_UNLOCKED;
static qmsd_gui_vsync_stats_t s_vsync_stats;

typedef struct {
    uint32_t handler_calls;
    uint64_t handler_total_us;
    uint32_t handler_max_us;
    uint32_t handler_slow_16ms;
    uint32_t handler_slow_25ms;
    uint32_t lock_wait_calls;
    uint64_t lock_wait_total_us;
    uint32_t lock_wait_max_us;
    uint32_t lvgl_handler_calls;
    uint64_t lvgl_handler_total_us;
    uint32_t lvgl_handler_max_us;
    uint32_t lvgl_work_calls;
    uint64_t lvgl_work_total_us;
    uint32_t lvgl_work_max_us;
    uint32_t lvgl_work_slow_16ms;
    uint32_t lvgl_work_slow_25ms;
    uint32_t flush_calls;
    uint64_t flush_total_us;
    uint64_t flush_pixels;
    uint32_t flush_max_us;
    uint32_t flush_max_pixels;
    lv_area_t flush_max_area;
    bool flush_max_area_valid;
    uint32_t flush_full_calls;
    uint32_t refr_cycles;
    uint64_t refr_total_us;
    uint32_t refr_max_us;
    int64_t refr_start_us;
    uint32_t render_cycles;
    uint64_t render_total_us;
    uint32_t render_max_us;
    int64_t render_start_us;
    int64_t render_start_vsync_us;
    uint32_t render_since_vsync_count;
    uint64_t render_since_vsync_total_us;
    uint32_t render_since_vsync_min_us;
    uint32_t render_since_vsync_max_us;
    uint32_t render_cross_vsync;
    uint32_t render_work_cycles;
    uint64_t render_work_total_us;
    uint32_t render_work_max_us;
    int64_t render_work_start_us;
    int64_t render_work_start_vsync_us;
    uint32_t render_work_cross_vsync;
    uint32_t flush_waits;
    uint64_t flush_wait_total_us;
    uint32_t flush_wait_max_us;
    int64_t flush_wait_start_us;
    int64_t flush_event_start_us;
    int64_t flush_event_start_vsync_us;
    uint32_t flush_since_vsync_count;
    uint64_t flush_since_vsync_total_us;
    uint32_t flush_since_vsync_min_us;
    uint32_t flush_since_vsync_max_us;
    uint32_t flush_cross_vsync;
    uint32_t inv_calls;
    uint64_t inv_pixels;
    uint32_t inv_max_pixels;
    lv_area_t inv_max_area;
    uint32_t inv_full_calls;
    uint32_t inv_pending_calls;
    uint64_t inv_pending_pixels;
    lv_area_t inv_pending_area;
    bool inv_pending_area_valid;
    uint32_t inv_pending_peak_calls;
    uint32_t inv_pending_peak_kpx;
    lv_area_t inv_pending_peak_area;
    bool inv_pending_peak_area_valid;
    uint32_t inv_pressure_windows;
    bool inv_pressure_seen;
    int64_t window_start_us;
} qmsd_gui_render_stats_t;

static qmsd_gui_render_stats_t s_render_stats;

static bool qmsd_gui_render_stats_enabled(void)
{
    bool enabled = esp_log_level_get(TAG) >= ESP_LOG_INFO;
    s_render_stats_active = enabled;
    return enabled;
}

static void qmsd_gui_notify_vsync_from_isr(void)
{
    TaskHandle_t update_task = s_gui_update_task_handle;
    if (!update_task) {
        return;
    }

    BaseType_t high_task_awoken = pdFALSE;
    vTaskNotifyGiveFromISR(update_task, &high_task_awoken);
    if (high_task_awoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void qmsd_gui_record_vsync(int64_t timestamp_us)
{
    qmsd_gui_notify_vsync_from_isr();

    if (!s_render_stats_active || timestamp_us <= 0) {
        return;
    }

    portENTER_CRITICAL_ISR(&s_vsync_stats_mux);

    if (s_vsync_stats.last_us > 0 && timestamp_us > s_vsync_stats.last_us) {
        uint32_t interval_us = (uint32_t)(timestamp_us - s_vsync_stats.last_us);
        s_vsync_stats.interval_count++;
        s_vsync_stats.interval_total_us += interval_us;
        if (s_vsync_stats.interval_min_us == 0 ||
            interval_us < s_vsync_stats.interval_min_us) {
            s_vsync_stats.interval_min_us = interval_us;
        }
        if (interval_us > s_vsync_stats.interval_max_us) {
            s_vsync_stats.interval_max_us = interval_us;
        }

        if (s_vsync_stats.last_interval_us > 0) {
            uint32_t jitter_us = interval_us > s_vsync_stats.last_interval_us
                                     ? interval_us - s_vsync_stats.last_interval_us
                                     : s_vsync_stats.last_interval_us - interval_us;
            s_vsync_stats.jitter_count++;
            s_vsync_stats.jitter_total_us += jitter_us;
            if (jitter_us > s_vsync_stats.jitter_max_us) {
                s_vsync_stats.jitter_max_us = jitter_us;
            }
        }
        s_vsync_stats.last_interval_us = interval_us;
    }

    s_vsync_stats.last_us = timestamp_us;
    s_vsync_stats.count++;

    portEXIT_CRITICAL_ISR(&s_vsync_stats_mux);
}

static void qmsd_gui_vsync_stats_snapshot_and_reset(qmsd_gui_vsync_stats_t *snapshot)
{
    if (!snapshot) {
        return;
    }

    portENTER_CRITICAL(&s_vsync_stats_mux);
    *snapshot = s_vsync_stats;
    s_vsync_stats.count = 0;
    s_vsync_stats.interval_count = 0;
    s_vsync_stats.interval_total_us = 0;
    s_vsync_stats.interval_min_us = 0;
    s_vsync_stats.interval_max_us = 0;
    s_vsync_stats.jitter_count = 0;
    s_vsync_stats.jitter_total_us = 0;
    s_vsync_stats.jitter_max_us = 0;
    portEXIT_CRITICAL(&s_vsync_stats_mux);
}

static int64_t qmsd_gui_last_vsync_us(void)
{
    int64_t last_us;
    portENTER_CRITICAL(&s_vsync_stats_mux);
    last_us = s_vsync_stats.last_us;
    portEXIT_CRITICAL(&s_vsync_stats_mux);
    return last_us;
}

static void qmsd_gui_area_union_into(lv_area_t *dst, bool *valid, const lv_area_t *area)
{
    if (!dst || !valid || !area) {
        return;
    }

    if (!*valid) {
        *dst = *area;
        *valid = true;
        return;
    }

    if (area->x1 < dst->x1) dst->x1 = area->x1;
    if (area->y1 < dst->y1) dst->y1 = area->y1;
    if (area->x2 > dst->x2) dst->x2 = area->x2;
    if (area->y2 > dst->y2) dst->y2 = area->y2;
}

static void qmsd_gui_render_stats_record_flush(const lv_area_t *area,
                                               uint32_t pixels,
                                               uint32_t elapsed_us)
{
    s_render_stats.flush_calls++;
    s_render_stats.flush_total_us += elapsed_us;
    s_render_stats.flush_pixels += pixels;
    if (elapsed_us > s_render_stats.flush_max_us) {
        s_render_stats.flush_max_us = elapsed_us;
    }
    if (pixels > s_render_stats.flush_max_pixels) {
        s_render_stats.flush_max_pixels = pixels;
        if (area) {
            s_render_stats.flush_max_area = *area;
            s_render_stats.flush_max_area_valid = true;
        }
    }
    uint32_t display_pixels = 0;
    if (g_lvgl_config && g_lvgl_config->width && g_lvgl_config->hight) {
        display_pixels = (uint32_t)g_lvgl_config->width * (uint32_t)g_lvgl_config->hight;
    }
    if (display_pixels && pixels >= display_pixels) {
        s_render_stats.flush_full_calls++;
    }
}

static void qmsd_gui_render_stats_record_elapsed(uint32_t *count,
                                                 uint64_t *total_us,
                                                 uint32_t *max_us,
                                                 int64_t start_us,
                                                 int64_t now_us)
{
    if (start_us <= 0 || now_us < start_us) {
        return;
    }
    uint32_t elapsed_us = (uint32_t)(now_us - start_us);
    (*count)++;
    *total_us += elapsed_us;
    if (elapsed_us > *max_us) {
        *max_us = elapsed_us;
    }
}

static void qmsd_gui_render_stats_record_since_vsync(uint32_t *count,
                                                     uint64_t *total_us,
                                                     uint32_t *min_us,
                                                     uint32_t *max_us,
                                                     int64_t event_us,
                                                     int64_t vsync_us)
{
    if (vsync_us <= 0 || event_us < vsync_us) {
        return;
    }

    uint32_t elapsed_us = (uint32_t)(event_us - vsync_us);
    (*count)++;
    *total_us += elapsed_us;
    if (*min_us == 0 || elapsed_us < *min_us) {
        *min_us = elapsed_us;
    }
    if (elapsed_us > *max_us) {
        *max_us = elapsed_us;
    }
}

static void qmsd_gui_render_stats_start_render_work(int64_t now_us)
{
    s_render_stats.render_work_start_us = now_us;
    s_render_stats.render_work_start_vsync_us = qmsd_gui_last_vsync_us();
}

static void qmsd_gui_render_stats_finish_render_work(int64_t now_us)
{
    qmsd_gui_render_stats_record_elapsed(&s_render_stats.render_work_cycles,
                                         &s_render_stats.render_work_total_us,
                                         &s_render_stats.render_work_max_us,
                                         s_render_stats.render_work_start_us,
                                         now_us);
    if (s_render_stats.render_work_start_vsync_us > 0 &&
        qmsd_gui_last_vsync_us() > s_render_stats.render_work_start_vsync_us) {
        s_render_stats.render_work_cross_vsync++;
    }
    s_render_stats.render_work_start_us = 0;
    s_render_stats.render_work_start_vsync_us = 0;
}

static uint32_t qmsd_gui_area_pixels(const lv_area_t *area)
{
    if (!area || area->x2 < area->x1 || area->y2 < area->y1) {
        return 0;
    }
    return (uint32_t)(area->x2 - area->x1 + 1) *
           (uint32_t)(area->y2 - area->y1 + 1);
}

static void qmsd_gui_direct_flush_area_reset(void)
{
    s_direct_flush_area_valid = false;
    memset(&s_direct_flush_area, 0, sizeof(s_direct_flush_area));
}

static void qmsd_gui_direct_flush_area_add(const lv_area_t *area)
{
    if (!area) {
        return;
    }

    if (!s_direct_flush_area_valid) {
        s_direct_flush_area = *area;
        s_direct_flush_area_valid = true;
        return;
    }

    if (area->x1 < s_direct_flush_area.x1) s_direct_flush_area.x1 = area->x1;
    if (area->y1 < s_direct_flush_area.y1) s_direct_flush_area.y1 = area->y1;
    if (area->x2 > s_direct_flush_area.x2) s_direct_flush_area.x2 = area->x2;
    if (area->y2 > s_direct_flush_area.y2) s_direct_flush_area.y2 = area->y2;
}

static bool qmsd_gui_should_coalesce_direct_flush(lv_display_t *display)
{
    return g_lvgl_config &&
           g_lvgl_config->flags.direct_mode &&
           lv_display_is_double_buffered(display);
}

static bool qmsd_gui_area_covers_display(lv_display_t *display, const lv_area_t *area)
{
    if (!display || !area) {
        return false;
    }
    int32_t width = lv_display_get_horizontal_resolution(display);
    int32_t height = lv_display_get_vertical_resolution(display);
    if (width <= 0 || height <= 0) {
        return false;
    }
    return area->x1 <= 0 && area->y1 <= 0 &&
           area->x2 >= (width - 1) && area->y2 >= (height - 1);
}

static void qmsd_gui_render_stats_record_invalidation(lv_display_t *display, const lv_area_t *area)
{
    uint32_t pixels = qmsd_gui_area_pixels(area);

    s_render_stats.inv_calls++;
    s_render_stats.inv_pixels += pixels;
    if (pixels > s_render_stats.inv_max_pixels) {
        s_render_stats.inv_max_pixels = pixels;
        s_render_stats.inv_max_area = *area;
    }
    if (qmsd_gui_area_covers_display(display, area)) {
        s_render_stats.inv_full_calls++;
    }

    s_render_stats.inv_pending_calls++;
    s_render_stats.inv_pending_pixels += pixels;
    qmsd_gui_area_union_into(&s_render_stats.inv_pending_area,
                             &s_render_stats.inv_pending_area_valid,
                             area);
    bool pending_peak = false;
    if (s_render_stats.inv_pending_calls > s_render_stats.inv_pending_peak_calls) {
        s_render_stats.inv_pending_peak_calls = s_render_stats.inv_pending_calls;
        pending_peak = true;
    }
    uint32_t pending_kpx = (uint32_t)(s_render_stats.inv_pending_pixels / 1000ULL);
    if (pending_kpx > s_render_stats.inv_pending_peak_kpx) {
        s_render_stats.inv_pending_peak_kpx = pending_kpx;
        pending_peak = true;
    }
    if (pending_peak && s_render_stats.inv_pending_area_valid) {
        s_render_stats.inv_pending_peak_area = s_render_stats.inv_pending_area;
        s_render_stats.inv_pending_peak_area_valid = true;
    }
    if (!s_render_stats.inv_pressure_seen &&
        s_render_stats.inv_pending_calls > QMSD_GUI_INV_PRESSURE_AREAS) {
        s_render_stats.inv_pressure_seen = true;
        s_render_stats.inv_pressure_windows++;
    }
}

static void qmsd_gui_render_stats_reset_pending_invalidations(void)
{
    s_render_stats.inv_pending_calls = 0;
    s_render_stats.inv_pending_pixels = 0;
    memset(&s_render_stats.inv_pending_area, 0, sizeof(s_render_stats.inv_pending_area));
    s_render_stats.inv_pending_area_valid = false;
    s_render_stats.inv_pressure_seen = false;
}

static void qmsd_gui_display_event_cb(lv_event_t *event)
{
    if (!qmsd_gui_render_stats_enabled()) {
        return;
    }

    lv_event_code_t code = lv_event_get_code(event);
    int64_t now_us = esp_timer_get_time();

    switch (code) {
    case LV_EVENT_INVALIDATE_AREA:
        qmsd_gui_render_stats_record_invalidation((lv_display_t *)lv_event_get_target(event),
                                                  lv_event_get_invalidated_area(event));
        break;
    case LV_EVENT_REFR_START:
        s_render_stats.refr_start_us = now_us;
        break;
    case LV_EVENT_REFR_READY:
        qmsd_gui_render_stats_record_elapsed(&s_render_stats.refr_cycles,
                                             &s_render_stats.refr_total_us,
                                             &s_render_stats.refr_max_us,
                                             s_render_stats.refr_start_us,
                                             now_us);
        s_render_stats.refr_start_us = 0;
        qmsd_gui_render_stats_reset_pending_invalidations();
        break;
    case LV_EVENT_RENDER_START:
        s_render_stats.render_start_us = now_us;
        s_render_stats.render_start_vsync_us = qmsd_gui_last_vsync_us();
        qmsd_gui_render_stats_start_render_work(now_us);
        qmsd_gui_render_stats_record_since_vsync(&s_render_stats.render_since_vsync_count,
                                                 &s_render_stats.render_since_vsync_total_us,
                                                 &s_render_stats.render_since_vsync_min_us,
                                                 &s_render_stats.render_since_vsync_max_us,
                                                 now_us,
                                                 s_render_stats.render_start_vsync_us);
        break;
    case LV_EVENT_RENDER_READY:
        qmsd_gui_render_stats_finish_render_work(now_us);
        qmsd_gui_render_stats_record_elapsed(&s_render_stats.render_cycles,
                                             &s_render_stats.render_total_us,
                                             &s_render_stats.render_max_us,
                                             s_render_stats.render_start_us,
                                             now_us);
        if (s_render_stats.render_start_vsync_us > 0 &&
            qmsd_gui_last_vsync_us() > s_render_stats.render_start_vsync_us) {
            s_render_stats.render_cross_vsync++;
        }
        s_render_stats.render_start_us = 0;
        s_render_stats.render_start_vsync_us = 0;
        break;
    case LV_EVENT_FLUSH_START:
        qmsd_gui_render_stats_finish_render_work(now_us);
        s_render_stats.flush_event_start_us = now_us;
        s_render_stats.flush_event_start_vsync_us = qmsd_gui_last_vsync_us();
        qmsd_gui_render_stats_record_since_vsync(&s_render_stats.flush_since_vsync_count,
                                                 &s_render_stats.flush_since_vsync_total_us,
                                                 &s_render_stats.flush_since_vsync_min_us,
                                                 &s_render_stats.flush_since_vsync_max_us,
                                                 now_us,
                                                 s_render_stats.flush_event_start_vsync_us);
        break;
    case LV_EVENT_FLUSH_FINISH:
        if (s_render_stats.flush_event_start_us > 0 &&
            s_render_stats.flush_event_start_vsync_us > 0 &&
            qmsd_gui_last_vsync_us() > s_render_stats.flush_event_start_vsync_us) {
            s_render_stats.flush_cross_vsync++;
        }
        if (s_render_stats.flush_event_start_us > 0 && now_us >= s_render_stats.flush_event_start_us) {
            s_handler_flush_event_total_us += (uint32_t)(now_us - s_render_stats.flush_event_start_us);
        }
        s_render_stats.flush_event_start_us = 0;
        s_render_stats.flush_event_start_vsync_us = 0;
        if (s_render_stats.render_start_us > 0) {
            qmsd_gui_render_stats_start_render_work(now_us);
        }
        break;
    case LV_EVENT_FLUSH_WAIT_START:
        s_render_stats.flush_wait_start_us = now_us;
        break;
    case LV_EVENT_FLUSH_WAIT_FINISH:
        qmsd_gui_render_stats_record_elapsed(&s_render_stats.flush_waits,
                                             &s_render_stats.flush_wait_total_us,
                                             &s_render_stats.flush_wait_max_us,
                                             s_render_stats.flush_wait_start_us,
                                             now_us);
        s_render_stats.flush_wait_start_us = 0;
        break;
    default:
        break;
    }
}

static void qmsd_gui_render_stats_record_handler(uint32_t elapsed_us,
                                                 uint32_t lock_wait_us,
                                                 uint32_t lvgl_handler_us,
                                                 uint32_t flush_event_us,
                                                 bool lock_taken)
{
    if (!qmsd_gui_render_stats_enabled()) {
        return;
    }

    int64_t now_us = esp_timer_get_time();
    if (s_render_stats.window_start_us == 0) {
        s_render_stats.window_start_us = now_us;
    }

    s_render_stats.handler_calls++;
    s_render_stats.handler_total_us += elapsed_us;
    if (elapsed_us > s_render_stats.handler_max_us) {
        s_render_stats.handler_max_us = elapsed_us;
    }
    if (elapsed_us >= QMSD_GUI_SLOW_HANDLER_US) {
        s_render_stats.handler_slow_16ms++;
    }
    if (elapsed_us >= QMSD_GUI_VERY_SLOW_HANDLER_US) {
        s_render_stats.handler_slow_25ms++;
    }

    if (lock_taken) {
        s_render_stats.lock_wait_calls++;
        s_render_stats.lock_wait_total_us += lock_wait_us;
        if (lock_wait_us > s_render_stats.lock_wait_max_us) {
            s_render_stats.lock_wait_max_us = lock_wait_us;
        }

        s_render_stats.lvgl_handler_calls++;
        s_render_stats.lvgl_handler_total_us += lvgl_handler_us;
        if (lvgl_handler_us > s_render_stats.lvgl_handler_max_us) {
            s_render_stats.lvgl_handler_max_us = lvgl_handler_us;
        }

        uint32_t lvgl_work_us = lvgl_handler_us > flush_event_us
                                    ? lvgl_handler_us - flush_event_us
                                    : 0;
        s_render_stats.lvgl_work_calls++;
        s_render_stats.lvgl_work_total_us += lvgl_work_us;
        if (lvgl_work_us > s_render_stats.lvgl_work_max_us) {
            s_render_stats.lvgl_work_max_us = lvgl_work_us;
        }
        if (lvgl_work_us >= QMSD_GUI_SLOW_HANDLER_US) {
            s_render_stats.lvgl_work_slow_16ms++;
        }
        if (lvgl_work_us >= QMSD_GUI_VERY_SLOW_HANDLER_US) {
            s_render_stats.lvgl_work_slow_25ms++;
        }
    }

    int64_t window_us = now_us - s_render_stats.window_start_us;
    if (window_us < QMSD_GUI_STATS_PERIOD_US) {
        return;
    }

    uint32_t handler_avg_us = s_render_stats.handler_calls
                                  ? (uint32_t)(s_render_stats.handler_total_us / s_render_stats.handler_calls)
                                  : 0;
    uint32_t flush_avg_us = s_render_stats.flush_calls
                                ? (uint32_t)(s_render_stats.flush_total_us / s_render_stats.flush_calls)
                                : 0;
    uint32_t refr_avg_us = s_render_stats.refr_cycles
                               ? (uint32_t)(s_render_stats.refr_total_us / s_render_stats.refr_cycles)
                               : 0;
    uint32_t render_avg_us = s_render_stats.render_cycles
                                 ? (uint32_t)(s_render_stats.render_total_us / s_render_stats.render_cycles)
                                 : 0;
    uint32_t render_work_avg_us = s_render_stats.render_work_cycles
                                      ? (uint32_t)(s_render_stats.render_work_total_us /
                                                   s_render_stats.render_work_cycles)
                                      : 0;
    uint32_t render_since_vsync_avg_us = s_render_stats.render_since_vsync_count
                                             ? (uint32_t)(s_render_stats.render_since_vsync_total_us /
                                                          s_render_stats.render_since_vsync_count)
                                             : 0;
    uint32_t flush_since_vsync_avg_us = s_render_stats.flush_since_vsync_count
                                            ? (uint32_t)(s_render_stats.flush_since_vsync_total_us /
                                                         s_render_stats.flush_since_vsync_count)
                                            : 0;
    uint32_t flush_wait_avg_us = s_render_stats.flush_waits
                                     ? (uint32_t)(s_render_stats.flush_wait_total_us / s_render_stats.flush_waits)
                                     : 0;
    uint32_t lock_wait_avg_us = s_render_stats.lock_wait_calls
                                    ? (uint32_t)(s_render_stats.lock_wait_total_us / s_render_stats.lock_wait_calls)
                                    : 0;
    uint32_t lvgl_handler_avg_us = s_render_stats.lvgl_handler_calls
                                       ? (uint32_t)(s_render_stats.lvgl_handler_total_us / s_render_stats.lvgl_handler_calls)
                                       : 0;
    uint32_t lvgl_work_avg_us = s_render_stats.lvgl_work_calls
                                    ? (uint32_t)(s_render_stats.lvgl_work_total_us / s_render_stats.lvgl_work_calls)
                                    : 0;
    uint32_t refr_fps_x10 = window_us
                                ? (uint32_t)(((uint64_t)s_render_stats.refr_cycles * 10000000ULL) /
                                             (uint64_t)window_us)
                                : 0;
    qmsd_gui_vsync_stats_t vsync = {0};
    qmsd_gui_vsync_stats_snapshot_and_reset(&vsync);
    uint32_t vsync_avg_us = vsync.interval_count
                                ? (uint32_t)(vsync.interval_total_us / vsync.interval_count)
                                : 0;
    uint32_t vsync_jitter_avg_us = vsync.jitter_count
                                       ? (uint32_t)(vsync.jitter_total_us / vsync.jitter_count)
                                       : 0;
    ESP_LOGI(TAG,
             "render handlers=%lu avg_us=%lu max_us=%lu slow16=%lu slow25=%lu lock_avg_us=%lu lock_max_us=%lu lvgl_avg_us=%lu lvgl_max_us=%lu lvgl_work_avg_us=%lu lvgl_work_max_us=%lu slow_work16=%lu slow_work25=%lu refresh=%lu fps=%lu.%01lu refresh_avg_us=%lu refresh_max_us=%lu "
             "render=%lu render_avg_us=%lu render_max_us=%lu render_vsync_avg_us=%lu render_vsync_min_us=%lu render_vsync_max_us=%lu render_cross_vsync=%lu render_work=%lu render_work_avg_us=%lu render_work_max_us=%lu render_work_cross_vsync=%lu "
             "flushes=%lu flush_full=%lu flush_kpx=%llu flush_avg_us=%lu flush_max_us=%lu flush_max_px=%lu flush_max_area=%ld,%ld,%ld,%ld flush_vsync_avg_us=%lu flush_vsync_min_us=%lu flush_vsync_max_us=%lu flush_cross_vsync=%lu "
             "wait=%lu wait_avg_us=%lu wait_max_us=%lu vsync=%lu vsync_avg_us=%lu vsync_min_us=%lu vsync_max_us=%lu jitter_avg_us=%lu jitter_max_us=%lu "
             "inv=%lu inv_kpx=%llu inv_max_px=%lu inv_max_area=%ld,%ld,%ld,%ld inv_full=%lu inv_pending_peak=%lu inv_pending_peak_kpx=%lu inv_pending_peak_area=%ld,%ld,%ld,%ld inv_over32_windows=%lu",
             (unsigned long)s_render_stats.handler_calls,
             (unsigned long)handler_avg_us,
             (unsigned long)s_render_stats.handler_max_us,
             (unsigned long)s_render_stats.handler_slow_16ms,
             (unsigned long)s_render_stats.handler_slow_25ms,
             (unsigned long)lock_wait_avg_us,
             (unsigned long)s_render_stats.lock_wait_max_us,
             (unsigned long)lvgl_handler_avg_us,
             (unsigned long)s_render_stats.lvgl_handler_max_us,
             (unsigned long)lvgl_work_avg_us,
             (unsigned long)s_render_stats.lvgl_work_max_us,
             (unsigned long)s_render_stats.lvgl_work_slow_16ms,
             (unsigned long)s_render_stats.lvgl_work_slow_25ms,
             (unsigned long)s_render_stats.refr_cycles,
             (unsigned long)(refr_fps_x10 / 10),
             (unsigned long)(refr_fps_x10 % 10),
             (unsigned long)refr_avg_us,
             (unsigned long)s_render_stats.refr_max_us,
             (unsigned long)s_render_stats.render_cycles,
             (unsigned long)render_avg_us,
             (unsigned long)s_render_stats.render_max_us,
             (unsigned long)render_since_vsync_avg_us,
             (unsigned long)s_render_stats.render_since_vsync_min_us,
             (unsigned long)s_render_stats.render_since_vsync_max_us,
             (unsigned long)s_render_stats.render_cross_vsync,
             (unsigned long)s_render_stats.render_work_cycles,
             (unsigned long)render_work_avg_us,
             (unsigned long)s_render_stats.render_work_max_us,
             (unsigned long)s_render_stats.render_work_cross_vsync,
             (unsigned long)s_render_stats.flush_calls,
             (unsigned long)s_render_stats.flush_full_calls,
             (unsigned long long)(s_render_stats.flush_pixels / 1000ULL),
             (unsigned long)flush_avg_us,
             (unsigned long)s_render_stats.flush_max_us,
             (unsigned long)s_render_stats.flush_max_pixels,
             (long)s_render_stats.flush_max_area.x1,
             (long)s_render_stats.flush_max_area.y1,
             (long)s_render_stats.flush_max_area.x2,
             (long)s_render_stats.flush_max_area.y2,
             (unsigned long)flush_since_vsync_avg_us,
             (unsigned long)s_render_stats.flush_since_vsync_min_us,
             (unsigned long)s_render_stats.flush_since_vsync_max_us,
             (unsigned long)s_render_stats.flush_cross_vsync,
             (unsigned long)s_render_stats.flush_waits,
             (unsigned long)flush_wait_avg_us,
             (unsigned long)s_render_stats.flush_wait_max_us,
             (unsigned long)vsync.count,
             (unsigned long)vsync_avg_us,
             (unsigned long)vsync.interval_min_us,
             (unsigned long)vsync.interval_max_us,
             (unsigned long)vsync_jitter_avg_us,
             (unsigned long)vsync.jitter_max_us,
             (unsigned long)s_render_stats.inv_calls,
             (unsigned long long)(s_render_stats.inv_pixels / 1000ULL),
             (unsigned long)s_render_stats.inv_max_pixels,
             (long)s_render_stats.inv_max_area.x1,
             (long)s_render_stats.inv_max_area.y1,
             (long)s_render_stats.inv_max_area.x2,
             (long)s_render_stats.inv_max_area.y2,
             (unsigned long)s_render_stats.inv_full_calls,
             (unsigned long)s_render_stats.inv_pending_peak_calls,
             (unsigned long)s_render_stats.inv_pending_peak_kpx,
             (long)s_render_stats.inv_pending_peak_area.x1,
             (long)s_render_stats.inv_pending_peak_area.y1,
             (long)s_render_stats.inv_pending_peak_area.x2,
             (long)s_render_stats.inv_pending_peak_area.y2,
             (unsigned long)s_render_stats.inv_pressure_windows);

    memset(&s_render_stats, 0, sizeof(s_render_stats));
    s_render_stats.window_start_us = now_us;
}

typedef struct {
    int offsetx1;
    int offsetx2;
    int offsety1;
    int offsety2;
    uint8_t* color;
    lv_display_t* display;
} show_data_t;

static void refresh_task(void* arg) {
    (void)arg;
    show_data_t* show_data;
    for (;;) {
        if (xQueuePeek(g_image_queue, &show_data, portMAX_DELAY) == pdTRUE) {
            int offsetx1 = show_data->offsetx1;
            int w = show_data->offsetx2 - show_data->offsetx1 + 1;
            int offsety1 = show_data->offsety1;
            int h = show_data->offsety2 - show_data->offsety1 + 1;
            lv_area_t flush_area = {
                .x1 = show_data->offsetx1,
                .y1 = show_data->offsety1,
                .x2 = show_data->offsetx2,
                .y2 = show_data->offsety2,
            };
            bool stats_enabled = qmsd_gui_render_stats_enabled();
            int64_t flush_start_us = stats_enabled ? esp_timer_get_time() : 0;
            g_lvgl_config->draw_bitmap(offsetx1, offsety1, w, h, (uint16_t*)show_data->color);
            if (stats_enabled) {
                qmsd_gui_render_stats_record_flush(&flush_area,
                                                   (uint32_t)(w * h),
                                                   (uint32_t)(esp_timer_get_time() - flush_start_us));
            }
            xQueueReceive(g_image_queue, &show_data, 0);
            free(show_data);
        }
    }
}

static void lvgl_task_refresh(lv_display_t* display, const lv_area_t* area, uint8_t* color_map) {
    if (qmsd_gui_should_coalesce_direct_flush(display)) {
        qmsd_gui_direct_flush_area_add(area);
        if (!lv_display_flush_is_last(display)) {
            lv_display_flush_ready(display);
            return;
        }
        area = &s_direct_flush_area;
    }

    show_data_t* show_data = (show_data_t*)calloc(1, sizeof(show_data_t));
    show_data->display = display;
    show_data->offsetx1 = area->x1;
    show_data->offsetx2 = area->x2;
    show_data->offsety1 = area->y1;
    show_data->offsety2 = area->y2;
    show_data->color = color_map;
    xQueueSend(g_image_queue, &show_data, portMAX_DELAY);
    qmsd_gui_direct_flush_area_reset();
    lv_display_flush_ready(display);
}

static void lvgl_flush(lv_display_t* display, const lv_area_t* area, uint8_t* color_map) {
    if (qmsd_gui_should_coalesce_direct_flush(display)) {
        qmsd_gui_direct_flush_area_add(area);
        if (!lv_display_flush_is_last(display)) {
            lv_display_flush_ready(display);
            return;
        }
        area = &s_direct_flush_area;
    }

    uint16_t w = (uint16_t)(area->x2 - area->x1 + 1);
    uint16_t h = (uint16_t)(area->y2 - area->y1 + 1);
    bool stats_enabled = qmsd_gui_render_stats_enabled();
    int64_t flush_start_us = stats_enabled ? esp_timer_get_time() : 0;
    g_lvgl_config->draw_bitmap(area->x1, area->y1, w, h, (uint16_t*)color_map);
    if (stats_enabled) {
        qmsd_gui_render_stats_record_flush(area,
                                           (uint32_t)w * (uint32_t)h,
                                           (uint32_t)(esp_timer_get_time() - flush_start_us));
    }
    qmsd_gui_direct_flush_area_reset();
    lv_display_flush_ready(display);
}

static void lvgl_tp_read(lv_indev_t* indev, lv_indev_data_t* data) {
    (void)indev;
    uint8_t press = 0;
    uint16_t x, y;
    g_lvgl_config->touch_read(&press, &x, &y);
    if (press) {
        data->point.x = x;
        data->point.y = y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
    // If the touch task buffered multiple samples while the GUI task was busy,
    // ask LVGL to call read_cb again immediately to drain the backlog in a single
    // lv_timer_handler() iteration.
    data->continue_reading = (touch_samples_waiting() > 0);
}

static void lvgl_encoder_read(lv_indev_t* indev, lv_indev_data_t* data) {
    (void)indev;
    uint8_t press = 0;
    int16_t enc_diff = 0;
    g_lvgl_config->encoder_read(&press, &enc_diff);
    data->enc_diff = enc_diff;
    if (press) {
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void increase_lvgl_tick(void* arg) {
    lv_tick_inc(portTICK_PERIOD_MS);
}

static bool qmsd_gui_direct_mode_phase_sync_enabled(void)
{
    return g_lvgl_config && g_lvgl_config->flags.direct_mode;
}

static bool qmsd_gui_wait_for_direct_render_vsync(void)
{
    if (!qmsd_gui_direct_mode_phase_sync_enabled()) {
        return false;
    }

    int64_t now_us = esp_timer_get_time();
    int64_t last_vsync_us = qmsd_gui_last_vsync_us();
    if (last_vsync_us > 0 &&
        now_us >= last_vsync_us &&
        (now_us - last_vsync_us) <= QMSD_GUI_DIRECT_RENDER_PHASE_WINDOW_US) {
        (void)ulTaskNotifyTake(pdTRUE, 0);
        return true;
    }

    /* Drop stale VSYNC notifications from work that completed after the last scan edge. */
    (void)ulTaskNotifyTake(pdTRUE, 0);
    return ulTaskNotifyTake(pdTRUE,
                            pdMS_TO_TICKS(QMSD_GUI_DIRECT_RENDER_VSYNC_WAIT_TIMEOUT_MS)) > 0;
}

static void gui_update_task(void* arg) {
    s_gui_update_task_handle = xTaskGetCurrentTaskHandle();
    while (1) {
        bool phase_synced = qmsd_gui_wait_for_direct_render_vsync();
        int64_t handler_start_us = esp_timer_get_time();
        int64_t lock_acquired_us = 0;
        int64_t lvgl_done_us = handler_start_us;
        bool lock_taken = false;
        if (qmsd_gui_lock(portMAX_DELAY) == 0) {
            lock_taken = true;
            lock_acquired_us = esp_timer_get_time();
            s_handler_flush_event_total_us = 0;
            lv_timer_handler();
            lvgl_done_us = esp_timer_get_time();
            qmsd_gui_unlock();
        }

        uint32_t handler_end = (uint32_t)(esp_timer_get_time() - handler_start_us);
        uint32_t lock_wait_us = lock_taken ? (uint32_t)(lock_acquired_us - handler_start_us) : handler_end;
        uint32_t lvgl_handler_us = lock_taken ? (uint32_t)(lvgl_done_us - lock_acquired_us) : 0;
        uint32_t flush_event_us = lock_taken ? s_handler_flush_event_total_us : 0;
        s_handler_flush_event_total_us = 0;
        qmsd_gui_render_stats_record_handler(handler_end,
                                             lock_wait_us,
                                             lvgl_handler_us,
                                             flush_event_us,
                                             lock_taken);
        if (phase_synced) {
            vTaskDelay(pdMS_TO_TICKS(1));
        } else if (handler_end > 50000) {
            vTaskDelay(pdMS_TO_TICKS(3));
        } else if (handler_end > 25000) {
            vTaskDelay(pdMS_TO_TICKS(6));
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void __attribute__((weak)) gui_user_init(void) {
}

static void configure_indev_read_period(lv_indev_t *indev)
{
    lv_timer_t *read_timer = lv_indev_get_read_timer(indev);
    if (read_timer) {
        lv_timer_set_period(read_timer, QMSD_GUI_INDEV_READ_PERIOD_MS);
    }
}

void qmsd_gui_init(qmsd_gui_config_t* lvgl_config) {
    g_lvgl_config = (qmsd_gui_config_t*)malloc(sizeof(qmsd_gui_config_t));
    memcpy(g_lvgl_config, lvgl_config, sizeof(qmsd_gui_config_t));
    (void)qmsd_gui_render_stats_enabled();

    g_gui_semaphore = xSemaphoreCreateMutex();

    lv_init();
    lv_display_t *display = lv_display_create(lvgl_config->width, lvgl_config->hight);
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    lv_display_add_event_cb(display, qmsd_gui_display_event_cb, LV_EVENT_ALL, NULL);

    lv_display_render_mode_t render_mode = LV_DISPLAY_RENDER_MODE_PARTIAL;
    if (lvgl_config->flags.full_refresh) {
        render_mode = LV_DISPLAY_RENDER_MODE_FULL;
    } else if (lvgl_config->flags.direct_mode) {
        render_mode = LV_DISPLAY_RENDER_MODE_DIRECT;
    }
    lv_display_set_buffers(display, lvgl_config->buffer[0], lvgl_config->buffer[1], lvgl_config->buffer_size, render_mode);

    if (lvgl_config->refresh_task.en) {
        g_image_queue = xQueueCreate(1, sizeof(show_data_t*));
        qmsd_thread_create(refresh_task, "gui-refresh", lvgl_config->refresh_task.stack_size, NULL, lvgl_config->refresh_task.priority, NULL, lvgl_config->refresh_task.core,
                           lvgl_config->refresh_task.task_in_psram);
        lv_display_set_flush_cb(display, lvgl_task_refresh);
    } else {
        lv_display_set_flush_cb(display, lvgl_flush);
    }

    lv_display_set_default(display);

    if (lvgl_config->touch_read) {
        lv_indev_t *indev = lv_indev_create();
        lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
        lv_indev_set_read_cb(indev, lvgl_tp_read);
        lv_indev_set_display(indev, display);
        configure_indev_read_period(indev);
    }

    if (lvgl_config->encoder_read) {
        lv_indev_t *indev = lv_indev_create();
        lv_indev_set_type(indev, LV_INDEV_TYPE_ENCODER);
        lv_indev_set_read_cb(indev, lvgl_encoder_read);
        lv_indev_set_display(indev, display);
        configure_indev_read_period(indev);
    }

    // Tick interface for LVGL
    const esp_timer_create_args_t periodic_timer_args = {.callback = increase_lvgl_tick, .name = "periodic_gui"};
    esp_timer_handle_t periodic_timer;
    esp_timer_create(&periodic_timer_args, &periodic_timer);
    esp_timer_start_periodic(periodic_timer, portTICK_PERIOD_MS * 1000);

    gui_user_init();

    if (lvgl_config->update_task.en) {
        qmsd_thread_create(gui_update_task, "gui-update", lvgl_config->update_task.stack_size, NULL, lvgl_config->update_task.priority, NULL, lvgl_config->update_task.core,
                           lvgl_config->update_task.task_in_psram);
    }
}

void qmsd_gui_loop() {
}

int qmsd_gui_lock(uint32_t ticks) {
    return (xSemaphoreTake(g_gui_semaphore, ticks) == pdTRUE) ? 0 : -1;
}

void qmsd_gui_unlock() {
    xSemaphoreGive(g_gui_semaphore);
}
