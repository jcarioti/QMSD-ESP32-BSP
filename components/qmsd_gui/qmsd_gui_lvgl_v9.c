#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "qmsd_gui.h"
#include "qmsd_utils.h"
#include "lvgl.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>

#ifndef QMSD_GUI_RENDER_TELEMETRY_ENABLED
#define QMSD_GUI_RENDER_TELEMETRY_ENABLED 0
#endif

// Forward declare to avoid adding a build-time dependency edge from qmsd_gui -> qmsd_touch.
uint32_t touch_samples_waiting(void);
void qmsd_lcd_rgb_panel_phase_stats_log(void) __attribute__((weak));
const char *dashboard_widget_inv_trace_recent_summary(void) __attribute__((weak));

#define QMSD_GUI_INDEV_READ_PERIOD_MS 10
#define QMSD_GUI_STATS_PERIOD_US 1000000
#define QMSD_GUI_SLOW_HANDLER_US 16000
#define QMSD_GUI_VERY_SLOW_HANDLER_US 25000
#define QMSD_GUI_INV_PRESSURE_AREAS 32
#define QMSD_GUI_DIRECT_RENDER_VSYNC_WAIT_TIMEOUT_MS 25
#define QMSD_GUI_DIRECT_RENDER_DEFAULT_INTERVAL_US 18762
#define QMSD_GUI_DIRECT_RENDER_LEAD_US 14000
#define QMSD_GUI_DIRECT_RENDER_MIN_REMAIN_US 12000
#define QMSD_GUI_DIRECT_RENDER_PRESSURE_LEAD_US 18000
#define QMSD_GUI_DIRECT_RENDER_PRESSURE_MIN_REMAIN_US 17000
#define QMSD_GUI_DIRECT_RENDER_PRESSURE_TIMER_US 1500
#define QMSD_GUI_DIRECT_RENDER_PRESSURE_INV_AREAS 8
#define QMSD_GUI_DIRECT_RENDER_PRESSURE_INV_PIXELS 40000ULL
#define QMSD_GUI_DIRECT_RENDER_SLICE_MIN_REMAIN_US 12000
#define QMSD_GUI_DIRECT_RENDER_SLICE_FENCE_INV_AREAS 8
#define QMSD_GUI_DIRECT_RENDER_SLICE_FENCE_INV_PIXELS 50000ULL
#define QMSD_GUI_DIRECT_RENDER_SLICE_FENCE_UNION_PIXELS 80000U
#define QMSD_GUI_DIRECT_RENDER_SLICE_FENCE_FLUSH_US 15500U
#define QMSD_GUI_TIMER_TRACE_STACK_DEPTH 4
#define QMSD_GUI_VSYNC_RISK_RECENT_LEN 192
#define QMSD_GUI_STATS_LOG_TASK_STACK 6144
#define QMSD_GUI_STATS_LOG_TASK_PRIORITY 1
#define QMSD_GUI_STATS_LOG_TASK_CORE 0

static const char *TAG = "QMSD_GUI";

static qmsd_gui_config_t* g_lvgl_config;
static QueueHandle_t g_image_queue;
static SemaphoreHandle_t g_gui_semaphore = NULL;
static lv_display_t *s_lvgl_display = NULL;
static TaskHandle_t s_gui_update_task_handle = NULL;
static TaskHandle_t s_render_stats_log_task_handle = NULL;
static QueueHandle_t s_render_stats_log_queue = NULL;
static volatile bool s_render_stats_active = false;
static bool s_direct_manual_refresh = false;
static lv_area_t s_direct_flush_area;
static bool s_direct_flush_area_valid = false;
static uint32_t s_handler_flush_event_total_us = 0;
static uint32_t s_handler_flush_wait_total_us = 0;
static uint32_t s_handler_refr_cycles = 0;
static uint32_t s_handler_render_cycles = 0;
static uint32_t s_handler_flushes = 0;
static uint32_t s_handler_render_slice_waits = 0;
static uint32_t s_handler_render_slice_wait_total_us = 0;
static uint32_t s_handler_render_slice_wait_max_us = 0;
static bool s_handler_last_flush_was_last = true;
static int64_t s_gui_update_vsync_consumed_us = 0;

static bool qmsd_gui_direct_render_slice_fence(void);

typedef struct {
    uint32_t inv_pending_calls;
    uint64_t inv_pending_pixels;
    lv_area_t inv_pending_area;
    bool inv_pending_area_valid;
    bool render_active;
    int64_t flush_event_start_us;
} qmsd_gui_direct_render_state_t;

static qmsd_gui_direct_render_state_t s_direct_render_state;

#if QMSD_GUI_RENDER_TELEMETRY_ENABLED
typedef struct {
    int64_t start_us;
    lv_timer_cb_t cb;
} qmsd_gui_lvgl_timer_trace_frame_t;

static qmsd_gui_lvgl_timer_trace_frame_t
    s_lvgl_timer_trace_stack[QMSD_GUI_TIMER_TRACE_STACK_DEPTH];
static uint8_t s_lvgl_timer_trace_depth = 0;
#endif

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
    uint32_t handler_wall_calls;
    uint64_t handler_wall_total_us;
    uint32_t handler_wall_max_us;
    uint32_t lvgl_timer_calls;
    uint64_t lvgl_timer_total_us;
    uint32_t lvgl_timer_max_us;
    uint32_t lvgl_timer_cb_calls;
    uint64_t lvgl_timer_cb_total_us;
    uint32_t lvgl_timer_cb_max_us;
    uint32_t lvgl_timer_cb_slow_16ms;
    uint32_t lvgl_timer_cb_slow_25ms;
    lv_timer_cb_t lvgl_timer_cb_max;
    uint32_t manual_refresh_calls;
    uint64_t manual_refresh_total_us;
    uint32_t manual_refresh_max_us;
    uint32_t phase_wait_calls;
    uint64_t phase_wait_total_us;
    uint32_t phase_wait_max_us;
    uint32_t render_slice_wait_calls;
    uint64_t render_slice_wait_total_us;
    uint32_t render_slice_wait_max_us;
    uint32_t handler_refresh_max;
    uint32_t handler_render_max;
    uint32_t handler_flush_max;
    uint32_t handler_flush_event_max_us;
    uint32_t handler_flush_wait_max_us;
    uint32_t handler_phase_synced;
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
    uint32_t render_fenced_cross_vsync;
    uint32_t render_unfenced_cross_vsync;
    bool render_slice_fenced;
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
    uint32_t flush_cross_start_since_vsync_us;
    uint32_t flush_cross_elapsed_us;
    char flush_cross_recent[QMSD_GUI_VSYNC_RISK_RECENT_LEN];
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

typedef struct {
    uint32_t handler_calls;
    uint32_t handler_avg_us;
    uint32_t handler_max_us;
    uint32_t handler_slow_16ms;
    uint32_t handler_slow_25ms;
    uint32_t lock_wait_avg_us;
    uint32_t lock_wait_max_us;
    uint32_t lvgl_handler_avg_us;
    uint32_t lvgl_handler_max_us;
    uint32_t lvgl_work_avg_us;
    uint32_t lvgl_work_max_us;
    uint32_t lvgl_work_slow_16ms;
    uint32_t lvgl_work_slow_25ms;
    uint32_t handler_wall_avg_us;
    uint32_t handler_wall_max_us;
    uint32_t lvgl_timer_avg_us;
    uint32_t lvgl_timer_max_us;
    uint32_t lvgl_timer_cb_calls;
    uint32_t lvgl_timer_cb_avg_us;
    uint32_t lvgl_timer_cb_max_us;
    uint32_t lvgl_timer_cb_slow_16ms;
    uint32_t lvgl_timer_cb_slow_25ms;
    lv_timer_cb_t lvgl_timer_cb_max;
    uint32_t manual_refresh_avg_us;
    uint32_t manual_refresh_max_us;
    uint32_t phase_wait_avg_us;
    uint32_t phase_wait_max_us;
    uint32_t render_slice_wait_calls;
    uint32_t render_slice_wait_avg_us;
    uint32_t render_slice_wait_max_us;
    uint32_t handler_refresh_max;
    uint32_t handler_render_max;
    uint32_t handler_flush_max;
    uint32_t handler_flush_event_max_us;
    uint32_t handler_flush_wait_max_us;
    uint32_t handler_phase_synced;
    uint32_t refr_cycles;
    uint32_t refr_fps_x10;
    uint32_t refr_avg_us;
    uint32_t refr_max_us;
    uint32_t render_cycles;
    uint32_t render_avg_us;
    uint32_t render_max_us;
    uint32_t render_since_vsync_avg_us;
    uint32_t render_since_vsync_min_us;
    uint32_t render_since_vsync_max_us;
    uint32_t render_cross_vsync;
    uint32_t render_fenced_cross_vsync;
    uint32_t render_unfenced_cross_vsync;
    uint32_t render_work_cycles;
    uint32_t render_work_avg_us;
    uint32_t render_work_max_us;
    uint32_t render_work_cross_vsync;
    uint32_t flush_calls;
    uint32_t flush_full_calls;
    uint64_t flush_kpx;
    uint32_t flush_avg_us;
    uint32_t flush_max_us;
    uint32_t flush_max_pixels;
    lv_area_t flush_max_area;
    uint32_t flush_since_vsync_avg_us;
    uint32_t flush_since_vsync_min_us;
    uint32_t flush_since_vsync_max_us;
    uint32_t flush_cross_vsync;
    uint32_t flush_cross_start_since_vsync_us;
    uint32_t flush_cross_elapsed_us;
    char flush_cross_recent[QMSD_GUI_VSYNC_RISK_RECENT_LEN];
    uint32_t flush_waits;
    uint32_t flush_wait_avg_us;
    uint32_t flush_wait_max_us;
    uint32_t vsync_count;
    uint32_t vsync_avg_us;
    uint32_t vsync_min_us;
    uint32_t vsync_max_us;
    uint32_t vsync_jitter_avg_us;
    uint32_t vsync_jitter_max_us;
    uint32_t inv_calls;
    uint64_t inv_kpx;
    uint32_t inv_max_pixels;
    lv_area_t inv_max_area;
    uint32_t inv_full_calls;
    uint32_t inv_pending_peak_calls;
    uint32_t inv_pending_peak_kpx;
    lv_area_t inv_pending_peak_area;
    uint32_t inv_pressure_windows;
} qmsd_gui_render_stats_log_snapshot_t;

static bool qmsd_gui_render_stats_enabled(void)
{
#if QMSD_GUI_RENDER_TELEMETRY_ENABLED
    bool enabled = esp_log_level_get(TAG) >= ESP_LOG_INFO;
    s_render_stats_active = enabled;
    return enabled;
#else
    s_render_stats_active = false;
    return false;
#endif
}

void qmsd_gui_lvgl_timer_exec_trace_begin(lv_timer_t *timer, lv_timer_cb_t cb)
{
    (void)timer;
#if !QMSD_GUI_RENDER_TELEMETRY_ENABLED
    (void)cb;
    return;
#else
    if (!s_render_stats_active || !cb) {
        return;
    }
    if (s_lvgl_timer_trace_depth >= QMSD_GUI_TIMER_TRACE_STACK_DEPTH) {
        return;
    }

    s_lvgl_timer_trace_stack[s_lvgl_timer_trace_depth++] =
        (qmsd_gui_lvgl_timer_trace_frame_t){
            .start_us = esp_timer_get_time(),
            .cb = cb,
        };
#endif
}

void qmsd_gui_lvgl_timer_exec_trace_end(lv_timer_t *timer, lv_timer_cb_t cb)
{
    (void)timer;
#if !QMSD_GUI_RENDER_TELEMETRY_ENABLED
    (void)cb;
    return;
#else
    if (!s_render_stats_active || s_lvgl_timer_trace_depth == 0) {
        return;
    }

    qmsd_gui_lvgl_timer_trace_frame_t frame =
        s_lvgl_timer_trace_stack[--s_lvgl_timer_trace_depth];
    if (frame.start_us <= 0) {
        return;
    }

    uint32_t elapsed_us = (uint32_t)(esp_timer_get_time() - frame.start_us);
    lv_timer_cb_t timer_cb = frame.cb ? frame.cb : cb;
    s_render_stats.lvgl_timer_cb_calls++;
    s_render_stats.lvgl_timer_cb_total_us += elapsed_us;
    if (elapsed_us > s_render_stats.lvgl_timer_cb_max_us) {
        s_render_stats.lvgl_timer_cb_max_us = elapsed_us;
        s_render_stats.lvgl_timer_cb_max = timer_cb;
    }
    if (elapsed_us >= QMSD_GUI_SLOW_HANDLER_US) {
        s_render_stats.lvgl_timer_cb_slow_16ms++;
    }
    if (elapsed_us >= QMSD_GUI_VERY_SLOW_HANDLER_US) {
        s_render_stats.lvgl_timer_cb_slow_25ms++;
    }
#endif
}

static void qmsd_gui_render_stats_log_task(void *arg)
{
    (void)arg;
    qmsd_gui_render_stats_log_snapshot_t snap;
    for (;;) {
        if (xQueueReceive(s_render_stats_log_queue, &snap, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (esp_log_level_get(TAG) < ESP_LOG_INFO) {
            continue;
        }
        ESP_LOGI(TAG,
                 "render handlers=%lu avg_us=%lu max_us=%lu slow16=%lu slow25=%lu lock_avg_us=%lu lock_max_us=%lu lvgl_avg_us=%lu lvgl_max_us=%lu lvgl_work_avg_us=%lu lvgl_work_max_us=%lu slow_work16=%lu slow_work25=%lu wall_avg_us=%lu wall_max_us=%lu timer_avg_us=%lu timer_max_us=%lu timer_cb_calls=%lu timer_cb_avg_us=%lu timer_cb_max_us=%lu timer_cb_slow16=%lu timer_cb_slow25=%lu timer_cb_max=%p manual_avg_us=%lu manual_max_us=%lu phase_wait_avg_us=%lu phase_wait_max_us=%lu slice_wait=%lu slice_wait_avg_us=%lu slice_wait_max_us=%lu handler_refresh_max=%lu handler_render_max=%lu handler_flush_max=%lu handler_flush_event_max_us=%lu handler_flush_wait_max_us=%lu phase_sync=%lu refresh=%lu fps=%lu.%01lu refresh_avg_us=%lu refresh_max_us=%lu "
                 "render=%lu render_avg_us=%lu render_max_us=%lu render_vsync_avg_us=%lu render_vsync_min_us=%lu render_vsync_max_us=%lu render_cross_vsync=%lu render_fenced_cross_vsync=%lu render_unfenced_cross_vsync=%lu render_work=%lu render_work_avg_us=%lu render_work_max_us=%lu render_work_cross_vsync=%lu "
                 "flushes=%lu flush_full=%lu flush_kpx=%llu flush_avg_us=%lu flush_max_us=%lu flush_max_px=%lu flush_max_area=%ld,%ld,%ld,%ld flush_vsync_avg_us=%lu flush_vsync_min_us=%lu flush_vsync_max_us=%lu flush_cross_vsync=%lu "
                 "wait=%lu wait_avg_us=%lu wait_max_us=%lu vsync=%lu vsync_avg_us=%lu vsync_min_us=%lu vsync_max_us=%lu jitter_avg_us=%lu jitter_max_us=%lu "
                 "inv=%lu inv_kpx=%llu inv_max_px=%lu inv_max_area=%ld,%ld,%ld,%ld inv_full=%lu inv_pending_peak=%lu inv_pending_peak_kpx=%lu inv_pending_peak_area=%ld,%ld,%ld,%ld inv_over32_windows=%lu",
                 (unsigned long)snap.handler_calls,
                 (unsigned long)snap.handler_avg_us,
                 (unsigned long)snap.handler_max_us,
                 (unsigned long)snap.handler_slow_16ms,
                 (unsigned long)snap.handler_slow_25ms,
                 (unsigned long)snap.lock_wait_avg_us,
                 (unsigned long)snap.lock_wait_max_us,
                 (unsigned long)snap.lvgl_handler_avg_us,
                 (unsigned long)snap.lvgl_handler_max_us,
                 (unsigned long)snap.lvgl_work_avg_us,
                 (unsigned long)snap.lvgl_work_max_us,
                 (unsigned long)snap.lvgl_work_slow_16ms,
                 (unsigned long)snap.lvgl_work_slow_25ms,
                 (unsigned long)snap.handler_wall_avg_us,
                 (unsigned long)snap.handler_wall_max_us,
                 (unsigned long)snap.lvgl_timer_avg_us,
                 (unsigned long)snap.lvgl_timer_max_us,
                 (unsigned long)snap.lvgl_timer_cb_calls,
                 (unsigned long)snap.lvgl_timer_cb_avg_us,
                 (unsigned long)snap.lvgl_timer_cb_max_us,
                 (unsigned long)snap.lvgl_timer_cb_slow_16ms,
                 (unsigned long)snap.lvgl_timer_cb_slow_25ms,
                 (void *)snap.lvgl_timer_cb_max,
                 (unsigned long)snap.manual_refresh_avg_us,
                 (unsigned long)snap.manual_refresh_max_us,
                 (unsigned long)snap.phase_wait_avg_us,
                 (unsigned long)snap.phase_wait_max_us,
                 (unsigned long)snap.render_slice_wait_calls,
                 (unsigned long)snap.render_slice_wait_avg_us,
                 (unsigned long)snap.render_slice_wait_max_us,
                 (unsigned long)snap.handler_refresh_max,
                 (unsigned long)snap.handler_render_max,
                 (unsigned long)snap.handler_flush_max,
                 (unsigned long)snap.handler_flush_event_max_us,
                 (unsigned long)snap.handler_flush_wait_max_us,
                 (unsigned long)snap.handler_phase_synced,
                 (unsigned long)snap.refr_cycles,
                 (unsigned long)(snap.refr_fps_x10 / 10),
                 (unsigned long)(snap.refr_fps_x10 % 10),
                 (unsigned long)snap.refr_avg_us,
                 (unsigned long)snap.refr_max_us,
                 (unsigned long)snap.render_cycles,
                 (unsigned long)snap.render_avg_us,
                 (unsigned long)snap.render_max_us,
                 (unsigned long)snap.render_since_vsync_avg_us,
                 (unsigned long)snap.render_since_vsync_min_us,
                 (unsigned long)snap.render_since_vsync_max_us,
                 (unsigned long)snap.render_cross_vsync,
                 (unsigned long)snap.render_fenced_cross_vsync,
                 (unsigned long)snap.render_unfenced_cross_vsync,
                 (unsigned long)snap.render_work_cycles,
                 (unsigned long)snap.render_work_avg_us,
                 (unsigned long)snap.render_work_max_us,
                 (unsigned long)snap.render_work_cross_vsync,
                 (unsigned long)snap.flush_calls,
                 (unsigned long)snap.flush_full_calls,
                 (unsigned long long)snap.flush_kpx,
                 (unsigned long)snap.flush_avg_us,
                 (unsigned long)snap.flush_max_us,
                 (unsigned long)snap.flush_max_pixels,
                 (long)snap.flush_max_area.x1,
                 (long)snap.flush_max_area.y1,
                 (long)snap.flush_max_area.x2,
                 (long)snap.flush_max_area.y2,
                 (unsigned long)snap.flush_since_vsync_avg_us,
                 (unsigned long)snap.flush_since_vsync_min_us,
                 (unsigned long)snap.flush_since_vsync_max_us,
                 (unsigned long)snap.flush_cross_vsync,
                 (unsigned long)snap.flush_waits,
                 (unsigned long)snap.flush_wait_avg_us,
                 (unsigned long)snap.flush_wait_max_us,
                 (unsigned long)snap.vsync_count,
                 (unsigned long)snap.vsync_avg_us,
                 (unsigned long)snap.vsync_min_us,
                 (unsigned long)snap.vsync_max_us,
                 (unsigned long)snap.vsync_jitter_avg_us,
                 (unsigned long)snap.vsync_jitter_max_us,
                 (unsigned long)snap.inv_calls,
                 (unsigned long long)snap.inv_kpx,
                 (unsigned long)snap.inv_max_pixels,
                 (long)snap.inv_max_area.x1,
                 (long)snap.inv_max_area.y1,
                 (long)snap.inv_max_area.x2,
                 (long)snap.inv_max_area.y2,
                 (unsigned long)snap.inv_full_calls,
                 (unsigned long)snap.inv_pending_peak_calls,
                 (unsigned long)snap.inv_pending_peak_kpx,
                 (long)snap.inv_pending_peak_area.x1,
                 (long)snap.inv_pending_peak_area.y1,
                 (long)snap.inv_pending_peak_area.x2,
                 (long)snap.inv_pending_peak_area.y2,
                 (unsigned long)snap.inv_pressure_windows);
        if (snap.flush_cross_vsync > 0) {
            ESP_LOGW(TAG,
                     "vsync-risk flush_cross=%lu start_since_us=%lu elapsed_us=%lu recent=\"%s\"",
                     (unsigned long)snap.flush_cross_vsync,
                     (unsigned long)snap.flush_cross_start_since_vsync_us,
                     (unsigned long)snap.flush_cross_elapsed_us,
                     snap.flush_cross_recent[0] ? snap.flush_cross_recent : "unavailable");
        }
    }
}

static void qmsd_gui_render_stats_log_task_start(void)
{
#if !QMSD_GUI_RENDER_TELEMETRY_ENABLED
    return;
#endif
    if (s_render_stats_log_task_handle || s_render_stats_log_queue) {
        return;
    }
    s_render_stats_log_queue = xQueueCreate(1, sizeof(qmsd_gui_render_stats_log_snapshot_t));
    if (!s_render_stats_log_queue) {
        ESP_LOGW(TAG, "failed to create GUI stats log queue");
        return;
    }
    esp_err_t err = qmsd_thread_create(qmsd_gui_render_stats_log_task,
                                       "gui-perf-log",
                                       QMSD_GUI_STATS_LOG_TASK_STACK,
                                       NULL,
                                       QMSD_GUI_STATS_LOG_TASK_PRIORITY,
                                       &s_render_stats_log_task_handle,
                                       QMSD_GUI_STATS_LOG_TASK_CORE,
                                       false);
    if (err != ESP_OK) {
        vQueueDelete(s_render_stats_log_queue);
        s_render_stats_log_queue = NULL;
        s_render_stats_log_task_handle = NULL;
        ESP_LOGW(TAG, "failed to create GUI stats log task");
    }
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
    if (timestamp_us <= 0) {
        qmsd_gui_notify_vsync_from_isr();
        return;
    }

    portENTER_CRITICAL_ISR(&s_vsync_stats_mux);

    int64_t previous_vsync_us = s_vsync_stats.last_us;
    uint32_t previous_interval_us = s_vsync_stats.last_interval_us;
    s_vsync_stats.last_us = timestamp_us;

    if (previous_vsync_us > 0 && timestamp_us > previous_vsync_us) {
        uint32_t interval_us = (uint32_t)(timestamp_us - previous_vsync_us);
        s_vsync_stats.last_interval_us = interval_us;

        if (s_render_stats_active) {
            s_vsync_stats.interval_count++;
            s_vsync_stats.interval_total_us += interval_us;
            if (s_vsync_stats.interval_min_us == 0 ||
                interval_us < s_vsync_stats.interval_min_us) {
                s_vsync_stats.interval_min_us = interval_us;
            }
            if (interval_us > s_vsync_stats.interval_max_us) {
                s_vsync_stats.interval_max_us = interval_us;
            }

            if (previous_interval_us > 0) {
                uint32_t jitter_us = interval_us > previous_interval_us
                                         ? interval_us - previous_interval_us
                                         : previous_interval_us - interval_us;
                s_vsync_stats.jitter_count++;
                s_vsync_stats.jitter_total_us += jitter_us;
                if (jitter_us > s_vsync_stats.jitter_max_us) {
                    s_vsync_stats.jitter_max_us = jitter_us;
                }
            }
        }
    }

    if (s_render_stats_active) {
        s_vsync_stats.count++;
    }

    portEXIT_CRITICAL_ISR(&s_vsync_stats_mux);

    qmsd_gui_notify_vsync_from_isr();
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

static void qmsd_gui_vsync_phase_snapshot(int64_t *last_us, uint32_t *interval_us)
{
    int64_t last = 0;
    uint32_t interval = 0;

    portENTER_CRITICAL(&s_vsync_stats_mux);
    last = s_vsync_stats.last_us;
    interval = s_vsync_stats.last_interval_us;
    portEXIT_CRITICAL(&s_vsync_stats_mux);

    if (last_us) {
        *last_us = last;
    }
    if (interval_us) {
        *interval_us = interval;
    }
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

static void qmsd_gui_render_stats_note_flush_cross(int64_t finish_us)
{
    int64_t start_us = s_render_stats.flush_event_start_us;
    int64_t start_vsync_us = s_render_stats.flush_event_start_vsync_us;
    if (start_us <= 0 || start_vsync_us <= 0 || finish_us < start_us) {
        return;
    }

    uint32_t elapsed_us = (uint32_t)(finish_us - start_us);
    if (elapsed_us < s_render_stats.flush_cross_elapsed_us) {
        return;
    }

    s_render_stats.flush_cross_elapsed_us = elapsed_us;
    s_render_stats.flush_cross_start_since_vsync_us =
        start_us >= start_vsync_us ? (uint32_t)(start_us - start_vsync_us) : 0;

    const char *recent = dashboard_widget_inv_trace_recent_summary
                             ? dashboard_widget_inv_trace_recent_summary()
                             : NULL;
    if (!recent || recent[0] == '\0') {
        recent = "unavailable";
    }
    (void)snprintf(s_render_stats.flush_cross_recent,
                   sizeof(s_render_stats.flush_cross_recent),
                   "%s",
                   recent);
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

static void qmsd_gui_direct_render_record_invalidation(const lv_area_t *area)
{
    uint32_t pixels = qmsd_gui_area_pixels(area);

    s_direct_render_state.inv_pending_calls++;
    s_direct_render_state.inv_pending_pixels += pixels;
    qmsd_gui_area_union_into(&s_direct_render_state.inv_pending_area,
                             &s_direct_render_state.inv_pending_area_valid,
                             area);
}

static bool qmsd_gui_direct_render_slice_fence_needed(void)
{
    if (s_direct_render_state.inv_pending_calls >= QMSD_GUI_DIRECT_RENDER_SLICE_FENCE_INV_AREAS) {
        return true;
    }
    if (s_direct_render_state.inv_pending_pixels >= QMSD_GUI_DIRECT_RENDER_SLICE_FENCE_INV_PIXELS) {
        return true;
    }
    if (s_direct_render_state.inv_pending_area_valid &&
        qmsd_gui_area_pixels(&s_direct_render_state.inv_pending_area) >=
            QMSD_GUI_DIRECT_RENDER_SLICE_FENCE_UNION_PIXELS) {
        return true;
    }
    return false;
}

static void qmsd_gui_direct_render_reset_pending_invalidations(void)
{
    s_direct_render_state.inv_pending_calls = 0;
    s_direct_render_state.inv_pending_pixels = 0;
    memset(&s_direct_render_state.inv_pending_area, 0, sizeof(s_direct_render_state.inv_pending_area));
    s_direct_render_state.inv_pending_area_valid = false;
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
    lv_event_code_t code = lv_event_get_code(event);
    int64_t now_us = -1;
    bool stats_enabled = qmsd_gui_render_stats_enabled();
#define QMSD_GUI_DISPLAY_EVENT_NOW_US() \
    ((now_us >= 0) ? now_us : (now_us = esp_timer_get_time()))

    switch (code) {
    case LV_EVENT_INVALIDATE_AREA:
        qmsd_gui_direct_render_record_invalidation(lv_event_get_invalidated_area(event));
        if (stats_enabled) {
            qmsd_gui_render_stats_record_invalidation((lv_display_t *)lv_event_get_target(event),
                                                      lv_event_get_invalidated_area(event));
        }
        break;
    case LV_EVENT_REFR_START:
        if (stats_enabled) {
            s_render_stats.refr_start_us = QMSD_GUI_DISPLAY_EVENT_NOW_US();
        }
        break;
    case LV_EVENT_REFR_READY:
        if (stats_enabled) {
            int64_t event_now_us = QMSD_GUI_DISPLAY_EVENT_NOW_US();
            s_handler_refr_cycles++;
            qmsd_gui_render_stats_record_elapsed(&s_render_stats.refr_cycles,
                                                 &s_render_stats.refr_total_us,
                                                 &s_render_stats.refr_max_us,
                                                 s_render_stats.refr_start_us,
                                                 event_now_us);
            s_render_stats.refr_start_us = 0;
            qmsd_gui_render_stats_reset_pending_invalidations();
        }
        qmsd_gui_direct_render_reset_pending_invalidations();
        break;
    case LV_EVENT_RENDER_START:
        s_direct_render_state.render_active = true;
        if (stats_enabled) {
            int64_t event_now_us = QMSD_GUI_DISPLAY_EVENT_NOW_US();
            s_render_stats.render_start_us = event_now_us;
            s_render_stats.render_start_vsync_us = qmsd_gui_last_vsync_us();
            s_render_stats.render_slice_fenced = false;
            qmsd_gui_render_stats_start_render_work(event_now_us);
            qmsd_gui_render_stats_record_since_vsync(&s_render_stats.render_since_vsync_count,
                                                     &s_render_stats.render_since_vsync_total_us,
                                                     &s_render_stats.render_since_vsync_min_us,
                                                     &s_render_stats.render_since_vsync_max_us,
                                                     event_now_us,
                                                     s_render_stats.render_start_vsync_us);
        }
        break;
    case LV_EVENT_RENDER_READY:
        if (stats_enabled) {
            int64_t event_now_us = QMSD_GUI_DISPLAY_EVENT_NOW_US();
            s_handler_render_cycles++;
            qmsd_gui_render_stats_finish_render_work(event_now_us);
            qmsd_gui_render_stats_record_elapsed(&s_render_stats.render_cycles,
                                                 &s_render_stats.render_total_us,
                                                 &s_render_stats.render_max_us,
                                                 s_render_stats.render_start_us,
                                                 event_now_us);
            if (s_render_stats.render_start_vsync_us > 0 &&
                qmsd_gui_last_vsync_us() > s_render_stats.render_start_vsync_us) {
                s_render_stats.render_cross_vsync++;
                if (s_render_stats.render_slice_fenced) {
                    s_render_stats.render_fenced_cross_vsync++;
                } else {
                    s_render_stats.render_unfenced_cross_vsync++;
                }
            }
            s_render_stats.render_start_us = 0;
            s_render_stats.render_start_vsync_us = 0;
            s_render_stats.render_slice_fenced = false;
        }
        s_direct_render_state.render_active = false;
        break;
    case LV_EVENT_FLUSH_START:
        s_direct_render_state.flush_event_start_us = QMSD_GUI_DISPLAY_EVENT_NOW_US();
        if (stats_enabled) {
            int64_t event_now_us = QMSD_GUI_DISPLAY_EVENT_NOW_US();
            qmsd_gui_render_stats_finish_render_work(event_now_us);
            s_render_stats.flush_event_start_us = event_now_us;
            s_render_stats.flush_event_start_vsync_us = qmsd_gui_last_vsync_us();
            qmsd_gui_render_stats_record_since_vsync(&s_render_stats.flush_since_vsync_count,
                                                     &s_render_stats.flush_since_vsync_total_us,
                                                     &s_render_stats.flush_since_vsync_min_us,
                                                     &s_render_stats.flush_since_vsync_max_us,
                                                     event_now_us,
                                                     s_render_stats.flush_event_start_vsync_us);
        }
        break;
    case LV_EVENT_FLUSH_FINISH: {
        uint32_t flush_event_elapsed_us = 0;
        if (s_direct_render_state.flush_event_start_us > 0 &&
            QMSD_GUI_DISPLAY_EVENT_NOW_US() >= s_direct_render_state.flush_event_start_us) {
            flush_event_elapsed_us =
                (uint32_t)(QMSD_GUI_DISPLAY_EVENT_NOW_US() - s_direct_render_state.flush_event_start_us);
        }
        s_direct_render_state.flush_event_start_us = 0;
        if (stats_enabled) {
            s_handler_flushes++;
            if (s_render_stats.flush_event_start_us > 0 &&
                s_render_stats.flush_event_start_vsync_us > 0 &&
                qmsd_gui_last_vsync_us() > s_render_stats.flush_event_start_vsync_us) {
                s_render_stats.flush_cross_vsync++;
                qmsd_gui_render_stats_note_flush_cross(QMSD_GUI_DISPLAY_EVENT_NOW_US());
            }
            if (s_render_stats.flush_event_start_us > 0 &&
                QMSD_GUI_DISPLAY_EVENT_NOW_US() >= s_render_stats.flush_event_start_us) {
                flush_event_elapsed_us =
                    (uint32_t)(QMSD_GUI_DISPLAY_EVENT_NOW_US() - s_render_stats.flush_event_start_us);
            }
            s_handler_flush_event_total_us += flush_event_elapsed_us;
            s_render_stats.flush_event_start_us = 0;
            s_render_stats.flush_event_start_vsync_us = 0;
        }
        if (s_direct_render_state.render_active) {
            if (!s_handler_last_flush_was_last &&
                (qmsd_gui_direct_render_slice_fence_needed() ||
                 flush_event_elapsed_us >= QMSD_GUI_DIRECT_RENDER_SLICE_FENCE_FLUSH_US)) {
                int64_t slice_wait_start_us = esp_timer_get_time();
                if (qmsd_gui_direct_render_slice_fence()) {
                    uint32_t slice_wait_us = (uint32_t)(esp_timer_get_time() - slice_wait_start_us);
                    if (stats_enabled) {
                        s_render_stats.render_slice_fenced = true;
                    }
                    s_handler_render_slice_waits++;
                    s_handler_render_slice_wait_total_us += slice_wait_us;
                    if (slice_wait_us > s_handler_render_slice_wait_max_us) {
                        s_handler_render_slice_wait_max_us = slice_wait_us;
                    }
                    now_us = esp_timer_get_time();
                }
            }
            if (stats_enabled) {
                qmsd_gui_render_stats_start_render_work(QMSD_GUI_DISPLAY_EVENT_NOW_US());
            }
        }
        break;
    }
    case LV_EVENT_FLUSH_WAIT_START:
        if (stats_enabled) {
            s_render_stats.flush_wait_start_us = QMSD_GUI_DISPLAY_EVENT_NOW_US();
        }
        break;
    case LV_EVENT_FLUSH_WAIT_FINISH:
        if (stats_enabled) {
            int64_t event_now_us = QMSD_GUI_DISPLAY_EVENT_NOW_US();
            if (s_render_stats.flush_wait_start_us > 0 && event_now_us >= s_render_stats.flush_wait_start_us) {
                s_handler_flush_wait_total_us += (uint32_t)(event_now_us - s_render_stats.flush_wait_start_us);
            }
            qmsd_gui_render_stats_record_elapsed(&s_render_stats.flush_waits,
                                                 &s_render_stats.flush_wait_total_us,
                                                 &s_render_stats.flush_wait_max_us,
                                                 s_render_stats.flush_wait_start_us,
                                                 event_now_us);
            s_render_stats.flush_wait_start_us = 0;
        }
        break;
    default:
        break;
    }
#undef QMSD_GUI_DISPLAY_EVENT_NOW_US
}

static void qmsd_gui_render_stats_record_handler(uint32_t elapsed_us,
                                                 uint32_t handler_wall_us,
                                                 uint32_t lock_wait_us,
                                                 uint32_t lvgl_handler_us,
                                                 uint32_t timer_handler_us,
                                                 uint32_t manual_refresh_us,
                                                 uint32_t phase_wait_us,
                                                 uint32_t render_slice_waits,
                                                 uint32_t render_slice_wait_total_us,
                                                 uint32_t render_slice_wait_max_us,
                                                 uint32_t flush_event_us,
                                                 uint32_t flush_wait_us,
                                                 uint32_t handler_refresh_cycles,
                                                 uint32_t handler_render_cycles,
                                                 uint32_t handler_flushes,
                                                 bool phase_synced,
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
    s_render_stats.handler_wall_calls++;
    s_render_stats.handler_wall_total_us += handler_wall_us;
    if (handler_wall_us > s_render_stats.handler_wall_max_us) {
        s_render_stats.handler_wall_max_us = handler_wall_us;
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

        if (timer_handler_us > 0) {
            s_render_stats.lvgl_timer_calls++;
            s_render_stats.lvgl_timer_total_us += timer_handler_us;
            if (timer_handler_us > s_render_stats.lvgl_timer_max_us) {
                s_render_stats.lvgl_timer_max_us = timer_handler_us;
            }
        }

        if (manual_refresh_us > 0) {
            s_render_stats.manual_refresh_calls++;
            s_render_stats.manual_refresh_total_us += manual_refresh_us;
            if (manual_refresh_us > s_render_stats.manual_refresh_max_us) {
                s_render_stats.manual_refresh_max_us = manual_refresh_us;
            }
        }

        if (phase_wait_us > 0) {
            s_render_stats.phase_wait_calls++;
            s_render_stats.phase_wait_total_us += phase_wait_us;
            if (phase_wait_us > s_render_stats.phase_wait_max_us) {
                s_render_stats.phase_wait_max_us = phase_wait_us;
            }
        }

        if (render_slice_waits > 0) {
            s_render_stats.render_slice_wait_calls += render_slice_waits;
            s_render_stats.render_slice_wait_total_us += render_slice_wait_total_us;
            if (render_slice_wait_max_us > s_render_stats.render_slice_wait_max_us) {
                s_render_stats.render_slice_wait_max_us = render_slice_wait_max_us;
            }
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

        if (handler_refresh_cycles > s_render_stats.handler_refresh_max) {
            s_render_stats.handler_refresh_max = handler_refresh_cycles;
        }
        if (handler_render_cycles > s_render_stats.handler_render_max) {
            s_render_stats.handler_render_max = handler_render_cycles;
        }
        if (handler_flushes > s_render_stats.handler_flush_max) {
            s_render_stats.handler_flush_max = handler_flushes;
        }
        if (flush_event_us > s_render_stats.handler_flush_event_max_us) {
            s_render_stats.handler_flush_event_max_us = flush_event_us;
        }
        if (flush_wait_us > s_render_stats.handler_flush_wait_max_us) {
            s_render_stats.handler_flush_wait_max_us = flush_wait_us;
        }
        if (phase_synced) {
            s_render_stats.handler_phase_synced++;
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
    uint32_t handler_wall_avg_us = s_render_stats.handler_wall_calls
                                       ? (uint32_t)(s_render_stats.handler_wall_total_us /
                                                    s_render_stats.handler_wall_calls)
                                       : 0;
    uint32_t lvgl_timer_avg_us = s_render_stats.lvgl_timer_calls
                                     ? (uint32_t)(s_render_stats.lvgl_timer_total_us /
                                                  s_render_stats.lvgl_timer_calls)
                                     : 0;
    uint32_t lvgl_timer_cb_avg_us = s_render_stats.lvgl_timer_cb_calls
                                        ? (uint32_t)(s_render_stats.lvgl_timer_cb_total_us /
                                                     s_render_stats.lvgl_timer_cb_calls)
                                        : 0;
    uint32_t manual_refresh_avg_us = s_render_stats.manual_refresh_calls
                                         ? (uint32_t)(s_render_stats.manual_refresh_total_us /
                                                      s_render_stats.manual_refresh_calls)
                                         : 0;
    uint32_t phase_wait_avg_us = s_render_stats.phase_wait_calls
                                     ? (uint32_t)(s_render_stats.phase_wait_total_us /
                                                  s_render_stats.phase_wait_calls)
                                     : 0;
    uint32_t render_slice_wait_avg_us = s_render_stats.render_slice_wait_calls
                                            ? (uint32_t)(s_render_stats.render_slice_wait_total_us /
                                                         s_render_stats.render_slice_wait_calls)
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
    qmsd_gui_render_stats_log_snapshot_t snap = {
        .handler_calls = s_render_stats.handler_calls,
        .handler_avg_us = handler_avg_us,
        .handler_max_us = s_render_stats.handler_max_us,
        .handler_slow_16ms = s_render_stats.handler_slow_16ms,
        .handler_slow_25ms = s_render_stats.handler_slow_25ms,
        .lock_wait_avg_us = lock_wait_avg_us,
        .lock_wait_max_us = s_render_stats.lock_wait_max_us,
        .lvgl_handler_avg_us = lvgl_handler_avg_us,
        .lvgl_handler_max_us = s_render_stats.lvgl_handler_max_us,
        .lvgl_work_avg_us = lvgl_work_avg_us,
        .lvgl_work_max_us = s_render_stats.lvgl_work_max_us,
        .lvgl_work_slow_16ms = s_render_stats.lvgl_work_slow_16ms,
        .lvgl_work_slow_25ms = s_render_stats.lvgl_work_slow_25ms,
        .handler_wall_avg_us = handler_wall_avg_us,
        .handler_wall_max_us = s_render_stats.handler_wall_max_us,
        .lvgl_timer_avg_us = lvgl_timer_avg_us,
        .lvgl_timer_max_us = s_render_stats.lvgl_timer_max_us,
        .lvgl_timer_cb_calls = s_render_stats.lvgl_timer_cb_calls,
        .lvgl_timer_cb_avg_us = lvgl_timer_cb_avg_us,
        .lvgl_timer_cb_max_us = s_render_stats.lvgl_timer_cb_max_us,
        .lvgl_timer_cb_slow_16ms = s_render_stats.lvgl_timer_cb_slow_16ms,
        .lvgl_timer_cb_slow_25ms = s_render_stats.lvgl_timer_cb_slow_25ms,
        .lvgl_timer_cb_max = s_render_stats.lvgl_timer_cb_max,
        .manual_refresh_avg_us = manual_refresh_avg_us,
        .manual_refresh_max_us = s_render_stats.manual_refresh_max_us,
        .phase_wait_avg_us = phase_wait_avg_us,
        .phase_wait_max_us = s_render_stats.phase_wait_max_us,
        .render_slice_wait_calls = s_render_stats.render_slice_wait_calls,
        .render_slice_wait_avg_us = render_slice_wait_avg_us,
        .render_slice_wait_max_us = s_render_stats.render_slice_wait_max_us,
        .handler_refresh_max = s_render_stats.handler_refresh_max,
        .handler_render_max = s_render_stats.handler_render_max,
        .handler_flush_max = s_render_stats.handler_flush_max,
        .handler_flush_event_max_us = s_render_stats.handler_flush_event_max_us,
        .handler_flush_wait_max_us = s_render_stats.handler_flush_wait_max_us,
        .handler_phase_synced = s_render_stats.handler_phase_synced,
        .refr_cycles = s_render_stats.refr_cycles,
        .refr_fps_x10 = refr_fps_x10,
        .refr_avg_us = refr_avg_us,
        .refr_max_us = s_render_stats.refr_max_us,
        .render_cycles = s_render_stats.render_cycles,
        .render_avg_us = render_avg_us,
        .render_max_us = s_render_stats.render_max_us,
        .render_since_vsync_avg_us = render_since_vsync_avg_us,
        .render_since_vsync_min_us = s_render_stats.render_since_vsync_min_us,
        .render_since_vsync_max_us = s_render_stats.render_since_vsync_max_us,
        .render_cross_vsync = s_render_stats.render_cross_vsync,
        .render_fenced_cross_vsync = s_render_stats.render_fenced_cross_vsync,
        .render_unfenced_cross_vsync = s_render_stats.render_unfenced_cross_vsync,
        .render_work_cycles = s_render_stats.render_work_cycles,
        .render_work_avg_us = render_work_avg_us,
        .render_work_max_us = s_render_stats.render_work_max_us,
        .render_work_cross_vsync = s_render_stats.render_work_cross_vsync,
        .flush_calls = s_render_stats.flush_calls,
        .flush_full_calls = s_render_stats.flush_full_calls,
        .flush_kpx = s_render_stats.flush_pixels / 1000ULL,
        .flush_avg_us = flush_avg_us,
        .flush_max_us = s_render_stats.flush_max_us,
        .flush_max_pixels = s_render_stats.flush_max_pixels,
        .flush_max_area = s_render_stats.flush_max_area,
        .flush_since_vsync_avg_us = flush_since_vsync_avg_us,
        .flush_since_vsync_min_us = s_render_stats.flush_since_vsync_min_us,
        .flush_since_vsync_max_us = s_render_stats.flush_since_vsync_max_us,
        .flush_cross_vsync = s_render_stats.flush_cross_vsync,
        .flush_cross_start_since_vsync_us = s_render_stats.flush_cross_start_since_vsync_us,
        .flush_cross_elapsed_us = s_render_stats.flush_cross_elapsed_us,
        .flush_waits = s_render_stats.flush_waits,
        .flush_wait_avg_us = flush_wait_avg_us,
        .flush_wait_max_us = s_render_stats.flush_wait_max_us,
        .vsync_count = vsync.count,
        .vsync_avg_us = vsync_avg_us,
        .vsync_min_us = vsync.interval_min_us,
        .vsync_max_us = vsync.interval_max_us,
        .vsync_jitter_avg_us = vsync_jitter_avg_us,
        .vsync_jitter_max_us = vsync.jitter_max_us,
        .inv_calls = s_render_stats.inv_calls,
        .inv_kpx = s_render_stats.inv_pixels / 1000ULL,
        .inv_max_pixels = s_render_stats.inv_max_pixels,
        .inv_max_area = s_render_stats.inv_max_area,
        .inv_full_calls = s_render_stats.inv_full_calls,
        .inv_pending_peak_calls = s_render_stats.inv_pending_peak_calls,
        .inv_pending_peak_kpx = s_render_stats.inv_pending_peak_kpx,
        .inv_pending_peak_area = s_render_stats.inv_pending_peak_area,
        .inv_pressure_windows = s_render_stats.inv_pressure_windows,
    };
    memcpy(snap.flush_cross_recent,
           s_render_stats.flush_cross_recent,
           sizeof(snap.flush_cross_recent));
    if (s_render_stats_log_queue) {
        (void)xQueueOverwrite(s_render_stats_log_queue, &snap);
    }

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
    bool flush_is_last = lv_display_flush_is_last(display);
    s_handler_last_flush_was_last = flush_is_last;
    if (qmsd_gui_should_coalesce_direct_flush(display)) {
        qmsd_gui_direct_flush_area_add(area);
        if (!flush_is_last) {
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
    bool flush_is_last = lv_display_flush_is_last(display);
    s_handler_last_flush_was_last = flush_is_last;
    if (qmsd_gui_should_coalesce_direct_flush(display)) {
        qmsd_gui_direct_flush_area_add(area);
        if (!flush_is_last) {
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

static bool qmsd_gui_direct_manual_refresh_enabled(void)
{
    return s_direct_manual_refresh && s_lvgl_display;
}

static bool qmsd_gui_vsync_interval_valid(uint32_t interval_us)
{
    return interval_us >= 15000 && interval_us <= 25000;
}

static void qmsd_gui_delay_until_near_us(int64_t target_us)
{
    while (true) {
        int64_t now_us = esp_timer_get_time();
        int64_t remaining_us = target_us - now_us;
        if (remaining_us <= 1000) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static bool qmsd_gui_direct_render_pressure(uint32_t timer_handler_us)
{
    return timer_handler_us >= QMSD_GUI_DIRECT_RENDER_PRESSURE_TIMER_US ||
           s_direct_render_state.inv_pending_calls >= QMSD_GUI_DIRECT_RENDER_PRESSURE_INV_AREAS ||
           s_direct_render_state.inv_pending_pixels >= QMSD_GUI_DIRECT_RENDER_PRESSURE_INV_PIXELS;
}

static bool qmsd_gui_wait_for_direct_render_vsync(bool pressure)
{
    if (!qmsd_gui_direct_mode_phase_sync_enabled()) {
        return false;
    }

    uint32_t lead_us = pressure ? QMSD_GUI_DIRECT_RENDER_PRESSURE_LEAD_US
                                : QMSD_GUI_DIRECT_RENDER_LEAD_US;
    uint32_t min_remain_us = pressure ? QMSD_GUI_DIRECT_RENDER_PRESSURE_MIN_REMAIN_US
                                      : QMSD_GUI_DIRECT_RENDER_MIN_REMAIN_US;

    (void)ulTaskNotifyTake(pdTRUE, 0);

    for (uint8_t attempt = 0; attempt < 2; ++attempt) {
        int64_t last_vsync_us = 0;
        uint32_t interval_us = 0;
        qmsd_gui_vsync_phase_snapshot(&last_vsync_us, &interval_us);

        if (last_vsync_us <= 0) {
            if (ulTaskNotifyTake(pdTRUE,
                                 pdMS_TO_TICKS(QMSD_GUI_DIRECT_RENDER_VSYNC_WAIT_TIMEOUT_MS)) <= 0) {
                return false;
            }
            qmsd_gui_vsync_phase_snapshot(&last_vsync_us, &interval_us);
            continue;
        }

        if (last_vsync_us == s_gui_update_vsync_consumed_us) {
            if (ulTaskNotifyTake(pdTRUE,
                                 pdMS_TO_TICKS(QMSD_GUI_DIRECT_RENDER_VSYNC_WAIT_TIMEOUT_MS)) <= 0) {
                return false;
            }
            qmsd_gui_vsync_phase_snapshot(&last_vsync_us, &interval_us);
            continue;
        }

        if (!qmsd_gui_vsync_interval_valid(interval_us)) {
            interval_us = QMSD_GUI_DIRECT_RENDER_DEFAULT_INTERVAL_US;
        }

        int64_t now_us = esp_timer_get_time();
        int64_t next_vsync_us = last_vsync_us + (int64_t)interval_us;
        int64_t target_us = next_vsync_us - lead_us;
        if (target_us < last_vsync_us) {
            target_us = last_vsync_us;
        }

        if (now_us < target_us) {
            qmsd_gui_delay_until_near_us(target_us);
            s_gui_update_vsync_consumed_us = last_vsync_us;
            return true;
        }

        if (now_us < (next_vsync_us - (int64_t)min_remain_us)) {
            s_gui_update_vsync_consumed_us = last_vsync_us;
            return true;
        }

        int64_t wait_us = next_vsync_us - now_us;
        uint32_t wait_ms = wait_us > 0 ? (uint32_t)((wait_us + 1999) / 1000) : 1;
        if (wait_ms > QMSD_GUI_DIRECT_RENDER_VSYNC_WAIT_TIMEOUT_MS) {
            wait_ms = QMSD_GUI_DIRECT_RENDER_VSYNC_WAIT_TIMEOUT_MS;
        }
        wait_ms += 1;
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait_ms)) <= 0) {
            return false;
        }
    }

    return false;
}

static bool qmsd_gui_direct_render_slice_fence(void)
{
    if (!qmsd_gui_direct_mode_phase_sync_enabled()) {
        return false;
    }

    int64_t last_vsync_us = 0;
    uint32_t interval_us = 0;
    qmsd_gui_vsync_phase_snapshot(&last_vsync_us, &interval_us);
    if (last_vsync_us <= 0) {
        return false;
    }

    if (!qmsd_gui_vsync_interval_valid(interval_us)) {
        interval_us = QMSD_GUI_DIRECT_RENDER_DEFAULT_INTERVAL_US;
    }

    int64_t now_us = esp_timer_get_time();
    int64_t next_vsync_us = last_vsync_us + (int64_t)interval_us;
    if (now_us < (next_vsync_us - (int64_t)QMSD_GUI_DIRECT_RENDER_SLICE_MIN_REMAIN_US)) {
        return false;
    }

    (void)ulTaskNotifyTake(pdTRUE, 0);
    int64_t wait_us = next_vsync_us - now_us;
    uint32_t wait_ms = wait_us > 0 ? (uint32_t)((wait_us + 1999) / 1000) : 1;
    if (wait_ms > QMSD_GUI_DIRECT_RENDER_VSYNC_WAIT_TIMEOUT_MS) {
        wait_ms = QMSD_GUI_DIRECT_RENDER_VSYNC_WAIT_TIMEOUT_MS;
    }
    wait_ms += 1;
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait_ms)) <= 0) {
        return false;
    }

    qmsd_gui_vsync_phase_snapshot(&last_vsync_us, &interval_us);
    if (last_vsync_us <= 0) {
        return false;
    }
    if (!qmsd_gui_vsync_interval_valid(interval_us)) {
        interval_us = QMSD_GUI_DIRECT_RENDER_DEFAULT_INTERVAL_US;
    }

    int64_t target_us = last_vsync_us + (int64_t)interval_us -
                        (int64_t)QMSD_GUI_DIRECT_RENDER_PRESSURE_LEAD_US;
    if (target_us < last_vsync_us) {
        target_us = last_vsync_us;
    }
    qmsd_gui_delay_until_near_us(target_us);
    s_gui_update_vsync_consumed_us = last_vsync_us;
    return true;
}

static void gui_update_task(void* arg) {
    s_gui_update_task_handle = xTaskGetCurrentTaskHandle();
    while (1) {
        int64_t handler_start_us = esp_timer_get_time();
        uint32_t lock_wait_us = 0;
        uint32_t lvgl_handler_us = 0;
        uint32_t timer_handler_us = 0;
        uint32_t manual_refresh_us = 0;
        uint32_t phase_wait_us = 0;
        bool lock_taken = false;
        bool phase_synced = false;

        s_handler_flush_event_total_us = 0;
        s_handler_flush_wait_total_us = 0;
        s_handler_refr_cycles = 0;
        s_handler_render_cycles = 0;
        s_handler_flushes = 0;
        s_handler_render_slice_waits = 0;
        s_handler_render_slice_wait_total_us = 0;
        s_handler_render_slice_wait_max_us = 0;
        s_handler_last_flush_was_last = true;

        if (qmsd_gui_direct_manual_refresh_enabled()) {
            int64_t timer_lock_start_us = esp_timer_get_time();
            if (qmsd_gui_lock(portMAX_DELAY) == 0) {
                lock_taken = true;
                int64_t timer_lock_acquired_us = esp_timer_get_time();
                lock_wait_us += (uint32_t)(timer_lock_acquired_us - timer_lock_start_us);
                lv_timer_handler();
                int64_t timer_done_us = esp_timer_get_time();
                uint32_t timer_elapsed_us = (uint32_t)(timer_done_us - timer_lock_acquired_us);
                timer_handler_us += timer_elapsed_us;
                lvgl_handler_us += timer_elapsed_us;
                qmsd_gui_unlock();
            }

            int64_t phase_wait_start_us = esp_timer_get_time();
            phase_synced = qmsd_gui_wait_for_direct_render_vsync(
                qmsd_gui_direct_render_pressure(timer_handler_us));
            phase_wait_us += (uint32_t)(esp_timer_get_time() - phase_wait_start_us);

            int64_t refresh_lock_start_us = esp_timer_get_time();
            if (qmsd_gui_lock(portMAX_DELAY) == 0) {
                lock_taken = true;
                int64_t refresh_lock_acquired_us = esp_timer_get_time();
                lock_wait_us += (uint32_t)(refresh_lock_acquired_us - refresh_lock_start_us);
                lv_display_refr_timer(NULL);
                int64_t refresh_done_us = esp_timer_get_time();
                uint32_t refresh_elapsed_us = (uint32_t)(refresh_done_us - refresh_lock_acquired_us);
                manual_refresh_us += refresh_elapsed_us;
                lvgl_handler_us += refresh_elapsed_us;
                qmsd_gui_unlock();
            }
        } else {
            int64_t lock_acquired_us = 0;
            int64_t lvgl_done_us = handler_start_us;
            if (qmsd_gui_lock(portMAX_DELAY) == 0) {
                lock_taken = true;
                lock_acquired_us = esp_timer_get_time();
                lv_timer_handler();
                lvgl_done_us = esp_timer_get_time();
                qmsd_gui_unlock();
            }
            lock_wait_us = lock_taken ? (uint32_t)(lock_acquired_us - handler_start_us) : 0;
            lvgl_handler_us = lock_taken ? (uint32_t)(lvgl_done_us - lock_acquired_us) : 0;
            timer_handler_us = lvgl_handler_us;
        }

        uint32_t handler_wall_us = (uint32_t)(esp_timer_get_time() - handler_start_us);
        uint32_t handler_end = lock_taken ? (lock_wait_us + lvgl_handler_us) : handler_wall_us;
        uint32_t flush_event_us = lock_taken ? s_handler_flush_event_total_us : 0;
        uint32_t flush_wait_us = lock_taken ? s_handler_flush_wait_total_us : 0;
        uint32_t handler_refresh_cycles = lock_taken ? s_handler_refr_cycles : 0;
        uint32_t handler_render_cycles = lock_taken ? s_handler_render_cycles : 0;
        uint32_t handler_flushes = lock_taken ? s_handler_flushes : 0;
        s_handler_flush_event_total_us = 0;
        s_handler_flush_wait_total_us = 0;
        s_handler_refr_cycles = 0;
        s_handler_render_cycles = 0;
        s_handler_flushes = 0;
        qmsd_gui_render_stats_record_handler(handler_end,
                                             handler_wall_us,
                                             lock_taken ? lock_wait_us : handler_end,
                                             lvgl_handler_us,
                                             timer_handler_us,
                                             manual_refresh_us,
                                             phase_wait_us,
                                             s_handler_render_slice_waits,
                                             s_handler_render_slice_wait_total_us,
                                             s_handler_render_slice_wait_max_us,
                                             flush_event_us,
                                             flush_wait_us,
                                             handler_refresh_cycles,
                                             handler_render_cycles,
                                             handler_flushes,
                                             phase_synced,
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
    qmsd_gui_render_stats_log_task_start();

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
    s_lvgl_display = display;
    if (lvgl_config->flags.direct_mode) {
        lv_display_delete_refr_timer(display);
        s_direct_manual_refresh = true;
        ESP_LOGI(TAG, "direct-mode display refresh timer decoupled from lv_timer_handler");
    }

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
