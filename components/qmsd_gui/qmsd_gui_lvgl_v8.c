#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "freertos/semphr.h"
#include "qmsd_gui.h"
#include "qmsd_utils.h"
#include "lvgl.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef CONFIG_QMSD_GUI_LVGL_V8

// Forward declare to avoid adding a build-time dependency edge from qmsd_gui -> qmsd_touch.
uint32_t touch_samples_waiting(void);

static qmsd_gui_config_t* g_lvgl_config;
static QueueHandle_t g_image_queue;
static SemaphoreHandle_t g_gui_semaphore = NULL;

typedef struct {
    int offsetx1;
    int offsetx2;
    int offsety1;
    int offsety2;
    lv_color_t* color;
    lv_disp_drv_t* drv;
} show_data_t;

// Optional performance tracing (enabled for instrumentation; set to 0 to disable)
#ifndef QMSD_GUI_PERF_TRACE
#define QMSD_GUI_PERF_TRACE 0
#endif

#if QMSD_GUI_PERF_TRACE
static const char *PERF_TAG = "UI_PERF";
static uint64_t perf_last_report_us = 0;
static uint32_t perf_frames = 0;
static uint64_t perf_handler_acc_us = 0;
static uint32_t perf_handler_max_us = 0;
// Flush/queue metrics
static volatile uint32_t perf_flush_count = 0;
static volatile uint64_t perf_flush_px = 0;
static volatile uint64_t perf_flush_acc_us = 0;
static volatile uint32_t perf_flush_max_us = 0;
static volatile uint64_t perf_q_block_acc_us = 0;
static volatile uint32_t perf_q_block_max_us = 0;
static uint64_t perf_last_rt_dump_us = 0; // rate limit runtime stats dumps
static volatile uint8_t perf_dump_inflight = 0; // ensure one dump task at a time

typedef struct {
    uint32_t fps_px;
    uint32_t handler_avg_us;
    uint32_t flush_avg_us;
} perf_dump_args_t;

static void ui_perf_dump_task(void *arg)
{
    perf_dump_args_t args = {0};
    if (arg) { args = *(perf_dump_args_t *)arg; free(arg); }
    // Take two snapshots and compute delta CPU per task
    UBaseType_t nTasks = uxTaskGetNumberOfTasks();
    TaskStatus_t *s1 = (TaskStatus_t *)heap_caps_malloc(nTasks * sizeof(TaskStatus_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s1) s1 = (TaskStatus_t *)heap_caps_malloc(nTasks * sizeof(TaskStatus_t), MALLOC_CAP_8BIT);
    if (!s1) { perf_dump_inflight = 0; vTaskDelete(NULL); return; }
    uint32_t total1 = 0;
    UBaseType_t n1 = uxTaskGetSystemState(s1, nTasks, &total1);
    (void)n1; // quiet unused if zero tasks change
    vTaskDelay(pdMS_TO_TICKS(500));
    TaskStatus_t *s2 = (TaskStatus_t *)heap_caps_malloc(nTasks * sizeof(TaskStatus_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s2) s2 = (TaskStatus_t *)heap_caps_malloc(nTasks * sizeof(TaskStatus_t), MALLOC_CAP_8BIT);
    if (!s2) { free(s1); perf_dump_inflight = 0; vTaskDelete(NULL); return; }
    uint32_t total2 = 0;
    UBaseType_t n2 = uxTaskGetSystemState(s2, nTasks, &total2);
    (void)n2;
    uint32_t delta_total = (total2 > total1) ? (total2 - total1) : 0;
    // Build simple top list (selection for top 6)
    #define PERF_TOPN 6
    struct { const char *name; uint32_t delta; } top[PERF_TOPN];
    for (int i = 0; i < PERF_TOPN; ++i) { top[i].name = NULL; top[i].delta = 0; }
    for (UBaseType_t i = 0; i < nTasks; ++i) {
        TaskStatus_t *t2 = &s2[i];
        // Find match by handle
        uint32_t rt1 = 0;
        for (UBaseType_t j = 0; j < nTasks; ++j) {
            if (s1[j].xHandle == t2->xHandle) { rt1 = s1[j].ulRunTimeCounter; break; }
        }
        uint32_t d = (t2->ulRunTimeCounter > rt1) ? (t2->ulRunTimeCounter - rt1) : 0;
        // Insert into top array if large
        int min_idx = 0; uint32_t min_val = top[0].delta;
        for (int k = 1; k < PERF_TOPN; ++k) { if (top[k].delta < min_val) { min_val = top[k].delta; min_idx = k; } }
        if (d > min_val) { top[min_idx].delta = d; top[min_idx].name = t2->pcTaskName; }
    }
    ESP_LOGI(PERF_TAG, "==== CPU Delta (0.5s) on fps_px=%lu handler_avg=%luus flush_avg=%luus ====",
             (unsigned long)args.fps_px, (unsigned long)args.handler_avg_us, (unsigned long)args.flush_avg_us);
    if (delta_total == 0) delta_total = 1;
    for (int i = 0; i < PERF_TOPN; ++i) {
        if (top[i].name) {
            uint32_t pct = (uint32_t)((top[i].delta * 100ULL) / delta_total);
            ESP_LOGI(PERF_TAG, "  %s: %lu%%", top[i].name, (unsigned long)pct);
        }
    }
    free(s1);
    free(s2);
    perf_dump_inflight = 0;
    vTaskDelete(NULL);
}

static void perf_maybe_dump_runtime_stats(uint32_t fps,
                                          uint32_t handler_avg_us,
                                          uint32_t flush_avg_us)
{
    // Trigger when fps is very low or costs are very high, rate-limited to every 5s
    uint64_t now = esp_timer_get_time();
    if (now - perf_last_rt_dump_us < 5000000ULL) return;
    bool trigger = false;
    if (fps <= 15) trigger = true;
    if (handler_avg_us >= 8000) trigger = true;
    if (flush_avg_us >= 20000) trigger = true;
    if (!trigger) return;

    if (perf_dump_inflight) return;
    perf_dump_inflight = 1;
    perf_dump_args_t *args = (perf_dump_args_t *)heap_caps_malloc(sizeof(perf_dump_args_t), MALLOC_CAP_8BIT);
    if (!args) { perf_dump_inflight = 0; return; }
    args->fps_px = fps;
    args->handler_avg_us = handler_avg_us;
    args->flush_avg_us = flush_avg_us;
    // Low priority dump task
    xTaskCreatePinnedToCore(ui_perf_dump_task, "ui_perf_dump", 4096, args, 1, NULL, 1);
    perf_last_rt_dump_us = now;
}
#endif

static void refresh_task(void* arg) {
    (void)arg;
    show_data_t* show_data;
    for (;;) {
        if (xQueuePeek(g_image_queue, &show_data, portMAX_DELAY) == pdTRUE) {
            int offsetx1 = show_data->offsetx1;
            int w = show_data->offsetx2 - show_data->offsetx1 + 1;
            int offsety1 = show_data->offsety1;
            int h = show_data->offsety2 - show_data->offsety1 + 1;
            #if QMSD_GUI_PERF_TRACE
            uint64_t t0 = esp_timer_get_time();
            #endif
            g_lvgl_config->draw_bitmap(offsetx1, offsety1, w, h, (uint16_t*)show_data->color);
            #if QMSD_GUI_PERF_TRACE
            uint32_t dt_us = (uint32_t)(esp_timer_get_time() - t0);
            perf_flush_count++;
            perf_flush_px += (uint64_t)w * (uint64_t)h;
            perf_flush_acc_us += dt_us;
            if (dt_us > perf_flush_max_us) perf_flush_max_us = dt_us;
            #endif
            xQueueReceive(g_image_queue, &show_data, 0);
            free(show_data);
        }
    }
}

static void lvgl_task_refresh(lv_disp_drv_t* drv, const lv_area_t* area, lv_color_t* color_map) {
    show_data_t* show_data = (show_data_t*)calloc(1, sizeof(show_data_t));
    show_data->drv = drv;
    show_data->offsetx1 = area->x1;
    show_data->offsetx2 = area->x2;
    show_data->offsety1 = area->y1;
    show_data->offsety2 = area->y2;
    show_data->color = color_map;
    #if QMSD_GUI_PERF_TRACE
    uint64_t tq0 = esp_timer_get_time();
    #endif
    xQueueSend(g_image_queue, &show_data, portMAX_DELAY);
    #if QMSD_GUI_PERF_TRACE
    uint32_t q_block = (uint32_t)(esp_timer_get_time() - tq0);
    perf_q_block_acc_us += q_block;
    if (q_block > perf_q_block_max_us) perf_q_block_max_us = q_block;
    #endif
    lv_disp_flush_ready(drv);
}

static void lvgl_flush(lv_disp_drv_t* drv, const lv_area_t* area, lv_color_t* color_map) {
    g_lvgl_config->draw_bitmap(area->x1, area->y1, (uint16_t)(area->x2 - area->x1 + 1), (uint16_t)(area->y2 - area->y1 + 1), (uint16_t*)color_map);
    lv_disp_flush_ready(drv);
}

static void lvgl_tp_read(struct _lv_indev_drv_t* indev_drv, lv_indev_data_t* data) {
    uint8_t press = 0;
    uint16_t x, y;
    g_lvgl_config->touch_read(&press, &x, &y);
    if (press) {
        data->point.x = x;
        data->point.y = y;
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
    // If the touch task buffered multiple samples while the GUI task was busy,
    // ask LVGL to call read_cb again immediately to drain the backlog in a single
    // lv_task_handler() iteration.
    data->continue_reading = (touch_samples_waiting() > 0);
}

static void lvgl_encoder_read(struct _lv_indev_drv_t* indev_drv, lv_indev_data_t* data) {
    uint8_t press = 0;
    int16_t enc_diff = 0;
    g_lvgl_config->encoder_read(&press, &enc_diff);
    data->enc_diff = enc_diff;
    if (press) {
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

static void increase_lvgl_tick(void* arg) {
    lv_tick_inc(portTICK_PERIOD_MS);
}

static void gui_update_task(void* arg) {
    while (1) {
        uint32_t handler_start = lv_tick_get();
        #if QMSD_GUI_PERF_TRACE
        uint64_t t0 = esp_timer_get_time();
        #endif
        if (qmsd_gui_lock(portMAX_DELAY) == 0) {
            lv_task_handler();
            qmsd_gui_unlock();
        }

        uint32_t handler_end = lv_tick_elaps(handler_start);
        #if QMSD_GUI_PERF_TRACE
        uint32_t dt_us = (uint32_t)(esp_timer_get_time() - t0);
        perf_frames++;
        perf_handler_acc_us += dt_us;
        if (dt_us > perf_handler_max_us) perf_handler_max_us = dt_us;

        uint64_t now = esp_timer_get_time();
        if (perf_last_report_us == 0) perf_last_report_us = now;
        if ((now - perf_last_report_us) >= 1000000ULL) {
            uint32_t lps = perf_frames; // loops per second (lv_task_handler calls)
            uint32_t avg_us = (perf_frames > 0) ? (uint32_t)(perf_handler_acc_us / perf_frames) : 0;
            uint32_t max_us = perf_handler_max_us;
            // Snapshot additional metrics
            uint32_t f_cnt = perf_flush_count;  perf_flush_count = 0;
            uint64_t f_px  = perf_flush_px;     perf_flush_px = 0;
            uint64_t f_acc = perf_flush_acc_us; perf_flush_acc_us = 0;
            uint32_t f_max = perf_flush_max_us; perf_flush_max_us = 0;
            uint64_t qb_acc = perf_q_block_acc_us; perf_q_block_acc_us = 0;
            uint32_t qb_max = perf_q_block_max_us; perf_q_block_max_us = 0;
            uint32_t f_avg_us = (f_cnt > 0) ? (uint32_t)(f_acc / f_cnt) : 0;
            uint32_t qb_avg = (f_cnt > 0) ? (uint32_t)(qb_acc / f_cnt) : 0;
            // Derive an approximate display FPS from flushed pixels per second
            uint32_t frame_px = (g_lvgl_config) ? (uint32_t)(g_lvgl_config->width * g_lvgl_config->hight) : 0;
            uint32_t fps_px_x10 = (frame_px > 0) ? (uint32_t)((f_px * 10ULL) / frame_px) : 0; // 1 decimal place
            uint32_t fps_px = fps_px_x10 / 10;
            uint32_t fps_px_frac = fps_px_x10 % 10;

            ESP_LOGI(PERF_TAG,
                     "fps_px=%lu.%lu lps=%lu handler_us avg=%lu max=%lu | flushes=%lu px=%llu flush_us avg=%lu max=%lu q_block_us avg=%lu max=%lu",
                     fps_px, fps_px_frac, lps, avg_us, max_us,
                     f_cnt, (unsigned long long)f_px, f_avg_us, f_max, qb_avg, qb_max);
            // Use pixel-based fps to trigger CPU dumps
            perf_maybe_dump_runtime_stats(fps_px, avg_us, f_avg_us);
            perf_last_report_us = now;
            perf_frames = 0;
            perf_handler_acc_us = 0;
            perf_handler_max_us = 0;
        }
        #endif
        if (handler_end > 50) {
            vTaskDelay(pdMS_TO_TICKS(3));
        } else if (handler_end > 25) {
            vTaskDelay(pdMS_TO_TICKS(6));
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void __attribute__((weak)) gui_user_init(void) {
}

void qmsd_gui_init(qmsd_gui_config_t* lvgl_config) {
    static lv_disp_draw_buf_t disp_buf;
    static lv_disp_drv_t disp_drv;

    static lv_indev_drv_t* indev_drv;

    g_lvgl_config = (qmsd_gui_config_t*)malloc(sizeof(qmsd_gui_config_t));
    memcpy(g_lvgl_config, lvgl_config, sizeof(qmsd_gui_config_t));

    g_gui_semaphore = xSemaphoreCreateMutex();

    lv_init();
    lv_disp_draw_buf_init(&disp_buf, lvgl_config->buffer[0], lvgl_config->buffer[1], lvgl_config->buffer_size >> 1);
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = lvgl_config->width;
    disp_drv.ver_res = lvgl_config->hight;
    disp_drv.draw_buf = &disp_buf;

    disp_drv.full_refresh = lvgl_config->flags.full_refresh;
    disp_drv.direct_mode = lvgl_config->flags.direct_mode;

    if (lvgl_config->refresh_task.en) {
        g_image_queue = xQueueCreate(1, sizeof(show_data_t*));
        qmsd_thread_create(refresh_task, "gui-refresh", lvgl_config->refresh_task.stack_size, NULL, lvgl_config->refresh_task.priority, NULL, lvgl_config->refresh_task.core,
                           lvgl_config->refresh_task.task_in_psram);
        disp_drv.flush_cb = lvgl_task_refresh;
    } else {
        disp_drv.flush_cb = lvgl_flush;
    }

    lv_disp_drv_register(&disp_drv);

    if (lvgl_config->touch_read) {
        indev_drv = (lv_indev_drv_t*)malloc(sizeof(lv_indev_drv_t));
        lv_indev_drv_init(indev_drv);
        indev_drv->type = LV_INDEV_TYPE_POINTER;
        indev_drv->read_cb = lvgl_tp_read;
        lv_indev_drv_register(indev_drv);
    }

    if (lvgl_config->encoder_read) {
        indev_drv = (lv_indev_drv_t*)malloc(sizeof(lv_indev_drv_t));
        lv_indev_drv_init(indev_drv);
        indev_drv->type = LV_INDEV_TYPE_ENCODER;
        indev_drv->read_cb = lvgl_encoder_read;
        lv_indev_drv_register(indev_drv);
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

#endif
