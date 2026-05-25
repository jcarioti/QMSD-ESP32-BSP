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

static const char *TAG = "QMSD_GUI";

static qmsd_gui_config_t* g_lvgl_config;
static QueueHandle_t g_image_queue;
static SemaphoreHandle_t g_gui_semaphore = NULL;

typedef struct {
    uint32_t handler_calls;
    uint64_t handler_total_us;
    uint32_t handler_max_us;
    uint32_t handler_slow_16ms;
    uint32_t handler_slow_25ms;
    uint32_t flush_calls;
    uint64_t flush_total_us;
    uint64_t flush_pixels;
    uint32_t flush_max_us;
    uint32_t flush_max_pixels;
    int64_t window_start_us;
} qmsd_gui_render_stats_t;

static qmsd_gui_render_stats_t s_render_stats;

static bool qmsd_gui_render_stats_enabled(void)
{
    return esp_log_level_get(TAG) >= ESP_LOG_INFO;
}

static void qmsd_gui_render_stats_record_flush(uint32_t pixels, uint32_t elapsed_us)
{
    s_render_stats.flush_calls++;
    s_render_stats.flush_total_us += elapsed_us;
    s_render_stats.flush_pixels += pixels;
    if (elapsed_us > s_render_stats.flush_max_us) {
        s_render_stats.flush_max_us = elapsed_us;
    }
    if (pixels > s_render_stats.flush_max_pixels) {
        s_render_stats.flush_max_pixels = pixels;
    }
}

static void qmsd_gui_render_stats_record_handler(uint32_t elapsed_us)
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
    ESP_LOGI(TAG,
             "render handlers=%lu avg_us=%lu max_us=%lu slow16=%lu slow25=%lu flushes=%lu flush_kpx=%llu flush_avg_us=%lu flush_max_us=%lu flush_max_px=%lu",
             (unsigned long)s_render_stats.handler_calls,
             (unsigned long)handler_avg_us,
             (unsigned long)s_render_stats.handler_max_us,
             (unsigned long)s_render_stats.handler_slow_16ms,
             (unsigned long)s_render_stats.handler_slow_25ms,
             (unsigned long)s_render_stats.flush_calls,
             (unsigned long long)(s_render_stats.flush_pixels / 1000ULL),
             (unsigned long)flush_avg_us,
             (unsigned long)s_render_stats.flush_max_us,
             (unsigned long)s_render_stats.flush_max_pixels);

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
            bool stats_enabled = qmsd_gui_render_stats_enabled();
            int64_t flush_start_us = stats_enabled ? esp_timer_get_time() : 0;
            g_lvgl_config->draw_bitmap(offsetx1, offsety1, w, h, (uint16_t*)show_data->color);
            if (stats_enabled) {
                qmsd_gui_render_stats_record_flush((uint32_t)(w * h),
                                                   (uint32_t)(esp_timer_get_time() - flush_start_us));
            }
            xQueueReceive(g_image_queue, &show_data, 0);
            free(show_data);
        }
    }
}

static void lvgl_task_refresh(lv_display_t* display, const lv_area_t* area, uint8_t* color_map) {
    show_data_t* show_data = (show_data_t*)calloc(1, sizeof(show_data_t));
    show_data->display = display;
    show_data->offsetx1 = area->x1;
    show_data->offsetx2 = area->x2;
    show_data->offsety1 = area->y1;
    show_data->offsety2 = area->y2;
    show_data->color = color_map;
    xQueueSend(g_image_queue, &show_data, portMAX_DELAY);
    lv_display_flush_ready(display);
}

static void lvgl_flush(lv_display_t* display, const lv_area_t* area, uint8_t* color_map) {
    uint16_t w = (uint16_t)(area->x2 - area->x1 + 1);
    uint16_t h = (uint16_t)(area->y2 - area->y1 + 1);
    bool stats_enabled = qmsd_gui_render_stats_enabled();
    int64_t flush_start_us = stats_enabled ? esp_timer_get_time() : 0;
    g_lvgl_config->draw_bitmap(area->x1, area->y1, w, h, (uint16_t*)color_map);
    if (stats_enabled) {
        qmsd_gui_render_stats_record_flush((uint32_t)w * (uint32_t)h,
                                           (uint32_t)(esp_timer_get_time() - flush_start_us));
    }
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

static void gui_update_task(void* arg) {
    while (1) {
        int64_t handler_start_us = esp_timer_get_time();
        if (qmsd_gui_lock(portMAX_DELAY) == 0) {
            lv_timer_handler();
            qmsd_gui_unlock();
        }

        uint32_t handler_end = (uint32_t)(esp_timer_get_time() - handler_start_us);
        qmsd_gui_render_stats_record_handler(handler_end);
        if (handler_end > 50000) {
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

    g_gui_semaphore = xSemaphoreCreateMutex();

    lv_init();
    lv_display_t *display = lv_display_create(lvgl_config->width, lvgl_config->hight);
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);

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
