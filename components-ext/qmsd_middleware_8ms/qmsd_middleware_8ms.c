#include "qmsd_middleware_8ms.h"
#include "qmsd_ctrl.h"

#define MASK_UNSED(x) (void)(x)

void lv_obj_qmsd_set_cb(lv_obj_t *obj, lv_obj_qmsd_cb q_cb) {
    if (!obj) {
        return;
    }

    lv_obj_set_user_data(obj, (void *)(uintptr_t)q_cb);
}

void qmsd_screen_remove(const char *id) {
    MASK_UNSED(id);
}

void qmsd_obj_set_id(lv_obj_t* obj, const char *id) {
    MASK_UNSED(obj);
}

void qmsd_screen_register(lv_obj_t* obj,const char* id) {
    MASK_UNSED(obj);
    MASK_UNSED(id);
    lv_obj_qmsd_cb cb = (lv_obj_qmsd_cb)(uintptr_t)lv_obj_get_user_data(obj);
    if(cb) {
        cb(obj, LV_EVENT_READY, NULL);
    }
}

void qmsd_screen_list_init(int max) {
    MASK_UNSED(max);
}

void qmsd_screen_print() {

}

lv_obj_t* qmsd_search_screen(const char *id) {
    MASK_UNSED(id);
    return NULL;
}

lv_obj_t* qmsd_search_widget(const char *id) {
    MASK_UNSED(id);
    return NULL;
}

void qmsd_set_screen(__qmsd_get_screen get_screen) {

}
void qmsd_set_widget(__qmsd_get_widget get_widget) {

}
void qmsd_set_img(__qmsd_get_img get_img) {

}
void qmsd_set_font(__qmsd_get_font get_font) {

}
