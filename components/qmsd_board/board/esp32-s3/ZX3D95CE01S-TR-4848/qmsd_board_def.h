#pragma once

#define QMSD_SCREEN_WIDTH 480
#define QMSD_SCREEN_HIGHT 480

#define QMSD_SCREEN_BK_FREQ     11111

// 20 MHz is the BSP default. Keep a 16 MHz fallback for panels that show
// timing-sensitive shifting or tearing. To stress timing during validation,
// see the comments around .bounce_buffer_size_px and .double_fb in
// qmsd_screen_rgb.c.
#ifndef QMSD_SCREEN_20_MHZ_CLK
#define QMSD_SCREEN_20_MHZ_CLK 1
#endif

#if QMSD_SCREEN_20_MHZ_CLK
#define QMSD_RGB_CLK_FREQ           (20000000)
#define QMSD_RGB_HSYNC_PULSE_WIDTH  (48)
#define QMSD_RGB_HSYNC_BACK_PORCH   (40)
#define QMSD_RGB_HSYNC_FRONT_PORCH  (8)
#define QMSD_RGB_VSYNC_PULSE_WIDTH  (100)
#define QMSD_RGB_VSYNC_BACK_PORCH   (48)
#define QMSD_RGB_VSYNC_FRONT_PORCH  (8)
#else
#define QMSD_RGB_CLK_FREQ           (16000000)
#define QMSD_RGB_HSYNC_PULSE_WIDTH  (10)
#define QMSD_RGB_HSYNC_BACK_PORCH   (60)
#define QMSD_RGB_HSYNC_FRONT_PORCH  (8)
#define QMSD_RGB_VSYNC_PULSE_WIDTH  (10)
#define QMSD_RGB_VSYNC_BACK_PORCH   (40)
#define QMSD_RGB_VSYNC_FRONT_PORCH  (8)
#endif

#define QMSD_SCREEN_DIR_0       0
#define QMSD_SCREEN_DIR_90      (QMSD_SCREEN_DIR_0 ^ SCR_MIRROR_X ^ SCR_SWAP_XY)
#define QMSD_SCREEN_DIR_180     (QMSD_SCREEN_DIR_0 ^ SCR_MIRROR_X ^ SCR_MIRROR_Y)
#define QMSD_SCREEN_DIR_270     (QMSD_SCREEN_DIR_0 ^ SCR_MIRROR_Y ^ SCR_SWAP_XY)

#define QMSD_TOUCH_DIR_0        0
#define QMSD_TOUCH_DIR_90       (QMSD_TOUCH_DIR_0 ^ TOUCH_MIRROR_X ^ TOUCH_SWAP_XY)
#define QMSD_TOUCH_DIR_180      (QMSD_TOUCH_DIR_0 ^ TOUCH_MIRROR_X ^ TOUCH_MIRROR_Y)
#define QMSD_TOUCH_DIR_270      (QMSD_TOUCH_DIR_0 ^ TOUCH_MIRROR_Y ^ TOUCH_SWAP_XY)
