/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "led_effects.h"

int parmlist_i[mMAX][8] = {
    {0, 0, 0, 0, 0, 0, 0, 0},      // mAM
    {200, 99, 20, 0, 0, 0, 0, 0},  // mStatic
    {300, 30, 0, 0, 0, 0, 0, 0},   // mListen (cala_listen_parm 会填充更多参数)
    {0, 18, 5, 0, 0, 0, 0, 0},     // mRainbow
    {50, 100, 200, 0, 0, 0, 0, 0}, // mProgressbar
    {0, 50, 0, 0, 0, 0, 0, 0},     // mBreath2c
    {0, 50, 0, 0, 0, 0, 0, 0},     // mBreathe
    {200, 5, 0, 0, 0, 0, 0, 0},    // mFlash
    {200, 100, 1, 50, 0, 0, 0, 0}, // mGlitch
};

double parmlist_d[mMAX][8] = {
    {0, 0, 0, 0, 0, 0, 0, 0},       // mAM
    {0, 0, 0, 0, 0, 0, 0, 0},       // mStatic (未使用)
    {0, 0, 0, 0, 0, 0.013333, 0.25, 471.238898},  // mListen (cala_listen_parm)
    {0, 0, 0, 0, 0, 0, 0, 0},       // mRainbow (未使用)
    {2.5, 0, 0, 0, 0, 0, 0, 0},     // mProgressbar (颜色平滑参数)
    {10, 50, 800, 0, 0, 0, 0, 0},   // mBreath2c
    {25, 50, 300, 0, 0, 0, 0, 0},   // mBreathe
    {0, 0, 0, 0, 0, 0, 0, 0},       // mFlash (未使用)
    {0, 0, 0, 0, 0, 0, 0, 0},       // mGlitch (未使用)
};

void app_main(void)
{
    QueueHandle_t msg_queue = init_leds_am(RMT_LED_STRIP_GPIO_NUM,LEDS_FLUSH_SPEED_MS);
    ledam_msg_t msg;
    int i = mStatic;

    while (1) {

        i = ( i >= mMAX ) ? mStatic : i;

        msg.type = i;

        memcpy(msg.parm_i, parmlist_i[i], sizeof(parmlist_i[i]));
        memcpy(msg.parm_d, parmlist_d[i], sizeof(parmlist_d[i]));
        
        xQueueSend(msg_queue, &(msg), (TickType_t)1);
        
        vTaskDelay(pdMS_TO_TICKS(10000));
        i++;
        msg.type = (msg.type == mListen) ? mBreath2c : mListen;
    }
}
