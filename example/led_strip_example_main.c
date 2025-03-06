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

int parmlist_i[kLEDC_MAX][8] = {
    {0, 0, 0, 0, 0, 0, 0, 0},      // mAM
    {200, 99, 20, 0, 0, 0, 0, 0},  // mStatic
    {300, 30, 30, 0, 0, 0, 0, 0},   // mListen (cala_listen_parm 会填充更多参数)
    {0, 99, 30, 18, 5, 0, 0, 0},   // mRainbow
    {50, 100, 200, 30, 0, 0, 0, 0},// mProgressbar
    {0, 50, 99, 30, 0, 0, 0, 0},     // mBreath2c
    {0, 99, 30, 0, 0, 0, 0, 0},     // mBreathe
    {200, 99, 30, 5, 0, 0, 0, 0},    // mFlash
    {200, 100, 1, 50, 0, 0, 0, 0}, // mGlitch
};

double parmlist_d[kLEDC_MAX][8] = {
    {0, 0, 0, 0, 0, 0, 0, 0},       // mAM
    {0, 0, 0, 0, 0, 0, 0, 0},       // mStatic (未使用)
    {0.3, 0.8, 0, 0, 0, 0.013333, 0.25, 471.238898},  // mListen (cala_listen_parm)
    {0, 0, 0, 0, 0, 0, 0, 0},       // mRainbow (未使用)
    {2.5, 0, 0, 0, 0, 0, 0, 0},     // mProgressbar (颜色平滑参数)
    {10, 50, 800, 0, 0, 0, 0, 0},   // mBreath2c
    {80, 50, 300, 0, 0, 0, 0, 0},   // mBreathe
    {0, 0, 0, 0, 0, 0, 0, 0},       // mFlash (未使用)
    {0, 0, 0, 0, 0, 0, 0, 0},       // mGlitch (未使用)
};

typedef struct leds_ctrl_msg{
    uint8_t type;
    int8_t  vs;
    int8_t  ls;
    bool    update;
    int parm_i[8];
    double parm_d[8];
}leds_ctrl_msg_t;

typedef enum {
    kDEVLED_NOT_CONFIGURED,      // 尚未配置
    kDEVLED_WIFI_FAILED,         // 连接 WiFi 失败
    kDEVLED_SERVER_FAILED,       // 连接服务器失败
    kDEVLED_HARDWARE_ERROR,      // 硬件状态异常
    kDEVLED_LOW_BATTERY,         // 电池电压不足
    kDEVLED_WAIT_NETWORK,        // 等待联网
    kDEVLED_WAIT_WAKEUP,         // 等待唤醒
    kDEVLED_WAIT_VOICE_INPUT,    // 等待人声输入
    kDEVLED_LISTENING,           // 正在收听
    kDEVLED_WAIT_RESPONSE,       // 等待后台回复
    kDEVLED_PLAYING,             // 设备正在播放
    kDEVLED_READING_RFID,        // 读取 RFID
    kDEVLED_APPLYING_SETTINGS,   // 正在应用设置
    kDEVLED_OTA_UPDATING,        // OTA 更新中
    kDEVLED_CLOSED,              // 关机
    kDEVLED_MAX,                 // 最大模式数 (用于边界检查)
} kDEVLED_Status_t;


const leds_ctrl_msg_t _g_led_ctrl[kDEVLED_MAX] = {
    {kLEDC_LISTEN,      -1, 2, true,  {15, 10, 30, 0, 0, 0, 0, 0},     {0.5, 1.0, 0, 0, 0, 0.013333, 0.25, 942.501359}},    //-kDEVLED_NOT_CONFIGURED
    {kLEDC_BREATHE,     -1, 2, true,  {0, 99, 30, 0, 0, 0, 0, 0},      {80, 50, 300, 0, 0, 0, 0, 0}},                       //-kDEVLED_WIFI_FAILED
    {kLEDC_BREATHE,     -1, 2, true,  {20, 99, 30, 0, 0, 0, 0, 0},     {80, 50, 300, 0, 0, 0, 0, 0}},                       //-kDEVLED_SERVER_FAILED
    {kLEDC_GLITCH,      -1, 2, true,  {10, 100, 1, 50, 0, 0, 0, 0},    {0}},                                                //-kDEVLED_HARDWARE_ERROR
    {kLEDC_BREATH_2C,   -1, 3, true,  {0, 20, 99, 30, 0, 0, 0, 0},     {10, 50, 800, 0, 0, 0, 0, 0}},                       //-kDEVLED_LOW_BATTERY
    {KLEDC_SYSMOTION,   -1, 3, true,  {0, 50, 99, 30, 0, 0, 0, 0},     {0.08, 3, 0, 0, 0, 0, 0,0}},                         //-kDEVLED_WAIT_NETWORK
    {kLEDC_LISTEN,      -1, 2, true,  {90, 25, 30, 0, 0, 0, 0, 0},     {0.5, 1.0, 0, 0, 0, 0.013333, 0.25, 942.501359}},    //-kDEVLED_WAIT_WAKEUP
    {kLEDC_LISTEN,      -1, 2, true,  {250, 30, 30, 0, 0, 0, 0, 0},    {0.3, 0.8, 0, 0, 0, 0.013333, 0.25, 4712.506793}},   //-kDEVLED_WAIT_VOICE_INPUT
    {kLEDC_LISTEN,      -1, 2, true,  {250, 30, 30, 0, 0, 0, 0, 0},    {0.8, 2.0, 0, 0, 0, 0.013333, 0.25, 1178.126698}},   //-kDEVLED_LISTENING
    {kLEDC_LISTEN,      -1, 2, true,  {290, 30, 30, 0, 0, 0, 0, 0},    {0.3, 0.8, 0, 0, 0, 0.013333, 0.25, 4712.506793}},   //-kDEVLED_WAIT_RESPONSE
    {kLEDC_LISTEN,      -1, 2, true,  {290, 30, 30, 0, 0, 0, 0, 0},    {0.8, 2.0, 0, 0, 0, 0.013333, 0.25, 1178.126698}},   //kDEVLED_PLAYING
    {KLEDC_SYSMOTION,   -1, 3, true,  {350, 269, 99, 30, 0, 0, 0, 0},  {0.08, 2, 0, 0, 0, 0, 0,0}},                         //kDEVLED_READING_RFID
    {KLEDC_SYSMOTION,   -1, 3, true,  {252, 296, 99, 30, 0, 0, 0, 0},  {0.08, 3, 0, 0, 0, 0, 0,0}},                         //kDEVLED_APPLYING_SETTINGS
    {kLEDC_PROGRESS_BAR, 0, 3, false, {50, 100, 200, 30, 0, 0, 0, 0},  {2.5, 0, 0, 0, 0, 0, 0, 0}},                         //kDEVLED_OTA_UPDATING
    {kDEVLED_CLOSED,    -1,-1, true,  {0, 99, 0, 0, 0, 0, 0, 0},       {0}},                                                //kDEVLED_CLOSED
};

void set_led_state(QueueHandle_t msg_queue,int state,int value,int light){

    if(msg_queue == NULL)return;
    if(state >= kDEVLED_MAX) return;
    ledam_msg_t msg;
    msg.type = _g_led_ctrl[state].type;
    msg.update = _g_led_ctrl[state].update;
    memcpy(msg.parm_i,_g_led_ctrl[state].parm_i,sizeof(int)*8);
    memcpy(msg.parm_d,_g_led_ctrl[state].parm_d,sizeof(double)*8);
    if( _g_led_ctrl[state].vs >= 0){
        msg.parm_i[_g_led_ctrl[state].vs] = (value < 0) ? 0 : value;
    }
    if(_g_led_ctrl[state].ls >= 0){
        msg.parm_i[_g_led_ctrl[state].ls] = (light < 0) ? 0 : light;
    }
    xQueueSend(msg_queue, &(msg), (TickType_t)1);
}

void app_main(void)
{
    QueueHandle_t msg_queue = init_leds_am(RMT_LED_STRIP_GPIO_NUM,LEDS_FLUSH_SPEED_MS);
    ledam_msg_t msg;
    int i = kDEVLED_NOT_CONFIGURED;

    while(1) {

        set_led_state(msg_queue,i,0,20);
        ESP_LOGI("MAIN","Is now %d",i);
        i++;
        i = ( i >= kDEVLED_MAX ) ? kDEVLED_NOT_CONFIGURED : i;
        vTaskDelay(pdMS_TO_TICKS(10000));
    }

    while (1) {

        i = ( i >= kLEDC_MAX ) ? kLEDC_STATIC : i;

        msg.type = i;

        memcpy(msg.parm_i, parmlist_i[i], sizeof(parmlist_i[i]));
        memcpy(msg.parm_d, parmlist_d[i], sizeof(parmlist_d[i]));
        
        xQueueSend(msg_queue, &(msg), (TickType_t)1);
        
        vTaskDelay(pdMS_TO_TICKS(10000));
        i++;
        //msg.type = (msg.type == mListen) ? mBreath2c : mListen;
    }
}
