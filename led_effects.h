#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define RMT_LED_STRIP_GPIO_NUM      4
#define LED_NUMBERS                 20
#define LEDS_FLUSH_SPEED_MS         20
#define LEDS_AM_TIMECNTS            20

typedef union color
{
    struct{
        uint32_t l:8;
        uint32_t r:8;
        uint32_t g:8;
        uint32_t b:8;
    };
    struct{
        uint32_t h:16;
        uint32_t s:8;
        uint32_t v:8;
    };
    uint32_t c;
}color_t;

typedef enum ledam_type{
    kLEDC_AM_RUN = 0,       // 运行模式
    kLEDC_STATIC,           // 静态单色模式
    kLEDC_LISTEN,           // 音频响应模式
    kLEDC_RAINBOW,          // 彩虹流动模式
    kLEDC_PROGRESS_BAR,     // 进度条模式
    kLEDC_BREATH_2C,        // 双色呼吸模式
    kLEDC_BREATHE,          // 单色呼吸模式
    kLEDC_FLASH,            // 闪烁模式
    kLEDC_GLITCH,           // 故障闪烁模式
    KLEDC_SYSMOTION,       // 中心对称光点循环运动模式
    kLEDC_MAX               // 最大模式数 (用于边界检查)
}ledam_type_t;

typedef struct ledam_msg{
    ledam_type_t type;
    uint16_t ledam_speed;
    double parm_d[8];
    int parm_i[8];
    bool update;
}ledam_msg_t;

typedef struct ledsconfig{
    int led_gpio;
    int flush_speed_ms;
    int am_time;
}ledsconfig_t;

QueueHandle_t init_leds_am(int led_gpio,int flush_speed);
void cala_listen_parm(double max_x,double max_t,int n_x,int n_t,double* _parm_d,int* _parm_i);
void calaparm(double* _parm_d);


#ifdef __cplusplus
}
#endif