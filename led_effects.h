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
    mAmrun = 0,
    mStatic,
    mListen,
    mRainbow,
    mProgressbar,
    mBreath2c,
    mBreathe,
    mFlash,
    mGlitch,
    mMAX,
}ledam_type_t;

typedef struct ledam_msg{
    ledam_type_t type;
    uint16_t ledam_speed;
    double parm_d[8];
    int parm_i[8];
}ledam_msg_t;

typedef struct ledsconfig{
    int led_gpio;
    int flush_speed_ms;
    int am_time;
}ledsconfig_t;

QueueHandle_t init_leds_am(int led_gpio,int flush_speed);
void cala_listen_parm(double max_x,double max_t,int n_x,int n_t,double* _parm_d,int* _parm_i);


#ifdef __cplusplus
}
#endif