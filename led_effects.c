#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"

#include "led_effects.h"

static const char *TAG = "led_effect";

static uint8_t led_strip_pixels[LED_NUMBERS * 3];

// LED 动画函数指针类型
typedef bool (*leds_am_fun)(color_t *ledbuff, int cnt, double* _parm_d, int* _parm_i);

#define SET_LED_COLOR(BUFF,I,R,G,B) BUFF[I*3+0]=G;BUFF[I*3+1]=R;BUFF[I*3+2]=B;
// 定义常量
#define PI 3.14159265358979323846

bool led_static(color_t *ledbuff, int cnt, double* _parm_d, int* _parm_i);
bool led_rainbow(color_t *ledbuff, int cnt, double* _parm_d, int* _parm_i);
bool led_listen(color_t *ledbuff,int cnt,double* _parm_d,int* _parm_i);
bool led_breathe_2c(color_t *ledbuff,int cnt,double* _parm_d,int* _parm_i);
bool led_breathe(color_t *ledbuff, int cnt, double* _parm_d, int* _parm_i);
bool led_progressbar(color_t *ledbuff, int cnt, double* _parm_d, int* _parm_i);
bool led_flash(color_t *ledbuff, int cnt, double* _parm_d, int* _parm_i);
bool led_glitch(color_t *ledbuff, int cnt, double* _parm_d, int* _parm_i);
bool led_symmetric_motion(color_t *ledbuff, int cnt, double* _parm_d, int* _parm_i);

leds_am_fun funlist[kLEDC_MAX] = {
    NULL,
    led_static,
    led_listen,
    led_rainbow,
    led_progressbar,
    led_breathe_2c,
    led_breathe,
    led_flash,
    led_glitch,
    led_symmetric_motion,
};

typedef struct led_am {
    color_t* hsv;
    ledam_type_t now;
    double  hvstep[LED_NUMBERS][3];
    double  temporary[LED_NUMBERS][3];
    color_t temp_color[LED_NUMBERS];
    double  parm_d[8];
    int     parm_i[8];
    int64_t amcnt;
    QueueHandle_t msg_queue;
    leds_am_fun fun;
}led_am_t;

led_am_t leds;

void hsv2rgb(uint32_t h, uint32_t s, uint32_t v, color_t* _color)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        _color->r = rgb_max;
        _color->g = rgb_min + rgb_adj;
        _color->b = rgb_min;
        break;
    case 1:
        _color->r = rgb_max - rgb_adj;
        _color->g = rgb_max;
        _color->b = rgb_min;
        break;
    case 2:
        _color->r = rgb_min;
        _color->g = rgb_max;
        _color->b = rgb_min + rgb_adj;
        break;
    case 3:
        _color->r = rgb_min;
        _color->g = rgb_max - rgb_adj;
        _color->b = rgb_max;
        break;
    case 4:
        _color->r = rgb_min + rgb_adj;
        _color->g = rgb_min;
        _color->b = rgb_max;
        break;
    default:
        _color->r = rgb_max;
        _color->g = rgb_min;
        _color->b = rgb_max - rgb_adj;
        break;
    }
}

void hsv2ledsbuff(color_t *hsv,uint8_t *ledsbuff,uint16_t cnt){
    
    for (int j = 0; j < cnt; j++){
        color_t led_rgb;
        hsv2rgb(hsv[j].h,hsv[j].s,hsv[j].v,&led_rgb);
        SET_LED_COLOR(ledsbuff,j,led_rgb.r,led_rgb.g,led_rgb.b);
    }
}

//// 函数：计算 f(x, t)
//double f(double x, double t, double L1_t, double L2_t, double phi_t, double theta_t) {
//    return sin(2 * PI * x / L1_t + phi_t) + sin(2 * PI * x / L2_t + theta_t);
//    //return A * sin(2 * PI * x / L1_t + phi_t) + B * sin(2 * PI * x / L2_t + theta_t);
//}

//// 平滑的正弦函数：计算 φ(t) 和 θ(t)
//inline double phi(double t) {
//    double A_phi = 2 * PI;     // 振幅
//    double omega_phi = 0.5;    // 频率
//    double delta_phi = 0.0;    // 相位偏移
//    return A_phi * sin(omega_phi * t + delta_phi);
//}
//
//inline double theta(double t) {
//    double A_theta = 2 * PI;   // 振幅
//    double omega_theta = 1.0;  // 频率
//    double delta_theta = PI / 2.0; // 相位偏移
//    return A_theta * sin(omega_theta * t + delta_theta);
//}

void cala_listen_parm(double max_x,double max_t,int n_x,int n_t,double* _parm_d,int* _parm_i){
    double delta_t = max_t / n_t;  // t 的步长
    double delta_x = max_x / n_x;  // x 的步长
    double fx_t = 3.14159265 * 2 / delta_t; //循环周期

    _parm_d[5] = delta_t;
    _parm_d[6] = delta_x;
    _parm_d[7] = fx_t;
}

double gcd(double a, double b) {
    while (fabs(b) > 1e-6) {  // 允许微小误差
        double temp = b;
        b = fmod(a, b);
        a = temp;
    }
    return a;
}

void calaparm(double* _parm_d){
    double T_phi = 2.0 * PI / _parm_d[0];  // 计算 phi_t 的周期
    double T_theta = 2.0 * PI / _parm_d[1]; // 计算 theta_t 的周期
    // 使用近似方法计算最小公倍周期
    double T_total = (T_phi * T_theta) / gcd(T_phi, T_theta); 
    // 更新参数 _parm_d[7] 作为 LED 颜色变化周期
    _parm_d[7] = T_total / _parm_d[5];
}

/**
 * @brief  静态单色模式
 * @param ledbuff  LED 颜色缓冲区 (HSV 格式)
 * @param cnt      运行计数器 (无影响)
 * @param _parm_d  (未使用)
 * @param _parm_i  参数数组：
 *                 _parm_i[0] - 色相 (0~359)
 *                 _parm_i[1] - 饱和度 (0~99)
 *                 _parm_i[2] - 亮度 (0~99)
 * @return true    是否为连续动画
 */
bool led_static(color_t *ledbuff, int cnt, double* _parm_d, int* _parm_i) {
    for (int j = 0; j < LED_NUMBERS; j++) {
        ledbuff[j].h = _parm_i[0] % 360;
        ledbuff[j].s = _parm_i[1] % 100;
        ledbuff[j].v = _parm_i[2] % 100;
    }
    return false;
}

/**
 * @brief  彩虹流动模式
 * @param ledbuff  LED 颜色缓冲区
 * @param cnt      运行计数器 (影响色相起始值)
 * @param _parm_d  (未使用)
 * @param _parm_i  参数数组：
 *                 _parm_i[0] - 起始色相 (0~359)
 *                 _parm_i[1] - 饱和度（0-99）
 *                 _parm_i[2] - 亮度 (0~99)
 *                 _parm_i[3] - LED 之间的色相步长
 *                 _parm_i[4] - 彩虹流动速度
 *                  
 * @return true    是否为连续动画
 */
bool led_rainbow(color_t *ledbuff, int cnt, double* _parm_d, int* _parm_i) {
    int hue_start = (_parm_i[0] + cnt * _parm_i[4]) % 360;
    int hue_step = _parm_i[3];

    for (int j = 0; j < LED_NUMBERS; j++) {
        ledbuff[j].h = (hue_start + j * hue_step) % 360;
        ledbuff[j].s = _parm_i[1] % 100;
        ledbuff[j].v = _parm_i[2] % 100;
    }
    return true;
}

/**
 * @brief  中心对称进度条 (前景色和平滑渐变过渡)
 * 
 * @param ledbuff  LED 颜色缓冲区 (HSV 格式)
 * @param cnt      运行计数器 (无影响)
 * @param _parm_d  参数数组：
 *                 _parm_d[0] - 颜色过渡平滑参数 σ (建议 5~20)
 * @param _parm_i  参数数组：
 *                 _parm_i[0] - 目标进度百分比 (0~100)
 *                 _parm_i[1] - 进度条前景色 (0~359)
 *                 _parm_i[2] - 进度条背景色 (0~359)
 *                 _parm_i[3] - 整体亮度 (0~99)
 * @return false   是否为连续动画
 */
bool led_progressbar(color_t *ledbuff, int cnt, double* _parm_d, int* _parm_i) {
    int progress = _parm_i[0];  // 进度百分比 (0~100)
    int fg_hue = _parm_i[1] % 360;  // 进度条前景色
    int bg_hue = _parm_i[2] % 360;  // 进度条背景色

    double sigma = _parm_d[0];  // 颜色过渡平滑系数 (控制颜色混合范围)

    int half_leds = LED_NUMBERS / 2;
    int threshold = (progress * half_leds) / 100;  // 计算亮灯数量 (单侧)

    for (int j = 0; j < LED_NUMBERS; j++) {
        int distance_from_center = abs(j - half_leds);  // 计算与中心的距离

        double blend_factor = 1.0;  // 默认完全前景色
        if (distance_from_center >= threshold) {
            int d = distance_from_center - threshold;  // 超出进度条的距离
            blend_factor = exp(-pow(d, 2) / (2 * sigma * sigma));  // 高斯平滑计算
        }

        // 颜色混合计算
        int hue = (int)(bg_hue * (1 - blend_factor) + fg_hue * blend_factor) % 360;
        
        ledbuff[j].h = hue;
        ledbuff[j].s = 100;
        ledbuff[j].v = _parm_i[3];
    }
    return false;
}

/**
 * @brief  音频响应模式
 * @param ledbuff  LED 颜色缓冲区
 * @param cnt      运行计数器 (影响颜色变化)
 * @param _parm_d  参数数组：
 *                 _parm_d[0] - omega_phi
 *                 _parm_d[1] - omega_theta
 *                 _parm_d[5] - 时间步长
 *                 _parm_d[6] - x 轴步长
 *                 _parm_d[7] - 颜色变化周期
 * @param _parm_i  参数数组：
 *                 _parm_i[0] - 基础色相
 *                 _parm_i[1] - 颜色变化幅度
 *                 _parm_i[2] - 整体亮度 (0~99)
 * @return true    是否为连续动画
 */
bool led_listen(color_t *ledbuff, int cnt, double* _parm_d, int* _parm_i) {
    double t = (cnt) % ((int)_parm_d[7]) * _parm_d[5];
    double L1_t = 10 + 5 * sin(t);
    double L2_t = 20 - L1_t;

    double phi_t = 2 * PI * sin(_parm_d[0] * t);
    double theta_t = 2 * PI * sin(_parm_d[1] * t + PI / 2.0);

    for (int j = 0; j < LED_NUMBERS; j++) {
        double x = j * _parm_d[6];
        double result = sin(2 * PI * x / L1_t + phi_t) + sin(2 * PI * x / L2_t + theta_t);
        ledbuff[j].h = _parm_i[0] + _parm_i[1] * result;
        ledbuff[j].s = 100;
        ledbuff[j].v = _parm_i[2];
    }
    return true;
}

/**
 * @brief  单色呼吸灯效果
 * 
 * @param ledbuff  LED 颜色缓冲区
 * @param cnt      运行计数器 (影响亮度变化)
 * @param _parm_d  参数数组 (控制亮度变化)
 *                  _parm_d[0] - parm_a：最大亮度 (建议 25~100)
 *                  _parm_d[1] - parm_b：峰值时间偏移 (建议 50)
 *                  _parm_d[2] - parm_c：控制衰减速率 (建议 300~600)
 * @param _parm_i  参数数组
 *                  _parm_i[0] - 色相 (0~359)
 *                  _parm_i[1] - 饱和度 (0~99)
 *                  _parm_i[2] - 亮度 (0~99)
 * @return true    执行成功
 */
bool led_breathe(color_t *ledbuff, int cnt, double* _parm_d, int* _parm_i) {
    // 读取参数
    double parm_a = _parm_d[0]; // 最大亮度
    double parm_b = _parm_d[1]; // 峰值时间偏移
    double parm_c = _parm_d[2]; // 控制衰减速率

    // 计算呼吸灯亮度
    uint8_t led_v = parm_a * exp(-(pow((cnt % 100) - parm_b, 2.0) / parm_c))*_parm_i[2] / 100;

    // 设置 LED 颜色
    for (int j = 0; j < LED_NUMBERS; j++) {
        ledbuff[j].h = _parm_i[0] % 360;        // 色相
        ledbuff[j].s = _parm_i[1];              // 饱和度
        ledbuff[j].v = led_v;                   // 计算出的亮度
    }
    return true;
}

/**
 * @brief  双色呼吸灯
 * @param ledbuff  LED 颜色缓冲区
 * @param cnt      运行计数器 (影响颜色渐变)
 * @param _parm_d  参数数组：
 *                 _parm_d[0] - 最大亮度
 *                 _parm_d[1] - 峰值时间偏移
 *                 _parm_d[2] - 控制衰减速率
 * @param _parm_i  参数数组：
 *                 _parm_i[0] - 起始色相
 *                 _parm_i[1] - 目标色相
 *                 _parm_i[2] - 饱和度 (0~99)
 *                 _parm_i[3] - 亮度 (0~99)
 * @return true    是否为连续动画
 */
bool led_breathe_2c(color_t *ledbuff, int cnt, double* _parm_d, int* _parm_i) {
    if (cnt < 0) return true;

    //double parm_a = _parm_d[0];  // 最大亮度
    double parm_b = _parm_d[1];  // 峰值时间
    double parm_c = _parm_d[2];  // 亮度变化平滑参数

    // 计算呼吸灯的亮度
    double brightness_factor = exp(-(pow((cnt % 100) - parm_b, 2.0) / parm_c));
    //uint8_t brightness = (uint8_t)(parm_a * brightness_factor); // 亮度随指数变化

    // 计算色相过渡，使用最短路径算法
    int hue_start = _parm_i[0];  // 起始色相
    int hue_end = _parm_i[1];    // 目标色相

    // 计算色相的最短路径
    int hue_delta = ((hue_end - hue_start + 540) % 360) - 180;
    double hue = hue_start + hue_delta * brightness_factor; // 按指数曲线过渡色相

    for (int j = 0; j < LED_NUMBERS; j++) {
        ledbuff[j].h = ((int)hue) % 360;  // 确保色相在 0~359 之间
        ledbuff[j].s = _parm_i[2];        // 设定饱和度
        ledbuff[j].v = _parm_i[3];        // 设定亮度
    }
    return true;
}

// ========== 5. 闪烁灯 ==========
/**
 * @brief  LED 闪烁效果
 * 
 * @param ledbuff  LED 颜色缓冲区
 * @param cnt      运行计数器 (用于控制闪烁)
 * @param _parm_d  (未使用)
 * @param _parm_i  _parm_i[0] - 色相
 *                 _parm_i[1] - 饱和度
 *                 _parm_i[2] - 亮度
 *                 _parm_i[3] - 闪烁周期 (帧数)
 *
 * @return true    是否为连续动画
 */
bool led_flash(color_t *ledbuff, int cnt, double* _parm_d, int* _parm_i) {
    int flash_interval = _parm_i[3]; 
    bool is_on = (cnt / flash_interval) % 2;

    for (int j = 0; j < LED_NUMBERS; j++) {
        if (!is_on) {
            ledbuff[j].h = _parm_i[0] % 360;
            ledbuff[j].s = _parm_i[1];
            ledbuff[j].v = _parm_i[2];
        } else {
            ledbuff[j].h = 0;
            ledbuff[j].s = 0;
            ledbuff[j].v = 0;
        }
    }
    return true;
}
/**
 * @brief  闪烁故障灯效 (Glitch Effect)
 * 
 * @param ledbuff  LED 颜色缓冲区 (HSV 格式)
 * @param cnt      运行计数器 (影响闪烁效果)
 * @param _parm_d  (未使用)
 * @param _parm_i  参数数组：
 *                 _parm_i[0] - 色相 Hue (0~359)
 *                 _parm_i[1] - 饱和度 Saturation (0~100)
 *                 _parm_i[2] - 是否启动闪烁 (0 = 关闭, 1 = 启动)
 *                 _parm_i[3] - 亮度波动幅度 (建议 10~50)
 * @return true    是否为连续动画
 */
bool led_glitch(color_t *ledbuff, int cnt, double* _parm_d, int* _parm_i) {
    static int glitch_counter = 0;  // 记录当前的闪烁计数
    static int glitch_brightness = 0;  // 记录当前的亮度

    if (glitch_counter == 0) {
        if (_parm_i[2] > 0) {  
            glitch_counter = rand() % 2;  // 生成短暂闪烁
            _parm_i[2] = 0;  // 关闭闪烁
        } else {
            glitch_counter = rand() % 5;  // 生成长间隔
            _parm_i[2] = 1;  // 重新激活
            glitch_brightness = rand() % _parm_i[3] + 10;  // 生成随机亮度
        }

        // 计算 RGB 颜色 (随机闪烁)
        color_t color;
        hsv2rgb(_parm_i[0], _parm_i[1], glitch_brightness, &color); // 允许用户自定义色相 & 饱和度

        for (int j = 0; j < LED_NUMBERS; j++) {
            ledbuff[j].h = _parm_i[0];  // 设置色相
            ledbuff[j].s = _parm_i[1];  // 设置饱和度
            ledbuff[j].v = glitch_brightness;  // 亮度波动
        }
    } else {
        glitch_counter--;
    }

    return true;
}

/**
 * @brief  中心对称光点循环运动 LED 效果
 *         光点从中心向边界运动，到达边界后返回中心，并在背景色上平滑过渡。
 * 
 * @param ledbuff  LED 颜色缓冲区 (HSV 格式)
 * @param cnt      运行计数器 (影响光点位置)
 * @param _parm_d  参数数组 (控制运动和色相平滑)
 *                 _parm_d[0] - 光点运动速度 (建议 0.01~0.1，较小值 = 运动慢，较大值 = 运动快)
 *                 _parm_d[1] - 颜色过渡平滑范围 σ (建议 2.0~5.0，较小值 = 颜色变化锐利，较大值 = 颜色渐变柔和)
 * @param _parm_i  参数数组 (控制光点和背景颜色)
 *                 _parm_i[0] - 光点色相 (0~359)
 *                 _parm_i[1] - 背景色相 (0~359)
 *                 _parm_i[2] - 饱和度
 *                 _parm_i[3] - 亮度
 * @return true    是否为连续动画 (始终返回 true)
 */
bool led_symmetric_motion(color_t *ledbuff, int cnt, double* _parm_d, int* _parm_i) {
    // 解析参数
    int fg_hue = _parm_i[0] % 360;  // 光点色相
    int bg_hue = _parm_i[1] % 360;  // 背景色相
    double speed = _parm_d[0];      // 运动速度
    double sigma = _parm_d[1];      // 颜色平滑过渡范围（标准差）

    // 计算时间步长和光点位置
    double t = cnt * speed;
    double pos = fabs(sin(t)) * (LED_NUMBERS / 2);  // 计算光点位置
    int center = LED_NUMBERS / 2;
    
    int left_index = center - (int)pos;
    int right_index = center + (int)pos;

    // 遍历所有 LED 进行色相插值计算
    for (int j = 0; j < LED_NUMBERS; j++) {
        // 计算该 LED 到左右光点的最小距离
        double dist_to_left = fabs(j - left_index);
        double dist_to_right = fabs(j - right_index);
        double min_dist = fmin(dist_to_left, dist_to_right);

        // 使用高斯曲线计算影响力（越接近光点，影响越大）
        double influence = exp(-pow(min_dist, 2) / (2 * pow(sigma, 2)));

        // 计算最终色相（平滑插值）
        double hue = (bg_hue * (1 - influence) + fg_hue * influence);
        
        ledbuff[j].h = ((int)hue) % 360;
        ledbuff[j].s = _parm_i[2];
        ledbuff[j].v = _parm_i[3]; // 亮度也进行平滑过渡
    }
    return true;
}


/**
 * @brief  计算渐变动画的步长 (支持 色相、饱和度、亮度)
 * 
 * @param ledbuff_1  起始 LED 颜色缓冲区 (HSV)
 * @param ledbuff_2  目标 LED 颜色缓冲区 (HSV)
 * @param temporary  存储当前渐变状态 (h, s, v)
 * @param step       计算渐变步长 (h, s, v)
 */
void cala_am_hstep(color_t *ledbuff_1, color_t *ledbuff_2, double temporary[][3], double step[][3]) {
    for (int i = 0; i < LED_NUMBERS; i++) {
        // 色相 (Hue) 计算最短路径
        int h1 = (int)ledbuff_1[i].h;
        int h2 = (int)ledbuff_2[i].h;
        int delta_h = ((h2 - h1 + 540) % 360) - 180;  // 确保变化量在 [-180, 180]
        
        step[i][0] = delta_h / (double)LEDS_AM_TIMECNTS;
        temporary[i][0] = (double)h1;

        // 饱和度 (Saturation) 线性过渡
        step[i][1] = (ledbuff_2[i].s - ledbuff_1[i].s) / (double)LEDS_AM_TIMECNTS;
        temporary[i][1] = (double)ledbuff_1[i].s;

        // 亮度 (Value) 线性过渡
        step[i][2] = (ledbuff_2[i].v - ledbuff_1[i].v) / (double)LEDS_AM_TIMECNTS;
        temporary[i][2] = (double)ledbuff_1[i].v;
    }
}

/**
 * @brief  计算渐变动画 (支持 色相、饱和度、亮度)
 * 
 * @param ledbuff    LED 颜色缓冲区 (HSV)
 * @param cnt        运行计数器 (影响颜色渐变)
 * @param temporary  存储当前渐变状态 (h, s, v)
 * @param step       计算渐变步长 (h, s, v)
 * @return true      是否为连续动画
 */
bool led_smooth_am(color_t *ledbuff, uint32_t cnt, double temporary[][3], double step[][3]) {
    for (int j = 0; j < LED_NUMBERS; j++) {
        // 计算新的色相值
        int new_h = (int)round(temporary[j][0] + step[j][0] * cnt);
        if (new_h < 0) new_h += 360;
        new_h %= 360;

        // 计算新的饱和度值 (确保范围 [0,100])
        int new_s = (int)round(temporary[j][1] + step[j][1] * cnt);
        if (new_s < 0) new_s = 0;
        if (new_s > 100) new_s = 100;

        // 计算新的亮度值 (确保范围 [0,100])
        int new_v = (int)round(temporary[j][2] + step[j][2] * cnt);
        if (new_v < 0) new_v = 0;
        if (new_v > 100) new_v = 100;

        // 更新 LED 颜色缓冲区
        ledbuff[j].h = new_h;
        ledbuff[j].s = new_s;
        ledbuff[j].v = new_v;
    }
    return true;
}

static void led_loop_task(void *arg)
{
    ledsconfig_t* cfg = (ledsconfig_t*)arg;

    int flush_speed = cfg->flush_speed_ms;
    int am_time = cfg->am_time;

    rmt_channel_handle_t led_chan = NULL;

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = cfg->led_gpio,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = 10000000,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    rmt_encoder_handle_t led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = 10000000,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    ESP_ERROR_CHECK(rmt_enable(led_chan));

    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };

    ledam_msg_t msg;

    leds.hsv = malloc(LED_NUMBERS*sizeof(color_t));
    leds.amcnt = 0;
    bool iscontinue = false;

    for (int i = 0; i < LED_NUMBERS; i++)
    {
        leds.hsv[i].h = 0;
        leds.hsv[i].s = 99;
        leds.hsv[i].v = 0;
    }

    hsv2ledsbuff(leds.hsv,led_strip_pixels,LED_NUMBERS);
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));

    ESP_LOGI(TAG,"LEDs-AM task is started");
    
    while (1) {
        if (xQueueReceive(leds.msg_queue, &(msg), (TickType_t)1))
        {
            if((msg.type != leds.now)||(msg.update)){
                leds.amcnt = -( am_time );
                leds.now = msg.type;
                leds.fun = funlist[msg.type];
                if(leds.fun == NULL) continue;
                memcpy(leds.parm_d,msg.parm_d,sizeof(double)*8);
                memcpy(leds.parm_i,msg.parm_i,sizeof(int)*8);
                leds.fun(leds.temp_color,0,leds.parm_d,leds.parm_i);
                cala_am_hstep(leds.hsv,leds.temp_color,leds.temporary,leds.hvstep);
                //ESP_LOGI(TAG,"LEDs-AM set type to %d,%d",leds.now,leds.amcnt);
            }
            iscontinue = true;
            memcpy(leds.parm_d,msg.parm_d,sizeof(double)*8);
            memcpy(leds.parm_i,msg.parm_i,sizeof(int)*8);
        }
        if(iscontinue){
            if( leds.amcnt <= 0 ){
                led_smooth_am(leds.hsv,am_time + leds.amcnt,leds.temporary,leds.hvstep);
            }
            else{
                iscontinue = leds.fun(leds.hsv,leds.amcnt,leds.parm_d,leds.parm_i);
            }
            hsv2ledsbuff(leds.hsv,led_strip_pixels,LED_NUMBERS);
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            leds.amcnt++;
        }
        vTaskDelay(pdMS_TO_TICKS(flush_speed));
    }
}

QueueHandle_t init_leds_am(int led_gpio,int flush_speed){
    ledsconfig_t config;
    config.flush_speed_ms = flush_speed;
    config.led_gpio = led_gpio;
    config.am_time = LEDS_AM_TIMECNTS;

    leds.msg_queue = xQueueCreate(16, sizeof(ledam_msg_t));

    xTaskCreatePinnedToCore(led_loop_task, "led_loop_task", 4 * 1024, &config, 5, NULL, 0);

    return leds.msg_queue;
}