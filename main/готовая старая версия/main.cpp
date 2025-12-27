#include <math.h>
#include <string.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "led_strip.h"
#include "esp_random.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ==========================================
// 1. НАСТРОЙКИ
// ==========================================
#define LED_GPIO            8
#define LEDS_PER_FACE       9
#define FACES_COUNT         6
#define TOTAL_LEDS          (LEDS_PER_FACE * FACES_COUNT)

// >>> НАСТРОЙКИ ЯРКОСТИ <<<
#define BRIGHTNESS_ACTIVE   100     // Яркость когда крутим (Днем)
#define BRIGHTNESS_IDLE     40      // Яркость когда стоит (Ночник) - ставь меньше для уюта

#define IDLE_TIMEOUT_MS     10000  // 10 секунд до засыпания
#define MOVE_THRESHOLD      4
#define SCRAMBLE_SPEED_MS   400    
#define INERTIA_FACTOR      0.02f 

#define PIN_SDA             5
#define PIN_SCL             6

static const char *TAG = "CUBE_RUBIK";
static led_strip_handle_t led_strip;
MPU6050 mpu;

typedef struct { float w, x, y, z; } Quat;
typedef struct { float x, y, z; } Vec3;
typedef struct { uint8_t r, g, b; } Color;

Color rubik_state[TOTAL_LEDS];

// ==========================================
// 2. ЦВЕТА "МИРА"
// ==========================================
const Color WORLD_UP     = {255, 0, 0};    // Красный
const Color WORLD_DOWN   = {0, 0, 255};    // Синий
const Color WORLD_FRONT  = {0, 255, 0};    // Зеленый
const Color WORLD_BACK   = {255, 255, 0};  // Желтый
const Color WORLD_RIGHT  = {255, 0, 255};  // Фиолетовый
const Color WORLD_LEFT   = {0, 255, 80};   // Холодная Мята

Vec3 face_normals[FACES_COUNT] = {
    {0, 0, 1},   // 0: Верхняя (Z+)
    {1, 0, 0},   // 2: Передняя (X+) 
    {0, -1, 0},   // 5: Левая (Y-)
    {-1, 0, 0},  // 3: Задняя (X-)   
    {0, 1, 0},   // 4: Правая (Y+)
    {0, 0, -1},  // 1: Нижняя (Z-)
};


// ==========================================
// 3. МАТЕМАТИКА
// ==========================================

void quat_normalize(Quat *q) {
    float mag = sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    if (mag == 0) return;
    q->w /= mag; q->x /= mag; q->y /= mag; q->z /= mag;
}

Quat quat_lerp(Quat q_curr, Quat q_target, float t) {
    Quat res;
    float dot = q_curr.w*q_target.w + q_curr.x*q_target.x + q_curr.y*q_target.y + q_curr.z*q_target.z;
    if (dot < 0) {
        q_target.w = -q_target.w; q_target.x = -q_target.x; q_target.y = -q_target.y; q_target.z = -q_target.z;
    }
    res.w = q_curr.w + t * (q_target.w - q_curr.w);
    res.x = q_curr.x + t * (q_target.x - q_curr.x);
    res.y = q_curr.y + t * (q_target.y - q_curr.y);
    res.z = q_curr.z + t * (q_target.z - q_curr.z);
    quat_normalize(&res);
    return res;
}

float vec_dot(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
float focus_curve(float val) { return (val<=0)?0.0f : powf(val, 4.0f); }

Vec3 rotate_vector(Vec3 v, Quat q) {
    float num2 = q.x * q.x; float num3 = q.y * q.y; float num4 = q.z * q.z;
    float num5 = q.x * q.y; float num6 = q.w * q.z; float num7 = q.x * q.z;
    float num8 = q.w * q.y; float num9 = q.y * q.z; float num10 = q.w * q.x;
    Vec3 res;
    res.x = (1.0f - 2.0f * (num3 + num4)) * v.x + (2.0f * (num5 - num6)) * v.y + (2.0f * (num7 + num8)) * v.z;
    res.y = (2.0f * (num5 + num6)) * v.x + (1.0f - 2.0f * (num2 + num4)) * v.y + (2.0f * (num9 - num10)) * v.z;
    res.z = (2.0f * (num7 - num8)) * v.x + (2.0f * (num9 + num10)) * v.y + (1.0f - 2.0f * (num2 + num3)) * v.z;
    return res;
}

// ==========================================
// 4. ЛОГИКА КУБИКА РУБИКА
// ==========================================
void rubik_reset() {
    Color palette[6] = {WORLD_UP, WORLD_DOWN, WORLD_FRONT, WORLD_BACK, WORLD_RIGHT, WORLD_LEFT};
    for (int face = 0; face < 6; face++) {
        for (int k = 0; k < 9; k++) {
            rubik_state[face * 9 + k] = palette[face];
        }
    }
}

void cycle_4_pixels(int i1, int i2, int i3, int i4) {
    Color temp = rubik_state[i4];
    rubik_state[i4] = rubik_state[i3]; rubik_state[i3] = rubik_state[i2]; rubik_state[i2] = rubik_state[i1]; rubik_state[i1] = temp;
}

void rotate_face_itself(int face_idx) {
    int start = face_idx * 9;
    cycle_4_pixels(start+0, start+2, start+8, start+6);
    cycle_4_pixels(start+1, start+5, start+7, start+3);
}

void move_rotate_right() {
    rotate_face_itself(4);
    int F=2*9; int U=0*9; int B=3*9; int D=1*9;
    cycle_4_pixels(F+2, U+2, B+6, D+2); cycle_4_pixels(F+5, U+5, B+3, D+5); cycle_4_pixels(F+8, U+8, B+0, D+8);
}
void move_rotate_left() {
    rotate_face_itself(5);
    int F=2*9; int D=1*9; int B=3*9; int U=0*9;
    cycle_4_pixels(F+0, D+0, B+8, U+0); cycle_4_pixels(F+3, D+3, B+5, U+3); cycle_4_pixels(F+6, D+6, B+2, U+6);
}
void move_rotate_top() {
    rotate_face_itself(0);
    int F=2*9; int L=5*9; int B=3*9; int R=4*9;
    cycle_4_pixels(F+0, L+0, B+0, R+0); cycle_4_pixels(F+1, L+1, B+1, R+1); cycle_4_pixels(F+2, L+2, B+2, R+2);
}
void move_rotate_bottom() {
    rotate_face_itself(1);
    int F=2*9; int R=4*9; int B=3*9; int L=5*9;
    cycle_4_pixels(F+6, R+6, B+6, L+6); cycle_4_pixels(F+7, R+7, B+7, L+7); cycle_4_pixels(F+8, R+8, B+8, L+8);
}

void rubik_scramble() {
    int move = esp_random() % 4;
    switch(move) {
        case 0: move_rotate_right(); break;
        case 1: move_rotate_left(); break;
        case 2: move_rotate_top(); break;
        case 3: move_rotate_bottom(); break;
    }
}

// ==========================================
// 5. ГЛАВНАЯ ЗАДАЧА
// ==========================================
void init_all() {
    led_strip_config_t s_cfg; memset(&s_cfg,0,sizeof(s_cfg));
    s_cfg.strip_gpio_num=LED_GPIO; s_cfg.max_leds=TOTAL_LEDS; s_cfg.led_model=LED_MODEL_WS2812; s_cfg.color_component_format=LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    led_strip_rmt_config_t r_cfg; memset(&r_cfg,0,sizeof(r_cfg));
    r_cfg.resolution_hz=10000000;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&s_cfg, &r_cfg, &led_strip));
    led_strip_clear(led_strip);

    i2c_config_t c; memset(&c,0,sizeof(c)); c.mode=I2C_MODE_MASTER; c.sda_io_num=PIN_SDA; c.scl_io_num=PIN_SCL; c.sda_pullup_en=1; c.scl_pullup_en=1; c.master.clk_speed=400000;
    i2c_param_config(I2C_NUM_0, &c); i2c_driver_install(I2C_NUM_0, c.mode, 0,0,0);
    
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setXGyroOffset(220); mpu.setYGyroOffset(76); mpu.setZGyroOffset(-85); mpu.setZAccelOffset(1788);
    mpu.setDMPEnabled(true);
    rubik_reset();
}

void cube_task(void *pvParam) {
    init_all();
    
    uint16_t packetSize = mpu.dmpGetFIFOPacketSize();
    uint8_t fifoBuffer[64];
    Quaternion q_raw; 
    
    Vec3 v_up={0,0,1}; Vec3 v_down={0,0,-1};
    Vec3 v_front={1,0,0}; Vec3 v_back={-1,0,0};
    Vec3 v_right={0,1,0}; Vec3 v_left={0,-1,0};

    uint32_t last_move_time = xTaskGetTickCount();
    uint32_t last_scramble_time = 0;
    
    Quat q_prev = {0,0,0,0};
    Quat q_displayed = {1, 0, 0, 0}; 

    bool was_idle = false;

    // >>> ПЕРЕМЕННАЯ ДЛЯ ПЛАВНОЙ ЯРКОСТИ <<<
    float current_brightness = BRIGHTNESS_ACTIVE; 

    while(1) {
        uint16_t fifoCount = mpu.getFIFOCount();
        if (fifoCount < packetSize) { vTaskDelay(1); continue; }
        if (fifoCount >= 1024) { mpu.resetFIFO(); continue; }
        while (fifoCount >= packetSize) { mpu.getFIFOBytes(fifoBuffer, packetSize); fifoCount-=packetSize; }
        mpu.dmpGetQuaternion(&q_raw, fifoBuffer);
        if (q_raw.w==0 && q_raw.x==0) continue;

        Quat q_target = {q_raw.w, q_raw.x, q_raw.y, q_raw.z};

        float diff = fabs(q_target.w - q_prev.w) + fabs(q_target.x - q_prev.x) + fabs(q_target.y - q_prev.y) + fabs(q_target.z - q_prev.z);
        if (diff * 1000 > MOVE_THRESHOLD) last_move_time = xTaskGetTickCount();
        q_prev = q_target;

        uint32_t now = xTaskGetTickCount();
        bool is_idle = (now - last_move_time) > pdMS_TO_TICKS(IDLE_TIMEOUT_MS);

        // >>> ЛОГИКА ПЛАВНОЙ СМЕНЫ ЯРКОСТИ <<<
        float target_brightness = is_idle ? BRIGHTNESS_IDLE : BRIGHTNESS_ACTIVE;
        
        // Медленно двигаем текущую яркость к целевой (на 0.5 единицы за кадр)
        if (current_brightness < target_brightness) current_brightness += 0.5f;
        if (current_brightness > target_brightness) current_brightness -= 0.5f;

        if (is_idle) {
            // === РЕЖИМ РУБИКА (IDLE) ===
            if (!was_idle) { 
                q_displayed = q_target;
                was_idle = true; 
            }

            if (now - last_scramble_time > pdMS_TO_TICKS(SCRAMBLE_SPEED_MS)) {
                rubik_scramble();
                last_scramble_time = now;
            }

            for (int i=0; i<TOTAL_LEDS; i++) {
                // Используем current_brightness
                uint8_t r = (uint8_t)(rubik_state[i].r * current_brightness / 255.0f);
                uint8_t g = (uint8_t)(rubik_state[i].g * current_brightness / 255.0f);
                uint8_t b = (uint8_t)(rubik_state[i].b * current_brightness / 255.0f);
                led_strip_set_pixel(led_strip, i, r, g, b);
            }

        } else {
            // === РЕЖИМ ГРАВИТАЦИИ (ACTIVE) ===
            was_idle = false;
            
            q_displayed = quat_lerp(q_displayed, q_target, INERTIA_FACTOR);
            Quat q_inv = {q_displayed.w, -q_displayed.x, -q_displayed.y, -q_displayed.z};
            
            Vec3 l_up = rotate_vector(v_up, q_inv);
            Vec3 l_down = rotate_vector(v_down, q_inv);
            Vec3 l_front = rotate_vector(v_front, q_inv);
            Vec3 l_back = rotate_vector(v_back, q_inv);
            Vec3 l_right = rotate_vector(v_right, q_inv);
            Vec3 l_left = rotate_vector(v_left, q_inv);

            for (int i=0; i<FACES_COUNT; i++) {
                Vec3 n = face_normals[i];
                float w[6];
                w[0]=focus_curve(vec_dot(n, l_up)); w[1]=focus_curve(vec_dot(n, l_down));
                w[2]=focus_curve(vec_dot(n, l_front)); w[3]=focus_curve(vec_dot(n, l_back));
                w[4]=focus_curve(vec_dot(n, l_right)); w[5]=focus_curve(vec_dot(n, l_left));

                float ra=0, ga=0, ba=0;
                auto add=[&](Color c, float f){ ra+=c.r*f; ga+=c.g*f; ba+=c.b*f; };
                add(WORLD_UP, w[0]); add(WORLD_DOWN, w[1]);
                add(WORLD_FRONT, w[2]); add(WORLD_BACK, w[3]);
                add(WORLD_RIGHT, w[4]); add(WORLD_LEFT, w[5]);

                float maxv = fmaxf(ra, fmaxf(ga, ba));
                if(maxv>255) { float s=255.0f/maxv; ra*=s; ga*=s; ba*=s; }

                for(int k=0; k<9; k++) {
                    int idx = i*9+k;
                    rubik_state[idx].r = (uint8_t)ra;
                    rubik_state[idx].g = (uint8_t)ga;
                    rubik_state[idx].b = (uint8_t)ba;

                    // Используем current_brightness
                    uint8_t r_fin = (uint8_t)(ra * current_brightness / 255.0f);
                    uint8_t g_fin = (uint8_t)(ga * current_brightness / 255.0f);
                    uint8_t b_fin = (uint8_t)(ba * current_brightness / 255.0f);
                    led_strip_set_pixel(led_strip, idx, r_fin, g_fin, b_fin);
                }
            }
        }
        led_strip_refresh(led_strip);
    }
}

extern "C" void app_main(void) {
    xTaskCreate(cube_task, "cube", 8192, NULL, 5, NULL);
}