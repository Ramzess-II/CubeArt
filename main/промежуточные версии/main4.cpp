#include <math.h>
#include <string.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "esp_random.h"

// Подключаем библиотеки DMP
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ==========================================
// 1. НАСТРОЙКИ ОБОРУДОВАНИЯ
// ==========================================
#define LED_GPIO            8
#define LEDS_PER_FACE       9
#define FACES_COUNT         6
#define TOTAL_LEDS          (LEDS_PER_FACE * FACES_COUNT)

#define KUBE_POWER_PIN      GPIO_NUM_7      
#define BUTTON_PIN          GPIO_NUM_4      
#define PIN_SDA             5
#define PIN_SCL             6

#define DEBOUNCE_TIME_MS    50
#define LONG_PRESS_MS       1500            

// Яркость режимов (0-255)
#define BRIGHT_ACTIVE       100             
#define BRIGHT_IDLE         30              // Можно вернуть повыше, т.к. дыхание само управляет яркостью
#define BRIGHT_DICE         120
#define BRIGHT_FLASHLIGHT   90              

#define IDLE_TIMEOUT_MS     10000           
#define IDLE_CHANGE_INTERVAL_MS (2 * 60 * 1000) 

#define MOVE_THRESHOLD      4               
#define INERTIA_FACTOR      0.08f           

// >>> МЕРТВАЯ ЗОНА (Deadzone) <<<
// Угол (в градусах), в пределах которого цвет НЕ меняется.
// 35 градусов - оптимально. 
#define GRAVITY_DEADZONE_DEG 40.0f

// ОФСЕТЫ
#define OFFSET_X            0.0f
#define OFFSET_Y            0.0f
#define OFFSET_Z            0.0f

// Скорости анимаций
#define SCRAMBLE_SPEED_MS   400             
#define MATRIX_SPEED_MS     120             
#define SPARKLE_SPEED_MS    50
// BREATH настраивается в функции

// Настройки DICE / YESNO
#define SHAKE_THRESHOLD     150             
#define DICE_CHAOS_TIME     2000            
#define DICE_DEMO_ON_MS     250             
#define DICE_DEMO_OFF_MS    150             
#define DICE_CHAOS_SPEED_MS 80              

#define YESNO_INTRO_TIME_MS 3000            

static const char *TAG = "CUBE_MAIN";
static led_strip_handle_t led_strip;
MPU6050 mpu;

// ==========================================
// 2. СТРУКТУРЫ И ГЛОБАЛЬНЫЕ ФЛАГИ
// ==========================================
typedef enum {
    MODE_GRAVITY = 0, 
    MODE_DICE,
    MODE_YESNO,       
    MODE_FLASHLIGHT,  
    MODE_COUNT        
} CubeMode;

volatile CubeMode current_mode = MODE_GRAVITY;
volatile bool is_shutdown = false; 

typedef struct { float w, x, y, z; } Quat;
typedef struct { float x, y, z; } Vec3;
typedef struct { float r, g, b; } ColorF;   
typedef struct { uint8_t r, g, b; } Color;  

Color target_leds[TOTAL_LEDS];      
ColorF current_leds[TOTAL_LEDS];    

// ==========================================
// 3. КОНСТАНТЫ
// ==========================================
const Color WORLD_UP     = {255, 0, 0};    
const Color WORLD_DOWN   = {0, 0, 255};    
const Color WORLD_FRONT  = {0, 255, 0};    
const Color WORLD_BACK   = {255, 255, 0};  
const Color WORLD_RIGHT  = {255, 0, 255};  
const Color WORLD_LEFT   = {0, 255, 80};   

const Color PALETTE[6] = {
    WORLD_UP, WORLD_DOWN, WORLD_FRONT, WORLD_BACK, WORLD_RIGHT, WORLD_LEFT
};

Vec3 face_normals[FACES_COUNT] = {
    {0, 0, 1},   // 0
    {0, 1, 0},   // 4 
    {-1, 0, 0},  // 3
    {0, -1, 0},  // 5
    {1, 0, 0},   // 2
    {0, 0, -1},  // 1
};

const uint8_t DICE_PATTERNS[7][9] = {
    {0,0,0, 0,0,0, 0,0,0}, 
    {0,0,0, 0,1,0, 0,0,0}, // 1
    {1,0,0, 0,0,0, 0,0,1}, // 2
    {1,0,0, 0,1,0, 0,0,1}, // 3
    {1,0,1, 0,0,0, 1,0,1}, // 4 
    {1,0,1, 0,1,0, 1,0,1}, // 5 
    {1,0,1, 1,0,1, 1,0,1}  // 6
};

const Color GAME_COLORS[3] = {
    {255, 0, 0},   // Red
    {0, 255, 0},   // Green
    {0, 0, 255}    // Blue
};

// ==========================================
// 4. МАТЕМАТИКА
// ==========================================
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Порог для мертвой зоны (cos(35 градусов))
const float DEADZONE_THRESHOLD = cosf(GRAVITY_DEADZONE_DEG * M_PI / 180.0f);

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

void lerp_color(ColorF *curr, Color target, float speed) {
    curr->r += ((float)target.r - curr->r) * speed;
    curr->g += ((float)target.g - curr->g) * speed;
    curr->b += ((float)target.b - curr->b) * speed;
}

Color hsv2rgb(uint16_t h, uint8_t s, uint8_t v) {
    h %= 360;
    uint8_t rgb_max = v * 2.55f;
    uint8_t rgb_min = rgb_max * (100 - s) / 100.0f;
    uint8_t i = h / 60;
    uint8_t diff = h % 60;
    uint8_t rgb_adj = (rgb_max - rgb_min) * diff / 60;
    uint8_t r=0, g=0, b=0;
    switch(i) {
    case 0: r=rgb_max; g=rgb_min+rgb_adj; b=rgb_min; break;
    case 1: r=rgb_max-rgb_adj; g=rgb_max; b=rgb_min; break;
    case 2: r=rgb_min; g=rgb_max; b=rgb_min+rgb_adj; break;
    case 3: r=rgb_min; g=rgb_max-rgb_adj; b=rgb_max; break;
    case 4: r=rgb_min+rgb_adj; g=rgb_min; b=rgb_max; break;
    case 5: r=rgb_max; g=rgb_min; b=rgb_max-rgb_adj; break;
    }
    return (Color){r, g, b};
}

// ==========================================
// 5. ЛОГИКА ЭФФЕКТОВ
// ==========================================

void draw_digit_all_faces(int number, Color c) {
    const uint8_t *pat = DICE_PATTERNS[number]; 
    for(int f=0; f<FACES_COUNT; f++) {
        int start = f * 9;
        for (int k=0; k<9; k++) {
            if (pat[k]) target_leds[start + k] = c;
            else target_leds[start + k] = (Color){0,0,0};
        }
    }
}

void fill_all_faces(Color c) {
    for(int i=0; i<TOTAL_LEDS; i++) target_leds[i] = c;
}

void fill_face_solid(int face_idx, Color c) {
    int start = face_idx * 9;
    for(int k=0; k<9; k++) target_leds[start + k] = c;
}

// --- Rubik Logic ---
void cycle_4_pixels(int i1, int i2, int i3, int i4) {
    Color temp = target_leds[i4];
    target_leds[i4] = target_leds[i3]; target_leds[i3] = target_leds[i2]; 
    target_leds[i2] = target_leds[i1]; target_leds[i1] = temp;
}
void rotate_face_itself(int face_idx) {
    int s = face_idx * 9;
    cycle_4_pixels(s+0, s+2, s+8, s+6);
    cycle_4_pixels(s+1, s+5, s+7, s+3);
}
void scramble_move() {
    int move = esp_random() % 4;
    int F=2*9; int U=0*9; int B=3*9; int D=1*9; int R=4*9; int L=5*9;
    if (move == 0) { 
        rotate_face_itself(4);
        cycle_4_pixels(F+2, U+2, B+6, D+2); cycle_4_pixels(F+5, U+5, B+3, D+5); cycle_4_pixels(F+8, U+8, B+0, D+8);
    } else if (move == 1) { 
        rotate_face_itself(5);
        cycle_4_pixels(F+0, D+0, B+8, U+0); cycle_4_pixels(F+3, D+3, B+5, U+3); cycle_4_pixels(F+6, D+6, B+2, U+6);
    } else if (move == 2) { 
        rotate_face_itself(0);
        cycle_4_pixels(F+0, L+0, B+0, R+0); cycle_4_pixels(F+1, L+1, B+1, R+1); cycle_4_pixels(F+2, L+2, B+2, R+2);
    } else if (move == 3) { 
        rotate_face_itself(1);
        cycle_4_pixels(F+6, R+6, B+6, L+6); cycle_4_pixels(F+7, R+7, B+7, L+7); cycle_4_pixels(F+8, R+8, B+8, L+8);
    }
}

// --- Matrix Rain Logic ---
void matrix_effect_step() {
    for(int f=0; f<FACES_COUNT; f++) {
        int s = f * 9;
        for(int col=0; col<3; col++) {
            int top = s + col;
            int mid = s + col + 3;
            int bot = s + col + 6;
            target_leds[bot] = target_leds[mid];
            target_leds[mid] = target_leds[top];
            if ((esp_random() % 100) < 40) {
                uint8_t b = (esp_random() % 200) + 55;
                target_leds[top] = (Color){0, b, 0};
            } else {
                target_leds[top] = (Color){0, 0, 0};
            }
        }
    }
}

// --- Sparkle Logic ---
void sparkle_effect_step() {
    for(int i=0; i<TOTAL_LEDS; i++) {
        target_leds[i].r = (uint8_t)(target_leds[i].r * 0.80f);
        target_leds[i].g = (uint8_t)(target_leds[i].g * 0.80f);
        target_leds[i].b = (uint8_t)(target_leds[i].b * 0.80f);
    }
    int spawns = (esp_random() % 2) + 1;
    for(int k=0; k<spawns; k++) {
        int idx = esp_random() % TOTAL_LEDS;
        int color_idx = esp_random() % 6;
        target_leds[idx] = PALETTE[color_idx];
    }
}

// --- Breath Logic (Исправлено: Синусоидальное дыхание) ---
static float breath_phase = 0.0f; 
static uint16_t breath_hue = 0;   

void breath_effect_fade_in_out() {
    // Двигаем фазу (скорость дыхания)
    breath_phase += 0.02f;
    
    // Если полный цикл (затухло) -> меняем цвет
    if (breath_phase > M_PI) {
        breath_phase = 0.0f;
        breath_hue = (uint16_t)(esp_random() % 360); 
    }
    
    // Яркость по синусу: 0.05 ... 1.0
    float sin_val = sinf(breath_phase);
    float val_factor = 0.05f + (0.95f * sin_val); // от 5% до 100%
    
    // Базовая яркость цвета 100 (полная насыщенность), 
    // но модулируем Value синусоидой
    Color c = hsv2rgb(breath_hue, 100, (uint8_t)(val_factor * 100)); 
    
    fill_all_faces(c);
}

// ==========================================
// 6. ИНИЦИАЛИЗАЦИЯ И ЗАДАЧИ
// ==========================================
void init_hardware() {
    gpio_reset_pin(KUBE_POWER_PIN);
    gpio_set_direction(KUBE_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(KUBE_POWER_PIN, 1); 

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

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
}

void task_button(void *pvParam) {
    bool btn_state = false;
    uint32_t press_time = 0;
    while(gpio_get_level(BUTTON_PIN)) vTaskDelay(10);
    
    while(1) {
        bool reading = gpio_get_level(BUTTON_PIN);
        if (reading && !btn_state) {
            btn_state = true;
            press_time = xTaskGetTickCount();
        } 
        else if (!reading && btn_state) {
            btn_state = false;
            uint32_t duration = (xTaskGetTickCount() - press_time) * portTICK_PERIOD_MS;
            if (duration > 50 && duration < LONG_PRESS_MS) {
                if (!is_shutdown) {
                    current_mode = (CubeMode)((current_mode + 1) % MODE_COUNT);
                }
            }
        }
        else if (reading && btn_state) {
            uint32_t duration = (xTaskGetTickCount() - press_time) * portTICK_PERIOD_MS;
            if (duration > LONG_PRESS_MS && !is_shutdown) {
                is_shutdown = true;
                vTaskDelay(pdMS_TO_TICKS(50));
                
                // Red Flash
                for(int i=0; i<TOTAL_LEDS; i++) led_strip_set_pixel(led_strip, i, 200, 0, 0);
                led_strip_refresh(led_strip);
                vTaskDelay(pdMS_TO_TICKS(500));
                
                // Off
                led_strip_clear(led_strip);
                led_strip_refresh(led_strip);
                gpio_set_level(KUBE_POWER_PIN, 0);
                while(1) vTaskDelay(100);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void task_cube(void *pvParam) {
    init_hardware();
    
    uint16_t packetSize = mpu.dmpGetFIFOPacketSize();
    uint8_t fifoBuffer[64];
    Quaternion q_raw;

    Vec3 v_up={0,0,1}; Vec3 v_down={0,0,-1};
    Vec3 v_front={1,0,0}; Vec3 v_back={-1,0,0};
    Vec3 v_right={0,1,0}; Vec3 v_left={0,-1,0};

    uint32_t last_move_time = xTaskGetTickCount();
    uint32_t last_anim_time = 0; 
    uint32_t last_idle_change = 0; 
    
    // Переменные DICE
    uint32_t shake_start_time = 0; 
    uint32_t demo_timer = 0;
    uint32_t chaos_anim_timer = 0; 
    int demo_digit = 1;            
    int dice_demo_substate = 0; 
    int demo_color_idx = 0;     
    
    // Переменные YESNO
    uint32_t yesno_timer = 0;
    uint32_t yesno_blink_timer = 0;
    bool yesno_toggle = false; 
    int yesno_state = 0; 
    
    Quat q_prev = {0,0,0,0};
    Quat q_displayed = {1,0,0,0};
    CubeMode prev_loop_mode = MODE_GRAVITY;
    
    int dice_state = 0; 
    int idle_effect_type = 0; 

    float current_global_bright = BRIGHT_ACTIVE;
    bool snap_animation = false;

    while(1) {
        if (is_shutdown) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        uint16_t fifoCount = mpu.getFIFOCount();
        if (fifoCount < packetSize) { vTaskDelay(1); continue; }
        if (fifoCount >= 1024) { mpu.resetFIFO(); continue; }
        while (fifoCount >= packetSize) { mpu.getFIFOBytes(fifoBuffer, packetSize); fifoCount-=packetSize; }
        mpu.dmpGetQuaternion(&q_raw, fifoBuffer);
        
        if (q_raw.w==0 && q_raw.x==0) continue;

        Quat q_target = {q_raw.w, q_raw.x + OFFSET_X, q_raw.y + OFFSET_Y, q_raw.z + OFFSET_Z};
        quat_normalize(&q_target);
        
        if (current_mode != prev_loop_mode) {
            if (current_mode == MODE_DICE) {
                dice_state = 0; 
                demo_timer = xTaskGetTickCount();
                demo_digit = 1;
                dice_demo_substate = 0; 
                demo_color_idx = esp_random() % 3; 
            }
            else if (current_mode == MODE_YESNO) {
                yesno_state = 0; 
                yesno_timer = xTaskGetTickCount();
                yesno_blink_timer = xTaskGetTickCount();
                yesno_toggle = false;
            }
            prev_loop_mode = current_mode;
            last_move_time = xTaskGetTickCount(); 
        }

        uint32_t now = xTaskGetTickCount();
        snap_animation = false; 
        
        float motion = fabs(q_target.w - q_prev.w) + fabs(q_target.x - q_prev.x) + fabs(q_target.y - q_prev.y) + fabs(q_target.z - q_prev.z);
        motion *= 1000;
        q_prev = q_target;

        // ========================= GRAVITY =========================
        if (current_mode == MODE_GRAVITY) {
            if (motion > MOVE_THRESHOLD) last_move_time = now;

            bool is_idle = (now - last_move_time) > pdMS_TO_TICKS(IDLE_TIMEOUT_MS);
            
            static bool was_idle = false;
            if (is_idle && !was_idle) {
                was_idle = true;
                idle_effect_type = esp_random() % 4; 
                last_idle_change = now;
                if (idle_effect_type == 1 || idle_effect_type == 2) {
                    for(int i=0; i<TOTAL_LEDS; i++) target_leds[i] = (Color){0,0,0};
                }
            } else if (!is_idle) {
                was_idle = false;
            }

            if (is_idle && (now - last_idle_change > pdMS_TO_TICKS(IDLE_CHANGE_INTERVAL_MS))) {
                int next_type = esp_random() % 4;
                while(next_type == idle_effect_type) next_type = esp_random() % 4;
                idle_effect_type = next_type;
                last_idle_change = now;
                if (idle_effect_type == 0) for(int f=0; f<FACES_COUNT; f++) fill_face_solid(f, PALETTE[f]);
                else if (idle_effect_type == 1 || idle_effect_type == 2) for(int i=0; i<TOTAL_LEDS; i++) target_leds[i] = (Color){0,0,0};
            }

            float target_bri = is_idle ? BRIGHT_IDLE : BRIGHT_ACTIVE;
            if (current_global_bright < target_bri) current_global_bright += 0.5f;
            if (current_global_bright > target_bri) current_global_bright -= 0.5f;

            if (is_idle) {
                if (idle_effect_type == 0) { 
                    if (now - last_anim_time > pdMS_TO_TICKS(SCRAMBLE_SPEED_MS)) { scramble_move(); last_anim_time = now; }
                } 
                else if (idle_effect_type == 1) { 
                    if (now - last_anim_time > pdMS_TO_TICKS(MATRIX_SPEED_MS)) { matrix_effect_step(); last_anim_time = now; }
                }
                else if (idle_effect_type == 2) { 
                    if (now - last_anim_time > pdMS_TO_TICKS(SPARKLE_SPEED_MS)) { sparkle_effect_step(); last_anim_time = now; }
                }
                else { 
                    breath_effect_fade_in_out(); 
                }
            } else {
                q_displayed = quat_lerp(q_displayed, q_target, INERTIA_FACTOR);
                Quat q_inv = {q_displayed.w, -q_displayed.x, -q_displayed.y, -q_displayed.z};
                Vec3 l_dirs[6];
                l_dirs[0] = rotate_vector(v_up, q_inv); l_dirs[1] = rotate_vector(v_down, q_inv);
                l_dirs[2] = rotate_vector(v_front, q_inv); l_dirs[3] = rotate_vector(v_back, q_inv);
                l_dirs[4] = rotate_vector(v_right, q_inv); l_dirs[5] = rotate_vector(v_left, q_inv);
                Color cols[6] = {WORLD_UP, WORLD_DOWN, WORLD_FRONT, WORLD_BACK, WORLD_RIGHT, WORLD_LEFT};
                
                // >>> НОВАЯ ЛОГИКА СМЕШИВАНИЯ С НОРМАЛИЗАЦИЕЙ <<<
                for(int i=0; i<FACES_COUNT; i++) {
                    Vec3 n = face_normals[i];
                    float weights[6];
                    float total_weight = 0.0f;
                    
                    // 1. Считаем веса для всех направлений
                    for(int w=0; w<6; w++) {
                        float dot = vec_dot(n, l_dirs[w]);
                        
                        // Если совпадение идеальное (> DEADZONE_THRESHOLD), то это победа
                        if (dot > DEADZONE_THRESHOLD) {
                            weights[w] = 1000.0f; // Огромный вес, перебьет все остальное
                        } else if (dot > 0.0f) {
                            // Иначе обычная степень для плавности
                            weights[w] = powf(dot, 4.0f);
                        } else {
                            weights[w] = 0.0f;
                        }
                        total_weight += weights[w];
                    }

                    // 2. Смешиваем и Нормализуем
                    float ra=0, ga=0, ba=0;
                    if (total_weight > 0.001f) {
                        for(int w=0; w<6; w++) {
                            float k = weights[w] / total_weight; // Нормализация (сумма всегда 1.0)
                            ra += cols[w].r * k;
                            ga += cols[w].g * k;
                            ba += cols[w].b * k;
                        }
                    }

                    for(int k=0; k<9; k++) target_leds[i*9+k] = (Color){(uint8_t)ra, (uint8_t)ga, (uint8_t)ba};
                }
            }
        } 
        // ========================= DICE =========================
        else if (current_mode == MODE_DICE) {
            current_global_bright = BRIGHT_DICE;
            
            switch(dice_state) {
                case 0: // DEMO
                    snap_animation = true; 
                    if (dice_demo_substate == 0) {
                        draw_digit_all_faces(demo_digit, GAME_COLORS[demo_color_idx]);
                        if (now - demo_timer > pdMS_TO_TICKS(DICE_DEMO_ON_MS)) {
                            if (demo_digit == 6) {
                                dice_state = 1; 
                            } else {
                                dice_demo_substate = 1; 
                                demo_timer = now;
                            }
                        }
                    } else {
                        for(int i=0; i<TOTAL_LEDS; i++) target_leds[i] = (Color){0,0,0};
                        if (now - demo_timer > pdMS_TO_TICKS(DICE_DEMO_OFF_MS)) {
                            demo_digit++;
                            demo_color_idx = esp_random() % 3; 
                            dice_demo_substate = 0; 
                            demo_timer = now;
                        }
                    }
                    break;

                case 1: // WAIT
                    snap_animation = true; 
                    if (motion > SHAKE_THRESHOLD) {
                        shake_start_time = now;
                        dice_state = 2; 
                        chaos_anim_timer = 0;
                    }
                    break;

                case 2: // CHAOS
                    snap_animation = true; 
                    if (now - chaos_anim_timer > pdMS_TO_TICKS(DICE_CHAOS_SPEED_MS)) {
                        for(int f=0; f<FACES_COUNT; f++) {
                            int c_idx = esp_random() % 3;
                            fill_face_solid(f, GAME_COLORS[c_idx]);
                        }
                        chaos_anim_timer = now;
                    }
                    if ((now - shake_start_time) > pdMS_TO_TICKS(DICE_CHAOS_TIME)) {
                        dice_state = 3; 
                    }
                    break;

                case 3: // RESULT
                    {
                        snap_animation = true;
                        int number = (esp_random() % 6) + 1;
                        int c_idx = esp_random() % 3;
                        draw_digit_all_faces(number, GAME_COLORS[c_idx]);
                        dice_state = 1; 
                    }
                    break;
            }
        }
        // ========================= YES / NO =========================
        else if (current_mode == MODE_YESNO) {
            current_global_bright = BRIGHT_ACTIVE;
            snap_animation = true;

            switch(yesno_state) {
                case 0: // INTRO
                    {
                        uint32_t elapsed = (now - yesno_timer) * portTICK_PERIOD_MS; 
                        if (elapsed > YESNO_INTRO_TIME_MS) {
                            yesno_state = 1;
                        } else {
                            float progress = (float)elapsed / YESNO_INTRO_TIME_MS; 
                            uint32_t period = 40 + (uint32_t)(progress * progress * progress * 800); 
                            
                            if (now - yesno_blink_timer > pdMS_TO_TICKS(period)) {
                                yesno_toggle = !yesno_toggle;
                                yesno_blink_timer = now;
                                Color c = yesno_toggle ? (Color){255,0,0} : (Color){0,255,0};
                                fill_all_faces(c);
                            }
                        }
                    }
                    break;

                case 1: // WAIT
                    fill_all_faces((Color){0, 0, 255});
                    if (motion > SHAKE_THRESHOLD) {
                        yesno_state = 2; 
                        shake_start_time = now;
                        yesno_blink_timer = 0; 
                    }
                    break;

                case 2: // THINKING
                    if (now - yesno_blink_timer > pdMS_TO_TICKS(50)) {
                        yesno_toggle = !yesno_toggle;
                        yesno_blink_timer = now;
                        Color c = yesno_toggle ? (Color){255,0,0} : (Color){0,255,0};
                        fill_all_faces(c);
                    }
                    if ((now - shake_start_time) > pdMS_TO_TICKS(2000)) {
                        yesno_state = 3; 
                        yesno_toggle = (esp_random() % 2) == 0; 
                    }
                    break;

                case 3: // RESULT
                    {
                        Color c = yesno_toggle ? (Color){0,255,0} : (Color){255,0,0}; 
                        fill_all_faces(c);
                        if (motion > SHAKE_THRESHOLD) {
                            yesno_state = 2; 
                            shake_start_time = now;
                        }
                    }
                    break;
            }
        }
        // ========================= FLASHLIGHT =========================
        else if (current_mode == MODE_FLASHLIGHT) {
            current_global_bright = BRIGHT_FLASHLIGHT;
            snap_animation = true;
            for(int i=0; i<TOTAL_LEDS; i++) target_leds[i] = (Color){0,0,0};
            
            fill_face_solid(2, (Color){255,255,255});
            fill_face_solid(4, (Color){255,255,255});
            fill_face_solid(5, (Color){255,255,255});
        }

        // ========================= RENDER =========================
        for(int i=0; i<TOTAL_LEDS; i++) {
            float lerp_speed = 0.15f;
            if (snap_animation) lerp_speed = 1.0f; 
            
            lerp_color(&current_leds[i], target_leds[i], lerp_speed);

            uint8_t r = (uint8_t)(current_leds[i].r * current_global_bright / 255.0f);
            uint8_t g = (uint8_t)(current_leds[i].g * current_global_bright / 255.0f);
            uint8_t b = (uint8_t)(current_leds[i].b * current_global_bright / 255.0f);
            led_strip_set_pixel(led_strip, i, r, g, b);
        }
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

extern "C" void app_main(void) {
    xTaskCreate(task_button, "btn", 2048, NULL, 10, NULL);
    xTaskCreate(task_cube, "cube", 8192, NULL, 5, NULL);
}