#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"
#include "driver/i2c.h"

// ==========================================
// 1. НАСТРОЙКИ ОБОРУДОВАНИЯ
// ==========================================
#define LED_GPIO            8       // Пин светодиодов
#define LEDS_PER_FACE       9       // Светодиодов на одной грани
#define FACES_COUNT         6       // Количество граней
#define TOTAL_LEDS          (LEDS_PER_FACE * FACES_COUNT)
#define LED_BRIGHTNESS      30      // Общая яркость (0-255)

// Пины I2C для ESP32-C3 SuperMini
#define I2C_SDA_PIN         5
#define I2C_SCL_PIN         6
#define I2C_PORT            0
#define ICM_ADDR            0x68    // Адрес акселерометра (проверь 0x68 или 0x69)

// ==========================================
// 2. СТРУКТУРЫ И ЦВЕТА
// ==========================================
typedef struct {
    float x, y, z;
} Vec3;

typedef struct {
    uint8_t r, g, b;
} Color;

// Описание одной грани куба
typedef struct {
    Vec3 normal;    // Куда смотрит грань (в системе координат куба)
    Color native;   // "Родной" цвет грани (когда она сбоку)
    int start_idx;  // Индекс первого светодиода этой грани (0, 9, 18...)
} CubeFace;

// ==========================================
// НОВЫЕ ОПРЕДЕЛЕНИЯ ЦВЕТОВ (Глобальные зоны)
// ==========================================
// Теперь цвета привязаны не к граням, а к НАПРАВЛЕНИЯМ
const Color ZONE_UP     = {255, 0, 0};    // Красный (Потолок)
const Color ZONE_DOWN   = {0, 0, 255};    // Синий (Пол)
const Color ZONE_NORTH  = {0, 255, 0};    // Зеленый (Перед)
const Color ZONE_SOUTH  = {255, 255, 0};  // Желтый (Зад)
const Color ZONE_EAST   = {148, 0, 211};  // Фиолетовый (Право)
const Color ZONE_WEST   = {0, 255, 255};  // Бирюзовый (Лево)

// Описание геометрии куба (нормали жестко заданы конструкцией)
// Порядок в массиве должен совпадать с порядком пайки ленты!
Vec3 face_normals[FACES_COUNT] = {
    {0, 0, 1},   // Грань 0: Верхняя (Z+)
    {0, 0, -1},  // Грань 1: Нижняя (Z-)
    {1, 0, 0},   // Грань 2: Передняя (X+)
    {-1, 0, 0},  // Грань 3: Задняя (X-)
    {0, 1, 0},   // Грань 4: Правая (Y+)
    {0, -1, 0}   // Грань 5: Левая (Y-)
};
// === КОНФИГУРАЦИЯ ГРАНЕЙ (Порядок пайки!) ===
// Здесь ты настраиваешь, какая грань какой является.
// Normal: (x,y,z). Native: цвет. Start_idx: адрес первого диода грани.
// Предполагаем, что лента идет последовательно: Грань 0 -> Грань 1 -> ...
/*CubeFace faces[FACES_COUNT] = {
    // Грань 0 (Z+ Верхняя по умолчанию) - пусть будет Белой, когда сбоку
    { .normal={0,0,1},  .native=COL_WHITE,  .start_idx=18  }, 
    
    // Грань 1 (Z- Нижняя) 
    { .normal={0,0,-1}, .native=COL_WHITE,  .start_idx=27  },

    // Грань 2 (X+ Передняя) - Зеленая
    { .normal={1,0,0},  .native=COL_GREEN,  .start_idx=0 },

    // Грань 3 (X- Задняя) - Желтая
    { .normal={-1,0,0}, .native=COL_YELLOW, .start_idx=9 },

    // Грань 4 (Y+ Правая) - Фиолетовая
    { .normal={0,1,0},  .native=COL_VIOLET, .start_idx=36 },

    // Грань 5 (Y- Левая) - Бирюзовая
    { .normal={0,-1,0}, .native=COL_CYAN,   .start_idx=45 },
};*/

static led_strip_handle_t led_strip;
static const char *TAG = "CUBE";

// ==========================================
// 3. МАТЕМАТИКА
// ==========================================

// Смешивание двух цветов (linear interpolation)
// factor: 0.0 = color1, 1.0 = color2
Color blend_colors(Color c1, Color c2, float factor) {
    if (factor < 0) factor = 0;
    if (factor > 1) factor = 1;
    Color res;
    res.r = (uint8_t)(c1.r + (c2.r - c1.r) * factor);
    res.g = (uint8_t)(c1.g + (c2.g - c1.g) * factor);
    res.b = (uint8_t)(c1.b + (c2.b - c1.b) * factor);
    return res;
}

// Скалярное произведение векторов
float vec_dot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Нормализация вектора (длина = 1)
Vec3 vec_normalize(Vec3 v) {
    float len = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
    if (len < 0.001f) return (Vec3){0,0,1}; // Защита от деления на 0
    return (Vec3){v.x/len, v.y/len, v.z/len};
}

// Векторное произведение (Cross Product)
Vec3 vec_cross(Vec3 a, Vec3 b) {
    return (Vec3){
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

// Сложение цветов с учетом веса (интенсивности)
void add_color_weighted(Color *accum, Color c, float weight) {
    if (weight <= 0) return;
    accum->r = (uint8_t)fmin(255, accum->r + c.r * weight);
    accum->g = (uint8_t)fmin(255, accum->g + c.g * weight);
    accum->b = (uint8_t)fmin(255, accum->b + c.b * weight);
}

// ==========================================
// 4. ДРАЙВЕРЫ (I2C и LED)
// ==========================================

void init_leds(void) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = TOTAL_LEDS,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, 
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
}

void init_i2c(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

// Простая инициализация MPU/ICM
void init_sensor(void) {
    uint8_t cmd_sleep[] = {0x6B, 0x01}; // PWR_MGMT_1: Wake up
    i2c_master_write_to_device(I2C_PORT, ICM_ADDR, cmd_sleep, 2, 100);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // При желании можно настроить чувствительность, но дефолтная ±2g подходит идеально
}

// Чтение ускорения
Vec3 read_accel(void) {
    uint8_t reg = 0x3B; // ACCEL_XOUT_H
    uint8_t data[6];
    
    // Читаем 6 байт подряд
    esp_err_t res = i2c_master_write_read_device(I2C_PORT, ICM_ADDR, &reg, 1, data, 6, 100);
    
    if (res != ESP_OK) {
        return (Vec3){0,0,0}; // Ошибка
    }

    int16_t ax = (data[0] << 8) | data[1];
    int16_t ay = (data[2] << 8) | data[3];
    int16_t az = (data[4] << 8) | data[5];

    // Конвертация в float (значения не важны, важно направление)
    return (Vec3){ (float)ax, (float)ay, (float)az };
}

// ==========================================
// 5. ОСНОВНАЯ ЗАДАЧА
// ==========================================

void cube_task(void *pvParam) {
    init_i2c();
    init_sensor();
    init_leds();

    ESP_LOGI(TAG, "Cube Logic v2 (World Colors) Started!");

    while(1) {
        // 1. Получаем "Сырую" гравитацию (куда показывает сила тяжести)
        Vec3 accel = read_accel();
        Vec3 gravity = vec_normalize(accel);

        // 2. Строим систему координат "МИРА" внутри кубика
        
        // Вектор ВВЕРХ (Против гравитации)
        Vec3 world_up = {-gravity.x, -gravity.y, -gravity.z}; 
        Vec3 world_down = gravity;

        // Вектор СЕВЕР (Горизонтальный).
        // Трюк: Считаем, что Север - это проекция оси X куба на горизонт.
        // Это позволяет "Зеленой" зоне оставаться сбоку при наклонах.
        Vec3 cube_front = {1, 0, 0};
        
        // Математика: Север = Front - (Front * Up) * Up
        float dot_gu = vec_dot(cube_front, world_up);
        Vec3 world_north = {
            cube_front.x - world_up.x * dot_gu,
            cube_front.y - world_up.y * dot_gu,
            cube_front.z - world_up.z * dot_gu
        };
        
        // Защита от деления на 0 (если куб стоит вертикально на носу)
        // В этом случае берем за север ось Y
        if (vec_dot(world_north, world_north) < 0.1) {
             Vec3 cube_side = {0, 1, 0};
             float dot_su = vec_dot(cube_side, world_up);
             world_north.x = cube_side.x - world_up.x * dot_su;
             world_north.y = cube_side.y - world_up.y * dot_su;
             world_north.z = cube_side.z - world_up.z * dot_su;
        }
        world_north = vec_normalize(world_north);

        // Вектор ВОСТОК (Перпендикулярно Северу и Вверх)
        Vec3 world_east = vec_cross(world_north, world_up); 
        // Вектора Юг и Запад - это просто инверсия Севера и Востока

        // 3. Проходим по всем граням и красим их
        for (int i = 0; i < FACES_COUNT; i++) {
            Vec3 normal = face_normals[i];
            
            // Считаем, насколько грань смотрит в каждую из 6 сторон света
            // Результат будет от -1 до 1. Нам нужны только положительные значения (>0)
            float match_up    = vec_dot(normal, world_up);
            float match_down  = vec_dot(normal, world_down);
            float match_north = vec_dot(normal, world_north);
            float match_south = vec_dot(normal, (Vec3){-world_north.x, -world_north.y, -world_north.z});
            float match_east  = vec_dot(normal, world_east);
            float match_west  = vec_dot(normal, (Vec3){-world_east.x, -world_east.y, -world_east.z});

            Color final_color = {0, 0, 0};

            // Смешиваем цвета. Если грань смотрит вверх (match_up=1), она станет полностью Красной.
            // Если она смотрит под углом 45 градусов между Верхом и Севером, 
            // она смешает Красный (0.7) и Зеленый (0.7).
            add_color_weighted(&final_color, ZONE_UP,    match_up);
            add_color_weighted(&final_color, ZONE_DOWN,  match_down);
            add_color_weighted(&final_color, ZONE_NORTH, match_north);
            add_color_weighted(&final_color, ZONE_SOUTH, match_south);
            add_color_weighted(&final_color, ZONE_EAST,  match_east);
            add_color_weighted(&final_color, ZONE_WEST,  match_west);

            // Применяем цвет к диодам грани
            int start_idx = i * LEDS_PER_FACE;
            for (int k = 0; k < LEDS_PER_FACE; k++) {
                 // Яркость
                uint8_t r = (final_color.r * LED_BRIGHTNESS) / 255;
                uint8_t g = (final_color.g * LED_BRIGHTNESS) / 255;
                uint8_t b = (final_color.b * LED_BRIGHTNESS) / 255;
                led_strip_set_pixel(led_strip, start_idx + k, r, g, b);
            }
        }

        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void) {
    xTaskCreate(cube_task, "cube", 4096, NULL, 5, NULL);
}