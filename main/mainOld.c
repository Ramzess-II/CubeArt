#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"

// --- НАСТРОЙКИ ---
#define BLINK_GPIO 8        // Пин, к которому подключен DIN ленты
#define LED_STRIP_LEN 12    // Количество светодиодов
#define LED_BRIGHTNESS 20   // Яркость (0-255). Не ставьте максимум при питании от USB!

static const char *TAG = "RAINBOW";

// Глобальный объект ленты
static led_strip_handle_t led_strip;

/**
 * @brief Вспомогательная функция для преобразования HSV в RGB
 * h (0-360) - оттенок
 * s (0-100) - насыщенность
 * v (0-100) - яркость (значение)
 * r, g, b - указатели для записи результата (0-255)
 */
void hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b) {
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;
    uint32_t i = h / 60;
    uint32_t diff = h % 60; // remainder inside sector
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0: *r = rgb_max; *g = rgb_min + rgb_adj; *b = rgb_min; break;
    case 1: *r = rgb_max - rgb_adj; *g = rgb_max; *b = rgb_min; break;
    case 2: *r = rgb_min; *g = rgb_max; *b = rgb_min + rgb_adj; break;
    case 3: *r = rgb_min; *g = rgb_max - rgb_adj; *b = rgb_max; break;
    case 4: *r = rgb_min + rgb_adj; *g = rgb_min; *b = rgb_max; break;
    default: *r = rgb_max; *g = rgb_min; *b = rgb_max - rgb_adj; break;
    }
}

// Функция инициализации ленты через RMT
void configure_led(void) {
    ESP_LOGI(TAG, "Инициализация WS2812B...");
    
    // Конфигурация самой ленты
led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = LED_STRIP_LEN,
        // 1. Указываем модель чипа (обязательно)
        .led_model = LED_MODEL_WS2812, 
        // 2. ИСПРАВЛЕНО: Новое имя поля и новая константа для формата GRB
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, 
        
        .flags.invert_out = false,
    };

    // Конфигурация RMT (бэкенда)
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // Тактирование по умолчанию
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false, // DMA для 10 диодов не обязателен, но для длинных лент полезен
    };

    // Создание объекта ленты
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    
    // Очистка ленты (выключение) при старте
    led_strip_clear(led_strip);
}

void app_main(void) {
    configure_led();

    uint32_t hue = 0;          // Оттенок (0-360)
    uint32_t red, green, blue;

    while (1) {
        // Проходим по всем светодиодам
        for (int i = 0; i < LED_STRIP_LEN; i++) {
            // Чтобы создать эффект "волны", добавляем смещение hue для каждого диода
            // (i * 10) определяет плотность радуги
            uint32_t current_hue = (hue + (i * 360 / LED_STRIP_LEN)) % 360;

            // Конвертируем HSV в RGB
            // Насыщенность 100%, Яркость берем из константы
            hsv2rgb(current_hue, 100, LED_BRIGHTNESS, &red, &green, &blue);

            // Устанавливаем цвет для конкретного пикселя
            led_strip_set_pixel(led_strip, i, red, green, blue);
        }

        // Отправляем данные на ленту (обновляем цвета)
        led_strip_refresh(led_strip);

        // Сдвигаем базовый оттенок для следующего кадра анимации
        hue += 2; // Чем больше число, тем быстрее меняется цвет
        if (hue >= 360) hue = 0;

        // Задержка определяет скорость анимации (20мс = 50 FPS)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}