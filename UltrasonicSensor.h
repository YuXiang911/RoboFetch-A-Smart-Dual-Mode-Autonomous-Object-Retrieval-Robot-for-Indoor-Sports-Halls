#pragma once
#include "driver/gpio.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "GPIOpins.h"

static void Ultrasonic_init()
{
    gpio_set_direction(US_TRIGGER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(US_ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_level(US_TRIGGER_PIN, 0);
}

static float Ultrasonic_getDistance()
{
    gpio_set_level(US_TRIGGER_PIN, 1);
    ets_delay_us(10);
    gpio_set_level(US_TRIGGER_PIN, 0);

    int64_t timeout = esp_timer_get_time() + 20000;
    while (gpio_get_level(US_ECHO_PIN) == 0)
    {
        if (esp_timer_get_time() > timeout) return -1.0;
    }
    int64_t start = esp_timer_get_time();
    timeout = start + 25000;
    while (gpio_get_level(US_ECHO_PIN) == 1)
    {
        if (esp_timer_get_time() > timeout) break;
    }
    int64_t end = esp_timer_get_time();
    float distance = ((end - start) * 0.0343) / 2.0;
    if (distance > 400 || distance < 2) return -1.0;
    return distance;
}
