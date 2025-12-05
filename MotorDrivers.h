#pragma once
#include "I2CManager.h"
#include "driver/ledc.h"

inline void Motors_init()
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t chL = { .gpio_num = PWM_LEFT, .speed_mode = LEDC_HIGH_SPEED_MODE, .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0, .flags = {0} };
    ledc_channel_config_t chR = { .gpio_num = PWM_RIGHT, .speed_mode = LEDC_HIGH_SPEED_MODE, .channel = LEDC_CHANNEL_1, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0, .flags = {0} };
    ledc_channel_config(&chL);
    ledc_channel_config(&chR);
    
    I2C_WriteByte(I2C1_PORT, PCF_ADDR_MOTORS, 0x00);
}

inline void Motors_set(int speedL, int speedR)
{
    uint8_t dirs = 0;
    
    if (speedL >= 0)
    {
        dirs |= (1 << 0);
    }
    else
    {
        dirs |= (1 << 1);
        speedL = -speedL;
    }

    if (speedR >= 0)
    {
        dirs |= (1 << 2);
    }
    else
    {
        dirs |= (1 << 3);
        speedR = -speedR;
    }

    if (speedL > 1023) speedL = 1023;
    if (speedR > 1023) speedR = 1023;

    I2C_WriteByte(I2C1_PORT, PCF_ADDR_MOTORS, dirs);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, speedL);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, speedR);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
}

