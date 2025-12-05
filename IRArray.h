#pragma once
#include "GPIOpins.h"
#include "I2CManager.h"
#include "driver/adc.h"
#include <math.h>

inline void IR_init()
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(IR_ANALOG_CHANNEL, ADC_ATTEN_DB_11);
    I2C_WriteByte(I2C0_PORT, PCF_ADDR_SENSORS, 0xFF);
}

inline float IR_getAnalogError()
{
    int raw = adc1_get_raw(IR_ANALOG_CHANNEL);
    float error = ((float)raw - 2048.0f) / 2048.0f;
    if (error < -1.0f) error = -1.0f;
    if (error > 1.0f) error = 1.0f;
    return error;
}

inline uint8_t IR_getDigital()
{
    return I2C_ReadByte(I2C0_PORT, PCF_ADDR_SENSORS);
}

inline bool IR_RightJunction()
{
    uint8_t data = IR_getDigital();
    bool d1 = (data >> 0) & 1;
    bool d2 = (data >> 1) & 1;
    bool d4 = (data >> 3) & 1;
    bool d5 = (data >> 4) & 1;
    return (d4 && d5 && !(d1 || d2));
}

inline bool IR_LeftJunction()
{
    uint8_t data = IR_getDigital();
    bool d1 = (data >> 0) & 1;
    bool d2 = (data >> 1) & 1;
    bool d4 = (data >> 3) & 1;
    bool d5 = (data >> 4) & 1;
    return (d1 && d2 && !(d4 || d5));
}

inline bool IR_CrossJunction()
{
    uint8_t data = IR_getDigital();
    bool d1 = (data >> 0) & 1;
    bool d2 = (data >> 1) & 1;
    bool d4 = (data >> 3) & 1;
    bool d5 = (data >> 4) & 1;
    return (d1 && d2 && d4 && d5);
}

inline bool IR_Destination()
{
    uint8_t data = IR_getDigital();
    bool d1 = (data >> 0) & 1;
    bool d3 = (data >> 2) & 1;
    bool d5 = (data >> 4) & 1;
    if (d1 && d3 && d5) return true;
    return false;
}
