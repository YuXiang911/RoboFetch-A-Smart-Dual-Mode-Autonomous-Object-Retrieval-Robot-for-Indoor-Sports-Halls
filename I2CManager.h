#pragma once
#include "driver/i2c.h"
#include "GPIOpins.h"
#include <stdio.h>

static void I2C_init()
{
    i2c_config_t conf0 = {};
    conf0.mode = I2C_MODE_MASTER;
    conf0.sda_io_num = I2C0_SDA;
    conf0.scl_io_num = I2C0_SCL;
    conf0.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf0.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf0.master.clk_speed = I2C0_FREQ;
    i2c_param_config(I2C0_PORT, &conf0);
    i2c_driver_install(I2C0_PORT, conf0.mode, 0, 0, 0);

    i2c_config_t conf1 = {};
    conf1.mode = I2C_MODE_MASTER;
    conf1.sda_io_num = I2C1_SDA;
    conf1.scl_io_num = I2C1_SCL;
    conf1.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf1.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf1.master.clk_speed = I2C1_FREQ;
    i2c_param_config(I2C1_PORT, &conf1);
    i2c_driver_install(I2C1_PORT, conf1.mode, 0, 0, 0);
}

inline void I2C_WriteByte(i2c_port_t port, uint8_t address, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
}

inline uint8_t I2C_ReadByte(i2c_port_t port, uint8_t address)
{
    uint8_t data = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
    return data;
}

