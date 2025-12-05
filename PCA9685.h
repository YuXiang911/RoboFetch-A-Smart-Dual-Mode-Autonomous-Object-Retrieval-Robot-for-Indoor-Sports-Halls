#pragma once
#include "I2CManager.h"
#include <math.h>

static void PCA_writeReg(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA_ADDR_ARM << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT_NUM, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
}

static void PCA_init()
{
    PCA_writeReg(0x00, 0x00); 
    
    uint8_t prescale = 125; 

    PCA_writeReg(0x00, 0x10); 
    PCA_writeReg(0xFE, prescale); 
    PCA_writeReg(0x00, 0xA0); 
    vTaskDelay(pdMS_TO_TICKS(5)); 
}
