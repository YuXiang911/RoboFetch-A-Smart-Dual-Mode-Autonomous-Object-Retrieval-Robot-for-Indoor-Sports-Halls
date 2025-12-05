#pragma once
#include <string.h>
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "esp_bt_device.h"
#include "GPIOpins.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"

extern volatile RobotMode currentMode;
extern int manualSpeedL, manualSpeedR;
extern int targetDestination;

static uint32_t connectionHandle = 0;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    if (event == ESP_SPP_INIT_EVT)
    {
        esp_spp_start_srv(sec_mask, role_slave, 0, "SPP_SERVER");
    }
    else if (event == ESP_SPP_SRV_OPEN_EVT)
    {
        connectionHandle = param->srv_open.handle;
        gpio_set_level(STATUS_LED, 1);
    }
    else if (event == ESP_SPP_CLOSE_EVT)
    {
        connectionHandle = 0;
        gpio_set_level(STATUS_LED, 0);
        currentMode = MODE_IDLE;
        manualSpeedL = 0; manualSpeedR = 0;
    }
    else if (event == ESP_SPP_DATA_IND_EVT)
    {
        char command = param->data_ind.data[0];
        
        if (command == 'S') { currentMode = MODE_IDLE; manualSpeedL=0; manualSpeedR=0; }
        else if (command == 'A') currentMode = MODE_AUTONOMOUS_FORWARD;
        else if (command == 'O') currentMode = MODE_AUTONOMOUS_REVERSE;
        else if (command == 'M') currentMode = MODE_MANUAL;
        else if (command == 'W') currentMode = MODE_RECOVERY;
        else if (command == 'C') currentMode = MODE_RESET;

        if (command == '1') targetDestination = 1;
        if (command == '2') targetDestination = 2;
        if (command == '3') targetDestination = 3;

        if (currentMode == MODE_MANUAL)
        {
            if (command == 'F') { manualSpeedL = 700; manualSpeedR = 700; }
            else if (command == 'B') { manualSpeedL = -700; manualSpeedR = -700; }
            else if (command == 'L') { manualSpeedL = -500; manualSpeedR = 500; }
            else if (command == 'R') { manualSpeedL = 500; manualSpeedR = -500; }
            else if (command == 'X') { manualSpeedL = 0; manualSpeedR = 0; }
        }
    }
}

static void Wireless_init()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    gpio_set_direction(STATUS_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(STATUS_LED, 0);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    
    esp_bluedroid_init();
    esp_bluedroid_enable();
    
    esp_bt_dev_set_device_name("ROBOFETCH");
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    
    esp_spp_register_callback(esp_spp_cb);
    
    esp_spp_init(ESP_SPP_MODE_CB);
}
