#pragma once
#include "driver/gpio.h"
#include "driver/adc.h"

#define I2C0_SDA          GPIO_NUM_21
#define I2C0_SCL          GPIO_NUM_22
#define I2C0_FREQ         400000
#define I2C0_PORT         I2C_NUM_0

#define I2C1_SDA          GPIO_NUM_17
#define I2C1_SCL          GPIO_NUM_16
#define I2C1_FREQ         400000
#define I2C1_PORT         I2C_NUM_1

#define PCF_ADDR_SENSORS  0x20
#define PCF_ADDR_MOTORS   0x20

#define PWM_LEFT          GPIO_NUM_13
#define PWM_RIGHT         GPIO_NUM_27

#define IR_ANALOG_CHANNEL ADC1_CHANNEL_4 
#define US_TRIGGER_PIN    GPIO_NUM_4
#define US_ECHO_PIN       GPIO_NUM_18
#define STATUS_LED        GPIO_NUM_2

#define OBSTACLE_DISTANCE_CM  15.0f
#define BASE_SPEED        750
#define TURN_SPEED        600
#define REVERSE_SPEED     550
#define MAX_SPEED         1023
#define PID_KP            380.0f
#define PID_KD            120.0f

#ifndef ROBOT_MODE_ENUM
#define ROBOT_MODE_ENUM

enum RobotMode 
{
    MODE_RESET,
    MODE_IDLE,
    MODE_AUTONOMOUS_FORWARD,
    MODE_AUTONOMOUS_REVERSE,
    MODE_MANUAL,
    MODE_RECOVERY 
};
#endif

enum AutonomousForwardStage 
{
    FORWARD_STAGE_STARTING_LANE, 
	FORWARD_STAGE_FINAL_LANE,
    FORWARD_STAGE_ENTER_DESTINATION, 
	FORWARD_STAGE_ARRIVED
};

enum AutonomousReturnStage 
{
    RETURN_STAGE_STARTING_LANE, 
	RETURN_STAGE_TURN_LEFT,
    RETURN_STAGE_FINAL_LANE, 
	RETURN_STAGE_ARRIVED
};

enum RecoveryStage 
{ 
	RECOVERY_REVERSING,
 	RECOVERY_REATTEMPT 
};
