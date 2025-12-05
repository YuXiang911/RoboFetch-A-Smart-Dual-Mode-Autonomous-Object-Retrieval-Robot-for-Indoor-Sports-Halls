extern "C" 
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

#include "GPIOpins.h"
#include "I2CManager.h"
#include "MotorDrivers.h"
#include "UltrasonicSensor.h"
#include "IRArray.h"
#include "Bluetooth.h"
#include <stdio.h>

volatile RobotMode currentMode = MODE_RESET;
int manualSpeedL = 0, manualSpeedR = 0;
int targetDestination = 1;

static float lastError = 0.0f;
static int RightJunctionDetected = 0;
static int LeftJunctionDetectedF = 0;
static int LeftJunctionDetectedR = 0;
static uint32_t junctionDebounceTimer = 0;
static uint32_t startingDebounceTimer = 0;

static AutonomousForwardStage currentStageF = FORWARD_STAGE_STARTING_LANE;
static AutonomousReturnStage currentStageR = RETURN_STAGE_STARTING_LANE;
static RecoveryStage recoveryStage = RECOVERY_REVERSING;

void pid_control()
{
    float ir_error = IR_getAnalogError();
    float P = ir_error * PID_KP;
    float D = (ir_error - lastError) * PID_KD;
    float output = P + D;
    lastError = ir_error;

    int autoSpeedL = BASE_SPEED + (int)output;
    int autoSpeedR = BASE_SPEED - (int)output;
    
    if(autoSpeedL > MAX_SPEED) autoSpeedL = MAX_SPEED;
    if(autoSpeedL < 0) autoSpeedL = 0;
    if(autoSpeedR > MAX_SPEED) autoSpeedR = MAX_SPEED;
    if(autoSpeedR < 0) autoSpeedR = 0;
    
    Motors_set(autoSpeedL, autoSpeedR);
}

void perform_turn(bool right)
{
    Motors_set(BASE_SPEED, BASE_SPEED);
    vTaskDelay(pdMS_TO_TICKS(150)); 
    
    if(right) Motors_set(TURN_SPEED, -TURN_SPEED);
    else Motors_set(-TURN_SPEED, TURN_SPEED);
    
    vTaskDelay(pdMS_TO_TICKS(350));

    while(1)
    {
        float error = IR_getAnalogError();
        if (error > -0.2f && error < 0.2f) break;
        
        if(right) Motors_set(TURN_SPEED, -TURN_SPEED);
        else Motors_set(-TURN_SPEED, TURN_SPEED);
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    lastError = 0;
}

void control_task(void *pvParams)
{
    TickType_t lastWake = xTaskGetTickCount();
    
    while (1)
    {
        float distance = Ultrasonic_getDistance();
        bool ObstaclesDetected = (distance > 2.0f && distance < OBSTACLE_DISTANCE_CM);

        switch (currentMode)
        {
            case MODE_RESET:
                Motors_set(0, 0);
                RightJunctionDetected = 0;
                LeftJunctionDetectedF = 0;
                LeftJunctionDetectedR = 0;
                currentStageF = FORWARD_STAGE_STARTING_LANE;
                currentStageR = RETURN_STAGE_STARTING_LANE;
                recoveryStage = RECOVERY_REVERSING;
                startingDebounceTimer = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
                currentMode = MODE_IDLE;
                break;
                
            case MODE_IDLE:
                Motors_set(0, 0);
                break;

            case MODE_AUTONOMOUS_FORWARD:
                if (ObstaclesDetected)
                {
                    Motors_set(0, 0);
                    break;
                }

                if (currentStageF == FORWARD_STAGE_STARTING_LANE)
                {
                    pid_control();
                    
                    if (xTaskGetTickCount() > startingDebounceTimer)
                    {
                        if (IR_RightJunction() && (xTaskGetTickCount() - junctionDebounceTimer > pdMS_TO_TICKS(1500)))
                        {
                            perform_turn(true);
                            RightJunctionDetected++;
                            junctionDebounceTimer = xTaskGetTickCount();
                            
                            if (RightJunctionDetected >= 2)
                            {
                                currentStageF = FORWARD_STAGE_FINAL_LANE;
                            }
                        }
                    }
                }
                else if (currentStageF == FORWARD_STAGE_FINAL_LANE)
                {
                    pid_control();
                    
                    if (IR_LeftJunction() && (xTaskGetTickCount() - junctionDebounceTimer > pdMS_TO_TICKS(1500)))
                    {
                        LeftJunctionDetectedF++;
                        junctionDebounceTimer = xTaskGetTickCount();
                        
                        if (LeftJunctionDetectedF == targetDestination)
                        {
                            perform_turn(false);
                            currentStageF = FORWARD_STAGE_ENTER_DESTINATION;
                        }
                    }
                }
                else if (currentStageF == FORWARD_STAGE_ENTER_DESTINATION)
                {
                    pid_control();
                    if (IR_Destination())
                    {
                        Motors_set(BASE_SPEED, BASE_SPEED);
                        vTaskDelay(pdMS_TO_TICKS(1500));
                        currentStageF = FORWARD_STAGE_ARRIVED;
                    }
                }
                else if (currentStageF == FORWARD_STAGE_ARRIVED)
                {
                    Motors_set(0, 0);
                }
                break;
                
            case MODE_AUTONOMOUS_REVERSE:
                if (ObstaclesDetected)
                {
                    Motors_set(0, 0);
                    break;
                }

                if (currentStageR == RETURN_STAGE_STARTING_LANE)
                {
                    pid_control();
                    
                    if (xTaskGetTickCount() > startingDebounceTimer)
                    {
                        if (IR_CrossJunction())
                        {
                            perform_turn(true);
                            currentStageR = RETURN_STAGE_TURN_LEFT;
                        }
                    }   
                }
                else if (currentStageR == RETURN_STAGE_TURN_LEFT)
                {
                    pid_control();
                    
                    if (LeftJunctionDetectedR < 2)
                    {
                        if (IR_LeftJunction() && (xTaskGetTickCount() - junctionDebounceTimer > pdMS_TO_TICKS(1500)))
                        {
                            perform_turn(false);
                            LeftJunctionDetectedR++;
                            junctionDebounceTimer = xTaskGetTickCount();
                        }   
                    } 
                    else
                    {
                        currentStageR = RETURN_STAGE_FINAL_LANE;
                    }
                }
                else if (currentStageR == RETURN_STAGE_FINAL_LANE)
                {
                    pid_control();
                    if (IR_Destination())
                    {
                        Motors_set(BASE_SPEED, BASE_SPEED);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        currentStageR = RETURN_STAGE_ARRIVED;
                    }
                }
                else if (currentStageR == RETURN_STAGE_ARRIVED)
                {
                    Motors_set(0, 0);
                }
                break;

            case MODE_RECOVERY:
                if (recoveryStage == RECOVERY_REVERSING)
                {
                    Motors_set(-REVERSE_SPEED, -REVERSE_SPEED);
                    if (IR_CrossJunction())
                    {
                        Motors_set(0, 0);
                        vTaskDelay(pdMS_TO_TICKS(200));
                        perform_turn(true);
                        recoveryStage = RECOVERY_REATTEMPT;
                    }
                }
                else if (recoveryStage == RECOVERY_REATTEMPT)
                {
                    pid_control();
                    
                    if (targetDestination == 1) LeftJunctionDetectedF = 0;
                    else if (targetDestination == 2) LeftJunctionDetectedF = 1;
                    else if (targetDestination == 3) LeftJunctionDetectedF = 2;
                    
                    junctionDebounceTimer = xTaskGetTickCount();
                    currentStageF = FORWARD_STAGE_FINAL_LANE;
                    currentMode = MODE_AUTONOMOUS_FORWARD;
                }
                break;

            case MODE_MANUAL:
                Motors_set(manualSpeedL, manualSpeedR);
                break;
        }
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(20));
    }
}

extern "C" void app_main()
{
    I2C_init();
    I2C_scan();
    Motors_init();
    IR_init();
    Ultrasonic_init();
    Wireless_init();
    
    printf("ROBOFETCH READY\n");
    startingDebounceTimer = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
    xTaskCreatePinnedToCore(control_task, "Control", 4096, NULL, 5, NULL, 1);
}
