#pragma once
#include "PCA9685.h"
#include "RCReceiver.h"
#include <math.h>

#define SERVO_BASE        0
#define SERVO_SHOULDER    1
#define SERVO_ELBOW       2
#define SERVO_WRIST       3
#define SERVO_GRIP        4

#define BASE_HEIGHT       10.0f
#define UPPER_ARM_LENGTH  10.5f
#define FOREARM_LENGTH    10.0f
#define HAND_LENGTH       12.0f

static float currentHorizontalReach = 15.0f;
static float currentVerticalHeight = 10.0f;
static float currentBaseRotationDegree = 90.0f;


static void Arm_setAngle(int servoChannel, float angleDegree)
{
    if (angleDegree < 0.0f)
    {
        angleDegree = 0.0f;
    }
    if (angleDegree > 180.0f)
    {
        angleDegree = 180.0f;
    }

    int pulseLength = 150 + (int)(angleDegree * 450.0f / 180.0f);

    i2c_cmd_handle_t commandHandle = i2c_cmd_link_create();
    i2c_master_start(commandHandle);
    i2c_master_write_byte(commandHandle, (PCA_ADDR_ARM << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(commandHandle, 0x06 + 4 * servoChannel, true);
    i2c_master_write_byte(commandHandle, 0, true);
    i2c_master_write_byte(commandHandle, 0, true);
    i2c_master_write_byte(commandHandle, pulseLength & 0xFF, true);
    i2c_master_write_byte(commandHandle, pulseLength >> 8, true);
    i2c_master_stop(commandHandle);
    i2c_master_cmd_begin(I2C_PORT_NUM, commandHandle, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(commandHandle);
}


static void Arm_moveTo(float targetX, float targetY, float targetZ,
                       float wristPitchDegree, float gripDegree)
{
    float wristPitchRadian = wristPitchDegree * M_PI / 180.0f;

    float horizontalDistance = sqrtf(targetX * targetX + targetY * targetY);

    float wristHorizontalPosition = horizontalDistance - (HAND_LENGTH * cosf(wristPitchRadian));
    float wristVerticalPosition = targetZ - (HAND_LENGTH * sinf(wristPitchRadian));

    float baseRotationDegree = atan2f(targetY, targetX) * 180.0f / M_PI;

    float verticalDistanceRelativeToBase = wristVerticalPosition - BASE_HEIGHT;

    float shoulderToWristDistance = sqrtf(
        wristHorizontalPosition * wristHorizontalPosition +
        verticalDistanceRelativeToBase * verticalDistanceRelativeToBase
    );

    float maximumArmReach = UPPER_ARM_LENGTH + FOREARM_LENGTH;
    if (shoulderToWristDistance > maximumArmReach)
    {
        shoulderToWristDistance = maximumArmReach - 0.1f;
    }

    float shoulderAngleOffsetRadian =
        acosf((UPPER_ARM_LENGTH * UPPER_ARM_LENGTH +
               shoulderToWristDistance * shoulderToWristDistance -
               FOREARM_LENGTH * FOREARM_LENGTH) /
              (2.0f * UPPER_ARM_LENGTH * shoulderToWristDistance));

    float elbowInnerAngleRadian =
        acosf((UPPER_ARM_LENGTH * UPPER_ARM_LENGTH +
               FOREARM_LENGTH * FOREARM_LENGTH -
               shoulderToWristDistance * shoulderToWristDistance) /
              (2.0f * UPPER_ARM_LENGTH * FOREARM_LENGTH));

    float wristLineAngleRadian = atan2f(verticalDistanceRelativeToBase, wristHorizontalPosition);

    float shoulderServoDegree =
        (wristLineAngleRadian + shoulderAngleOffsetRadian) * 180.0f / M_PI;

    float elbowServoDegree =
        180.0f - (elbowInnerAngleRadian * 180.0f / M_PI);

    float forearmAbsoluteAngle =
        shoulderServoDegree - (180.0f - elbowServoDegree);

    float wristServoDegree =
        wristPitchDegree - forearmAbsoluteAngle + 90.0f;


    Arm_setAngle(SERVO_BASE, baseRotationDegree + 90.0f);
    Arm_setAngle(SERVO_SHOULDER, shoulderServoDegree);
    Arm_setAngle(SERVO_ELBOW, elbowServoDegree);
    Arm_setAngle(SERVO_WRIST, wristServoDegree);
    Arm_setAngle(SERVO_GRIP, gripDegree);
}


static void Arm_ManualControl()
{
    float reachAdjustment = RC_getSpeed(1) * 0.4f;
    float baseRotationAdjustment = RC_getSpeed(0) * 0.8f;
    float heightAdjustment = RC_getSpeed(2) * 0.4f;

    float gripDegree = RC_getAngle(3);
    float wristPitchDegree = RC_getAngle(4) - 90.0f;

    currentHorizontalReach += reachAdjustment;
    currentVerticalHeight += heightAdjustment;
    currentBaseRotationDegree -= baseRotationAdjustment;

    if (currentHorizontalReach < 5.0f)
    {
        currentHorizontalReach = 5.0f;
    }
    if (currentHorizontalReach > 30.0f)
    {
        currentHorizontalReach = 30.0f;
    }
    if (currentBaseRotationDegree < 0.0f)
    {
        currentBaseRotationDegree = 0.0f;
    }
    if (currentBaseRotationDegree > 180.0f)
    {
        currentBaseRotationDegree = 180.0f;
    }

    float baseRotationRadian = (currentBaseRotationDegree - 90.0f) * M_PI / 180.0f;

    float targetX = currentHorizontalReach * cosf(baseRotationRadian);
    float targetY = currentHorizontalReach * sinf(baseRotationRadian);

    Arm_moveTo(targetX, targetY, currentVerticalHeight, wristPitchDegree, gripDegree);
}


static void Arm_PickupSequence()
{
    Arm_moveTo(15.0f, 0.0f, 10.0f, -45.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(500));

    Arm_moveTo(15.0f, 0.0f, 0.0f, -90.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(500));

    Arm_moveTo(15.0f, 0.0f, 0.0f, -90.0f, 90.0f);
    vTaskDelay(pdMS_TO_TICKS(500));

    Arm_moveTo(10.0f, 0.0f, 10.0f, 0.0f, 90.0f);
}
