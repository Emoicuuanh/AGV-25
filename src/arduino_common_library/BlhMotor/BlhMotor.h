#ifndef BLH_MOTOR_H_
#define BLH_MOTOR_H_

#include <Adafruit_MCP4725.h>

#define DAC_LEFT_ID 0x60
#define DAC_RIGHT_ID 0x61

Adafruit_MCP4725 DAC_Left;
Adafruit_MCP4725 DAC_Right;

void MotorSetup()
{
    DAC_Left.begin(DAC_LEFT_ID);
    DAC_Right.begin(DAC_RIGHT_ID);
}

void Control_Motor_Left(float vel)
{
    bool dir = FORWARD;
    if (vel < 0)
    {
        dir = BACKWARD;
        vel = abs(vel);
    }
    if (vel > MAX_VEL)
        vel = MAX_VEL;

    digitalWrite(LEFT_DRIVER_INTVR_EXT, EXTERNAL_SPEED);

    if (dir == FORWARD)
    {
        digitalWrite(LEFT_DRIVER_CW_CCW, DIR_CCW);
    }
    else
    {
        digitalWrite(LEFT_DRIVER_CW_CCW, DIR_CW);
    }

    if (abs(vel) < MIN_VEL)
    {
        vel = 0;
        digitalWrite(LEFT_DRIVER_START_STOP, MOTOR_STOP);
        digitalWrite(LEFT_DRIVER_RUN_BRAKE, MOTOR_BREAK);
    }
    else
    {
        digitalWrite(LEFT_DRIVER_START_STOP, MOTOR_START);
        digitalWrite(LEFT_DRIVER_RUN_BRAKE, MOTOR_RUN);
    }
    //analogWrite(LEFT_DRIVER_VRM, (int)(vel * 255 / MAX_VEL));
    DAC_Left.setVoltage((int)(vel * 4095 / MAX_VEL), false);
}

void Control_Motor_Right(float vel)
{
    bool dir = FORWARD;
    if (vel < 0)
    {
        dir = BACKWARD;
        vel = abs(vel);
    }
    if (vel > MAX_VEL)
        vel = MAX_VEL;

    digitalWrite(RIGHT_DRIVER_INTVR_EXT, EXTERNAL_SPEED);

    if (dir == BACKWARD)
    {
        digitalWrite(RIGHT_DRIVER_CW_CCW, DIR_CCW);
    }
    else
    {
        digitalWrite(RIGHT_DRIVER_CW_CCW, DIR_CW);
    }

    if (abs(vel) < MIN_VEL)
    {
        vel = 0;
        digitalWrite(RIGHT_DRIVER_START_STOP, MOTOR_STOP);
        digitalWrite(RIGHT_DRIVER_RUN_BRAKE, MOTOR_BREAK);
    }
    else
    {
        digitalWrite(RIGHT_DRIVER_START_STOP, MOTOR_START);
        digitalWrite(RIGHT_DRIVER_RUN_BRAKE, MOTOR_RUN);
    }
    //analogWrite(RIGHT_DRIVER_VRM, (int)(vel * 255 / MAX_VEL));
    DAC_Right.setVoltage((int)(vel * 4095 / MAX_VEL), false);
}

void Check_Motor_Reset()
{
    static bool prev_left_alarm = MOTOR_NORMAL;
    static bool prev_right_alarm = MOTOR_NORMAL;
    static uint32_t time_left_reset = millis();
    static uint32_t time_right_reset = millis();

    // Reset left motor
    if (digitalRead(LEFT_DRIVER_ALARM) == MOTOR_ALARM and prev_left_alarm == MOTOR_NORMAL)
    {
        digitalWrite(LEFT_DRIVER_ALARM_RESET, MOTOR_RESET);
        time_left_reset = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        Serial.println("Left reset");
    }
    if (digitalRead(LEFT_DRIVER_ALARM_RESET) == MOTOR_RESET and millis() - time_left_reset >= 500)
    {
        digitalWrite(LEFT_DRIVER_ALARM_RESET, MOTOR_RESET_RELEASE);
        Serial.println("Left release");
    }
    prev_left_alarm = digitalRead(LEFT_DRIVER_ALARM);

    if (digitalRead(RIGHT_DRIVER_ALARM) == MOTOR_ALARM and prev_right_alarm == MOTOR_NORMAL)
    {
        digitalWrite(RIGHT_DRIVER_ALARM_RESET, MOTOR_RESET);
        time_right_reset = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        Serial.println("Right reset");
    }
    if (digitalRead(RIGHT_DRIVER_ALARM_RESET) == MOTOR_RESET and millis() - time_right_reset >= 500)
    {
        digitalWrite(RIGHT_DRIVER_ALARM_RESET, MOTOR_RESET_RELEASE);
        Serial.println("Right release");
    }
    prev_right_alarm = digitalRead(RIGHT_DRIVER_ALARM);
}

#endif