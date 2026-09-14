/* Define function to command with PC.
*/

#ifndef ARDUINO_BRIDGE_H
#define ARDUINO_BRIDGE_H

#include "commands.h"
#include <HardwareSerial.h>
void rosLoop();
void resetCommand();
int runCommand();

static uint32_t tLastArduinoRequest;

#ifdef USE_CMD_VEL
void cmd_vel_write(float lin_vel, float ang_vel);
#endif
#ifdef USE_DUTY_MOTOR
void duty_motor_write(float left_vel, float right_vel);
#endif
#ifdef USE_LED
void ledWrite(int state);
void ledRGBWrite(int type, int duration, int blink_cycle, int r, int g, int b);
#endif
#ifdef USE_ENCODER
void Read_Encoder();
void Reset_Encoder();
#endif
#ifdef USE_CART
void Lift_Cart(int cmd);
#endif
#ifdef USE_READ_PARAM
float Read_Parameter(int code);
#endif
#ifdef USE_TOWER_LAMP
void SetTowerLamp(int red, int green, int yellow, int buzzer, int blink_interval);
#endif
#ifdef USE_TURN_SIGNAL
void setTurnSignal(int front_left, int front_right, int rear_left, int rear_right);
#endif
#ifdef USE_CONVEYOR
void setConveyor(int id, int cmd);
#endif
#ifdef USE_STOPPER
void setStopper(int id, int cmd);
#endif
// Module enable define
bool LedEnable = true;
bool UltraSonicEnable = true;
bool EncoderEnable = true;

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int _index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char sum_arr[5];

// TODO: Use 2D arrays to optimize
char argv1[16];
char argv2[16];
char argv3[16];
char argv4[16];
char argv5[16];
char argv6[16];

// The arguments converted to integers
long arg0;
long arg1;
long arg2;
long arg3;
long arg4;
long arg5;
long arg6;

// Sum check
int sum_calr = 0;
int sum_recv = 0;

// For logger
bool logger_req = false;
bool logger_busy = false;
uint32_t log_key = 0;
String log_msg = "";
String log_confirm = "";

/* Clear the current command parameters */
void resetCommand()
{
   cmd = NULL;
   memset(sum_arr, 0, sizeof(sum_arr));
   memset(argv1, 0, sizeof(argv1));
   memset(argv2, 0, sizeof(argv2));
   memset(argv3, 0, sizeof(argv3));
   memset(argv4, 0, sizeof(argv4));
   memset(argv5, 0, sizeof(argv5));
   memset(argv6, 0, sizeof(argv6));
   arg1 = 0;
   arg2 = 0;
   arg3 = 0;
   arg4 = 0;
   arg5 = 0;
   arg6 = 0;
   arg = 0;
   _index = 0;
   sum_calr = 0;
   sum_recv = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand(HardwareSerial &ser)
{
   static String toprint = "";
   arg1 = atoi(argv1);
   arg2 = atoi(argv2);
   arg3 = atoi(argv3);
   arg4 = atoi(argv4);
   arg5 = atoi(argv5);
   arg6 = atoi(argv6);
   String str;
   tLastArduinoRequest = millis();

   switch(cmd)
   {
   case GET_BAUDRATE:
      ser.println(BAUDRATE);
      break;
   case ANALOG_READ:
      ser.println(analogRead(arg1));
      break;
   case DIGITAL_READ:
      // Also use to keep alive and send log from Arduino
      if (logger_req)
      {
         toprint = "";
         toprint += log_key;
         toprint += '$';
         toprint += log_msg;
         ser.println(toprint);
         logger_req = false;
         log_key = 0;
      }
      else
      {
         ser.println(digitalRead(arg1));
      }
      break;
   case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      ser.println("OK");
      break;
   case DIGITAL_WRITE:
      if (arg2 == 0) digitalWrite(arg1, LOW);
      else if (arg2 == 1) digitalWrite(arg1, HIGH);
      ser.println("OK");
      break;
   case PIN_MODE:
      if (arg2 == 0) pinMode(arg1, INPUT_PULLUP);
      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      ser.println("OK");
      break;
   #ifdef USE_LED
   case LED_WRITE:
      ledWrite(arg1);
      ser.println("OK");
      break;
   case LED_RGB:
      ledRGBWrite(arg1, arg2, arg3, arg4, arg5, arg6);
      ser.println("OK");
      break;
   #endif
   case LOG_CONFIRM:
      log_confirm = "";
      log_confirm = argv1;
      ser.println(log_confirm);
      break;
   case CARD_ID:
      ser.println("0123456789");
      break;
   #ifdef USE_DUTY_MOTOR
   case MOTOR_SPEEDS:
      // str = String("Motor: ");
      // str += atof(argv1);
      // str += " ";
      // str += atof(argv2);
      // ser.println(str);
      duty_motor_write(atoi(argv1), atoi(argv2));
      ser.println("OK");
      break;
   #endif
   #ifdef USE_CMD_VEL
   case CMD_VEL:
      cmd_vel_write(atof(argv1), atof(argv2));
      ser.println("OK");
      break;
   #endif
   #ifdef USE_ENCODER
   case READ_ENCODERS:
      if (EncoderEnable)
      {
         Read_Encoder();
      }
      ser.print(counter_left);
      ser.print(" ");
      ser.println(counter_right);
      break;
   case RESET_ENCODERS:
      Reset_Encoder();
      ser.println("OK");
      break;
   #endif
   #ifdef USE_CART
   case LIFT:
      Lift_Cart(arg1);
      ser.println("OK");
      break;
   #endif
   #ifdef USE_DYP_ULTRA_SONIC
   case READ_ULTRASONIC:
      // TODO: check sizeof
      // if (arg1 > sizeof(UltraSonicData))
      // {
      //    ser.println("Invalid Command");
      //    break;
      // }
      for (int i = 0; i < arg1; i ++)
      {
         if (i < arg1 - 1)
         {
            ser.print(UltraSonicData[i]);
            ser.print(" ");
         }
         else
         {
            ser.println(UltraSonicData[i]);
         }
      }
      break;
   #endif
   case MODULE_SETUP:
      LedEnable = arg1;
      UltraSonicEnable = arg2;
      EncoderEnable = arg3;
      ser.println("OK");
      break;
   // Read parameter
   #ifdef USE_READ_PARAM
   case READ_PARAMETER:
      // ser.println("OK");
      ser.println(float(Read_Parameter(arg1))) ;
      break;
   #endif
   // Tower lamp
   #ifdef USE_TOWER_LAMP
   case TOWER_LAMP:
      SetTowerLamp(arg1, arg2, arg3, arg4, arg5);
      ser.println("OK");
      break;
   #endif
   // Turn signal
   #ifdef USE_TURN_SIGNAL
   case TURN_SIGNAL:
      setTurnSignal(arg1, arg2, arg3, arg4);
      ser.println("OK");
      break;
   #endif
   #ifdef USE_CONVEYOR
   case CONVEYOR:
      void setConveyor(int id, int cmd);
      ser.println("OK");
      break;
   #endif
   #ifdef USE_STOPPER
   case STOPPER:
      void setStopper(int id, int cmd);
      ser.println("OK");
      break;
   #endif
   }
}

void sendLog(String msg)
{
   log_msg = msg;
   logger_req = true;
   log_key = millis();
}

void rosLoop(HardwareSerial &ser)
{
   static String toprint = "";

   while (ser.available() > 0)
   {
      // Read the next character
      chr = ser.read();
      // Serial.print("$");
      // Serial.println(chr,DEC);

      // Terminate a command with a CR
      if (chr == 13)
      {
         sum_recv = atoi(sum_arr);
         sum_calr &= 0x00FF;

         if ( arg== 1)
         {
            sum_arr[_index] = NULL;
         }
         else if (arg == 2)
         {
            argv1[_index] = NULL;
         }
         else if (arg == 3)
         {
            argv2[_index] = NULL;
         }
         else if (arg == 4)
         {
            argv3[_index] = NULL;
         }
         else if (arg == 5)
         {
            argv4[_index] = NULL;
         }
         else if (arg == 6)
         {
            argv5[_index] = NULL;
         }
         else if (arg == 7)
         {
            argv6[_index] = NULL;
         }



         if (sum_recv == sum_calr)
         {
            runCommand(ser);
         }
         else
         {
            // Serial.println(arg);
            // Serial.println(_index);
            // Serial.println(sum_recv);
            // Serial.println(sum_calr);
            ser.println("SUM_ERR");
         }
         resetCommand();
      }
      // Use # to delimit parts of the command
      else if (chr == '#')
      {
         // Step through the arguments
         if (arg == 0)
         {
            arg = 1;
         }
         else if (arg == 1)
         {
            sum_arr[_index] = NULL;
            arg = 2;
            _index = 0;
         }
         else if (arg == 2)
         {
            argv1[_index] = NULL;
            arg = 3;
            _index = 0;
         }
         else if (arg == 3)
         {
            argv2[_index] = NULL;
            arg = 4;
            _index = 0;
         }
         else if (arg == 4)
         {
            argv3[_index] = NULL;
            arg = 5;
            _index = 0;
         }
         else if (arg == 5)
         {
            argv4[_index] = NULL;
            arg = 6;
            _index = 0;
         }
         else if (arg == 6)
         {
            argv5[_index] = NULL;
            arg = 7;
            _index = 0;
         }
         // return;     // use if
         continue;   // use while
      }
      // Receive each arg with more than one character
      else
      {
         // Serial.print("x");
         if (arg == 0)
         {
            // The first arg is the single-letter command
            cmd = chr;
         }
         else if (arg == 1)
         {
            // Subsequent arguments can be more than one character
            sum_arr[_index] = chr;
            _index+=1;
         }
         else if (arg == 2)
         {
            // Subsequent arguments can be more than one character
            argv1[_index] = chr;
            _index+=1;
         }
         else if (arg == 3)
         {
            argv2[_index] = chr;
            _index+=1;
         }
         else if (arg == 4)
         {
            argv3[_index] = chr;
            _index+=1;
         }
         else if (arg == 5)
         {
            argv4[_index] = chr;
            _index+=1;
         }
         else if (arg == 6)
         {
            argv5[_index] = chr;
            _index+=1;
         }
         else if (arg == 7)
         {
            argv6[_index] = chr;
            _index+=1;
         }
         // Calr check sum: cmd + arg1 + arg2 + ...
         if (arg ==0 or arg > 1)
         {
            sum_calr += chr;
            // Serial.println("cal");
         }
      }
   }
}

#endif
