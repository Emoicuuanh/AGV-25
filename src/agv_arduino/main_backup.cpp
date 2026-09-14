#include "../../arduino_common_library/CommonLibrary.h"
#include "agv_arduino_define.h"
#include "battery_control.h"
#include "stdlib.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <ModbusRtu.h>
#include <arduino_bridge.h>  // Must to include in last line because it use global variable (function) in another library
#include <fast_led.h>
#include <led_control.h>
#include <stdio.h>
#include <unistd.h>

BatteryControl battery_cal;
/* Global variables  */
bool is_battery_full_after_charge = 0;
int count = 0;
bool rosConnected;
bool POWER_STATUS;                    // Machine power ON, OFF status
bool STARTUP_CMD = 0;                 // Start up machine command
bool STARTUP_CMP = 0;                 // Start up machine complete
bool STARTUP_ERR = 0;                 // Start up machine Error
bool SHUTDOWN_CMD = 0;                // Shutdown machine command
bool SHUTDOWN_CMP = 0;                // Shutdown machine complete
bool SHUTDOWN_ERR = 0;                // Shutdown machine Error
bool AUTO_CHARGING_RELAY_STATUS = 0;  // Relay for auto charge jack ON/OFF

// EEPROM variable
int EE_addr = 0;
// Lift motor variable
int PWM_M1_Valve = 0;         // value output to the PWM (analog out)
int PWM_TargetSpeed = 255;    // taget value PWM
bool LIFT_ENABLE = LIFT_OFF;  // ON/OFF Lift Motor
bool SOFT_START_RUN = 0;      // Soft start running
bool SOFT_START_CMP = 0;      // Soft start complete

//____ Speed Parameter ____
int MinSpeed = 100;
int MaxSpeed = 255;

float oldTimeReadBatery = 0;
float TimeReadBatery = 0;

//____ Time Parameter  ______
unsigned char ResolutionTime = 20;  // (1-255) ms

// Timer Interrupt Parameter
int PWM_TCNT = 0;
int PWM_DIV = 0;
int liftDir = 0;

// IPC Power Parameter
bool IPC_Power;
bool IPC_PowerState = 0;  // Power ON/OFF status
bool lastIPC_Power = SENSOR_OFF;
// uint32_t cycleSend;

// time variable
unsigned long t_auto_charging_jack_connect = millis();
unsigned long tUpdatePower = 0.0;
unsigned long tLogDebug;

// Function Delace
void Liftup_Control(uint32_t, int, int, int);
void ledSwitch(int, int, int, int, int, int);
void ledCharge(int duration, int batt_Capacity, int r, int g, int b, int mode);
float Read_CurrentCharging();
float Read_Parameter(int code);

bool writeable_percent = 0;
void writeBatteryPercent(int value);
bool inChargingRange = 0;

/*
                                        ##
  ####  ###### ##### #    # #####      #  #      #    #   ##   # #    #
 #      #        #   #    # #    #      ##       ##  ##  #  #  # ##   #
  ####  #####    #   #    # #    #     ###       # ## # #    # # # #  #
      # #        #   #    # #####     #   # #    #    # ###### # #  # #
 #    # #        #   #    # #         #    #     #    # #    # # #   ##
  ####  ######   #    ####  #          ###  #    #    # #    # # #    #

*/

void setup()
{
  // Serial.begin(BAUDRATE);
  RS485_Serial.begin(9600);  // mac dinh 9600 khong thay doi duoc
  RS232_SERIAL.begin(BAUDRATE);
  // Serial1.begin(BAUDRATE); // Truong_test
  // analogReference(INTERNAL1V1);
  get_unique_id(RS232_SERIAL);

  pinMode(START_1_PIN, INPUT_PULLUP);
  pinMode(START_2_PIN, INPUT_PULLUP);
  pinMode(EMG_1_PIN, INPUT_PULLUP);
  pinMode(MAN_CHARGING_PIN, INPUT_PULLUP);
  pinMode(AUTO_CHARGING_PIN, INPUT_PULLUP);
  pinMode(AUTO_MAN_SW, INPUT_PULLUP);

  pinMode(LIFT_MIN_SENSOR, INPUT_PULLUP);
  pinMode(LIFT_MAX_SENSOR, INPUT_PULLUP);

  pinMode(LED_START_FORWARD, OUTPUT);
  pinMode(LED_START_BACKWARD, OUTPUT);
  pinMode(LED_STOP_FORWARD, OUTPUT);
  pinMode(LED_STOP_BACKWARD, OUTPUT);
  pinMode(AUTO_CHARGING_RELAY_PIN, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);

  pinMode(LED_SIGNAL_FOR_TURN_LEFT, OUTPUT);
  pinMode(LED_SIGNAL_FOR_TURN_RIGHT, OUTPUT);
  pinMode(LED_SIGNAL_BACK_TURN_LEFT, OUTPUT);
  pinMode(LED_SIGNAL_BACK_TURN_RIGHT, OUTPUT);

  pinMode(LIFT_DIR, OUTPUT);

  FastLedSetup();

  strip.begin();            // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();             // Turn OFF all pixels ASAP
  strip.setBrightness(50);  // Set BRIGHTNESS to about 1/5 (max = 255)

  /* setup timer 1 interrupt */
  cli();  //stop interrupts

  TCCR1A = 0;  // set entire TCCR1A register to 0
  TCCR1B = 0;  // same for TCCR1B
  //TCNT1  = 0;//initialize counter value to 0
  // Set CS11 and CS10 bits for 64 prescaler
  TCCR1B |= (1 << CS11) | (1 << CS10);
  // Disable timer Overflow interrupt
  TIMSK1 &= (0 << TOIE1);
  sei();  //allow interrupts

  // Read Battery state
  BattCurrent = battery_cal.readCurrentCharging();
  BattVoltage = battery_cal.readBatteryVoltage();
  if (BattCurrent > -0.5)
  {
    BATT_CHARGE = 1;     // Charging
    BATT_DISCHARGE = 0;  // Not discharging
  }
  else
  {
    BATT_CHARGE = 0;     // not Charging
    BATT_DISCHARGE = 1;  // Discharging
  }
  for (int count = 0; count < 5; count++)
  {
    battery_cal.readBatteryCapacity();
  }
}

void loop()
{
  uint32_t t = millis();

  rosLoop(RS232_SERIAL);
  // Send command to read pin
  float dt = millis() - oldTimeReadBatery;
  oldTimeReadBatery = millis();
  TimeReadBatery += dt;
  if (TimeReadBatery > 500)
  {
    // RS485_Serial.flush() ;

    battery_cal.dataHandle();
    TimeReadBatery = 0.0;
  }
  BatteryCapacityF = static_cast<int>(battery_cal.readBatteryCapacity());
  inChargingRange = battery_cal.inChargingRange();

  // Led
  // TODO: check laster

  if (RS232_SERIAL.available() == 0)
  {
    ledSwitch(Led_State, Led_Duration, Led_Blink_Cycle, Led_R, Led_G, Led_B);
    // RS232_SERIAL1.print("Duration: ");
    // Serial1.println(Led_Duration);
  }
  // When Arduino board disconnect with PC
  if ((t > tLastArduinoRequest) and
      (t - tLastArduinoRequest) >= ARDUINO_CONNECT_TIMEOUT)
  {
    rosConnected = false;
    if ((digitalRead(MAN_CHARGING_PIN) == SENSOR_ON) ||
        ((digitalRead(AUTO_CHARGING_PIN) == SENSOR_ON) &&
         AUTO_CHARGING_RELAY_STATUS))
    {
      ledRGBWrite(CHARGE_MANUAL, 100, 0, 0, 0, 0);
    }
    else
    {
      ledWrite(STARTING_UP);
    }
  }
  else
  {
    rosConnected = true;
  }

  // Enable motor
  digitalWrite(MOTOR_ENABLE_PIN, OUT_ON);

  // Auto charging control
  if (digitalRead(AUTO_CHARGING_PIN) == SENSOR_ON)
  {
    // if (BATTERY_FULL)
    // {
    //   is_battery_full_after_charge = true;
    // }
    // else if (BatteryCapacityF < 80)
    // {
    //   is_battery_full_after_charge = false;
    // }
    if ((millis() > (3000 + t_auto_charging_jack_connect)) && !BATTERY_FULL)
    {
      if (rosConnected)
      // when arduino connect with PC
      {
        if (digitalRead(AUTO_MAN_SW) == SENSOR_OFF &&
            inChargingRange)  // Manual mode
        {
          digitalWrite(AUTO_CHARGING_RELAY_PIN, HIGH);
          AUTO_CHARGING_RELAY_STATUS = true;
        }
        else if (digitalRead(AUTO_MAN_SW) == SENSOR_OFF && !inChargingRange)
        {
          digitalWrite(AUTO_CHARGING_RELAY_PIN, LOW);
          AUTO_CHARGING_RELAY_STATUS = false;
        }
      }
      else
      {
        if (inChargingRange)
        {
          digitalWrite(AUTO_CHARGING_RELAY_PIN, HIGH);
          AUTO_CHARGING_RELAY_STATUS = true;
        }
        else
        {
          digitalWrite(AUTO_CHARGING_RELAY_PIN, LOW);
          AUTO_CHARGING_RELAY_STATUS = false;
        }
      }
    }
    else
    {
      if (!rosConnected)
      {
        digitalWrite(AUTO_CHARGING_RELAY_PIN, LOW);
        AUTO_CHARGING_RELAY_STATUS = false;
      }
    }
  }
  else
  {
    digitalWrite(AUTO_CHARGING_RELAY_PIN, LOW);
    AUTO_CHARGING_RELAY_STATUS = false;
    t_auto_charging_jack_connect = millis();
    // is_battery_full_after_charge = false;
  }

  // Liftup control
  Liftup_Control(t, 100, 255, 500);

  // LED display battery pecent
  bool EMG_EN = digitalRead(EMG_1_PIN) == SENSOR_ON;
  // if (!EMG_EN)
  // {
  //   while (digitalRead(STOP_1_PIN) == SENSOR_ON ||
  //          digitalRead(STOP_2_PIN) == SENSOR_ON)
  //   {
  //     ledSwitch(CHARGE_READY, 10, 0, 0, 0, 0);
  //   }
  // }

  // Debug
  if (!rosConnected)
  {
    if (t > tLogDebug and t - tLogDebug >= 1000)
    {
      tLogDebug = t;
      float vol = battery_cal.readBatteryVoltage();
      float ampe = battery_cal.readCurrentCharging();
      float percent = battery_cal.readBatteryCapacity();
      RS232_SERIAL.println("---Filtered---");
      RS232_SERIAL.print("Battery %: ");
      RS232_SERIAL.println(percent);
      RS232_SERIAL.print("Battery voltage: ");
      RS232_SERIAL.println(vol);
      RS232_SERIAL.print("Current: ");
      RS232_SERIAL.println(ampe);
      RS232_SERIAL.print("AUTO_CHARGING_PIN: ");
      RS232_SERIAL.println(digitalRead(AUTO_CHARGING_PIN));
      RS232_SERIAL.print("MAN_CHARGING_PIN: ");
      RS232_SERIAL.println(digitalRead(MAN_CHARGING_PIN));
      RS232_SERIAL.print("AUTO_CHARGING_RELAY_PIN: ");
      RS232_SERIAL.println(digitalRead(AUTO_CHARGING_RELAY_PIN));
      RS232_SERIAL.print("BATTERY_FULL: ");
      RS232_SERIAL.println(BATTERY_FULL);
      RS232_SERIAL.print("inChargingRange: ");
      RS232_SERIAL.println(inChargingRange);
      //
      // Serial.println("---Filtered---");
      // Serial.print("Battery %: ");
      // Serial.println(percent);
      // Serial.print("Battery voltage: ");
      // Serial.println(vol);
      // Serial.print("Current: ");
      // Serial.println(ampe);
      // Serial.print("AUTO_CHARGING_PIN: ");
      // Serial.println(digitalRead(AUTO_CHARGING_PIN));
      // Serial.print("MAN_CHARGING_PIN: ");
      // Serial.println(digitalRead(MAN_CHARGING_PIN));
      // Serial.print("AUTO_CHARGING_RELAY_PIN: ");
      // Serial.println(digitalRead(AUTO_CHARGING_RELAY_PIN));
      // Serial.print("BATTERY_FULL: ");
      // Serial.println(BATTERY_FULL);
      // Serial.print("inChargingRange: ");
      // Serial.println(inChargingRange);
    }
  }
}

/*
 ######  #######  #####     ######  ######  ### ######   #####  #######
 #     # #     # #     #    #     # #     #  #  #     # #     # #
 #     # #     # #          #     # #     #  #  #     # #       #
 ######  #     #  #####     ######  ######   #  #     # #  #### #####
 #   #   #     #       #    #     # #   #    #  #     # #     # #
 #    #  #     # #     #    #     # #    #   #  #     # #     # #
 #     # #######  #####     ######  #     # ### ######   #####  #######

*/

void ledWrite(int state)
{
  if (Led_State != state)
  {
    // Serial.println(state);
    NewLedEffectRequest = true;
    NewLedEffectDone = false;
  }
  Led_State = state;
}

void ledRGBWrite(int type, int duration, int blink_cycle, int r, int g, int b)
{
  if (Led_R != r or Led_G != g or Led_B != b or type != Led_State or
      duration != Led_Duration or blink_cycle != Led_Blink_Cycle)
  {
    NewLedEffectRequest = true;
    NewLedEffectDone = false;
  }

  Led_State = type;
  Led_R = r;
  Led_G = g;
  Led_B = b;
  Led_Duration = duration;
  Led_Blink_Cycle = blink_cycle;
}

/*
 #       ### ####### #######    #     # ######
 #        #  #          #       #     # #     #
 #        #  #          #       #     # #     #
 #        #  #####      #       #     # ######
 #        #  #          #       #     # #
 #        #  #          #       #     # #
 ####### ### #          #        #####  #

*/
/* Timer Interrupt */
ISR(TIMER1_OVF_vect)
{
  // generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  TCNT1 = PWM_TCNT;  // Reload value
  if (LIFT_ENABLE == LIFT_ON)
  {
    PWM_M1_Valve += PWM_DIV;
    if (PWM_M1_Valve > PWM_TargetSpeed)
    {
      PWM_M1_Valve = PWM_TargetSpeed;  // Keep speed
      SOFT_START_RUN = 0;              // Soft start Done!
      SOFT_START_CMP = 1;
      TIMSK1 &= (0 << TOIE1);  // Disable Timer 1 interrupt
    }
  }
  analogWrite(PWM_M1, PWM_M1_Valve);
  // Serial.print(" PWM Value = ");
  // Serial.println(PWM_M1_Valve);
}

/* Lift Motor RUN */
void runLiftMotor(bool _liftDir, int _StartSpeed)
{
  digitalWrite(LIFT_DIR, _liftDir);  // DIR
  // ENABLE LIFT
  LIFT_ENABLE = LIFT_ON;

  // On PWM
  if (!SOFT_START_RUN && !SOFT_START_CMP)
  {
    PWM_M1_Valve = _StartSpeed;
    analogWrite(PWM_M1, PWM_M1_Valve);  // PWM
    TCNT1 = PWM_TCNT;                   // Duty update PWM value time
    // enable timer Overflow interrupt
    TIMSK1 |= (1 << TOIE1);
    SOFT_START_RUN = 1;
    SOFT_START_CMP = 0;
  }
}

/* Lift Motor OFF */
void stopLiftMotor()
{
  LIFT_ENABLE = LIFT_OFF;
  PWM_M1_Valve = 0;  // off PWM
  analogWrite(PWM_M1, PWM_M1_Valve);
  TIMSK1 &= (0 << TOIE1);  // Disable Timer 1 interrupt
  SOFT_START_CMP = 0;
  SOFT_START_RUN = 0;
  // Serial.println("OFF");
}

/* Lift control*/
// uint32_t tBeginLiftMaxHitByCmd = 0;
// uint32_t tBeginLiftMinHitByCmd = 0;
void Liftup_Control(uint32_t t, int StartSpeed = 100, int TargetSpeed = 255,
                    int SoftStartTime = 1000)
{
  // Limit parameter input
  if (StartSpeed < MinSpeed)
    StartSpeed = MinSpeed;
  if (TargetSpeed > MaxSpeed)
    TargetSpeed = MaxSpeed;

  // result for timer parameter
  PWM_TargetSpeed = TargetSpeed;
  PWM_TCNT =
      65536 - ResolutionTime * 250;  // Reload value for timer 250*4us = 1ms
  PWM_DIV = (TargetSpeed - StartSpeed) / (SoftStartTime / ResolutionTime);

  // Display LED
  if (digitalRead(START_1_PIN) == SENSOR_ON)
    digitalWrite(LED_START_FORWARD, LED_ON);
  else
    digitalWrite(LED_START_FORWARD, LED_OFF);
  if (digitalRead(STOP_1_PIN) == SENSOR_ON)
    digitalWrite(LED_STOP_FORWARD, LED_ON);
  else
    digitalWrite(LED_STOP_FORWARD, LED_OFF);
  if (digitalRead(START_2_PIN) == SENSOR_ON)
    digitalWrite(LED_START_BACKWARD, LED_ON);
  else
    digitalWrite(LED_START_BACKWARD, LED_OFF);
  if (digitalRead(STOP_2_PIN) == SENSOR_ON)
    digitalWrite(LED_STOP_BACKWARD, LED_ON);
  else
    digitalWrite(LED_STOP_BACKWARD, LED_OFF);

  // Liftup control
  bool LIFT_MAX_PUSH = digitalRead(START_1_PIN) == SENSOR_ON ||
                       digitalRead(START_2_PIN) == SENSOR_ON;
  bool LIFT_MIN_PUSH = digitalRead(STOP_1_PIN) == SENSOR_ON ||
                       digitalRead(STOP_2_PIN) == SENSOR_ON;
  bool EMG_EN = digitalRead(EMG_1_PIN) == SENSOR_ON;
  bool AU_MAN_SEL =
      digitalRead(AUTO_MAN_SW) == SENSOR_ON;  // Auto : 1 ; Manual : 0

  bool MAX_LIMIT = digitalRead(LIFT_MAX_SENSOR);
  bool MIN_LIMIT = digitalRead(LIFT_MIN_SENSOR);
  bool limitMaxHold = false;  // Deactivate
  bool limitMinHold = false;  // Deactivate
  static bool lastMaxLimit = MAX_LIMIT;
  static bool lastMinLimit = MIN_LIMIT;
  static uint32_t tBeginMaxHit = 0;
  static uint32_t tBeginMinHit = 0;
  if (MAX_LIMIT && !lastMaxLimit)
  {
    tBeginMaxHit = t;
  }
  if (MIN_LIMIT && !lastMinLimit)
  {
    tBeginMinHit = t;
  }
  lastMaxLimit = MAX_LIMIT;
  lastMinLimit = MIN_LIMIT;
  // Reset hold if button was released
  if (MIN_LIMIT && !LIFT_MIN_PUSH)
  {
    tBeginMinHit = 0;
  }
  if (MAX_LIMIT && !LIFT_MAX_PUSH)
  {
    tBeginMaxHit = 0;
  }
  // Hold after hit position sensor
  if (LIFT_MAX_PUSH && MAX_LIMIT &&
      (t >= tBeginMaxHit && t - tBeginMaxHit < LIFT_LIMIT_DELAY ||
       t < tBeginMaxHit && t + MAX_UINIT32_T - tBeginMaxHit < LIFT_LIMIT_DELAY))
  {
    limitMaxHold = true;
  }
  if (LIFT_MIN_PUSH && MIN_LIMIT &&
      (t >= tBeginMinHit && t - tBeginMinHit < LIFT_LIMIT_DELAY ||
       t < tBeginMinHit && t + MAX_UINIT32_T - tBeginMinHit < LIFT_LIMIT_DELAY))
  {
    limitMinHold = true;
  }
  // Reset hold if button release
  if (limitMinHold && !LIFT_MIN_PUSH)
  {
    limitMinHold = false;
  }
  if (limitMaxHold && !LIFT_MAX_PUSH)
  {
    limitMaxHold = false;
  }

  // Serial.print("Lift up: ");
  // Serial.println(LIFT_MAX_PUSH);
  // Serial.print("Lift down: ");
  // Serial.println(LIFT_MIN_PUSH);
  // Serial.print("EMG OK: ");
  // Serial.println(EMG_EN);
  if (LIFT_MAX_PUSH && !LIFT_MIN_PUSH && EMG_EN && !AU_MAN_SEL &&
      (!MAX_LIMIT || limitMaxHold))
  {
    runLiftMotor(LIFT_UP, StartSpeed);
    // Serial.println("UP");
  }
  else if (LIFT_MIN_PUSH && !LIFT_MAX_PUSH && EMG_EN && !AU_MAN_SEL &&
           (!MIN_LIMIT || limitMinHold))
  {
    runLiftMotor(LIFT_DOWN, StartSpeed);
    // Serial.println("DOWN");
  }
  else
  {
    if ((t > (tLiftupTimeout + CONTROL_LIFTUP_TIMEOUT)) ||
        !EMG_EN)  // Over time or EMG
    {
      // OFF Lift motor
      stopLiftMotor();
      tLiftupTimeout = t;
      // Reset hold if button was released
      // tBeginLiftMaxHitByCmd = 0;
      // tBeginLiftMinHitByCmd = 0;
      // Serial.println("Time");
      // Serial.println(t);
      // Serial.println(tLiftupTimeout);
      // Serial.println(t > (tLiftupTimeout + CONTROL_LIFTUP_TIMEOUT));
      // Serial.println(!EMG_EN);
    }
  }
}

void Lift_Cart(int cmd)
{
  uint32_t t = millis();
  tLiftupTimeout = t;
  // Serial.println("LC");
  if (digitalRead(START_1_PIN) == SENSOR_ON ||
      digitalRead(START_2_PIN) == SENSOR_ON ||
      digitalRead(STOP_1_PIN) == SENSOR_ON ||
      digitalRead(STOP_2_PIN) == SENSOR_ON)
    return;

  // Set Parameter default
  int StartSpeed = MinSpeed;
  int TargetSpeed = MaxSpeed;  // Max speed
  int SoftStartTime = 500;     // ms
  // result for timer parameter
  PWM_TargetSpeed = TargetSpeed;
  PWM_TCNT =
      65536 - ResolutionTime * 250;  // Reload value for timer 250*4us = 1ms
  PWM_DIV = (TargetSpeed - StartSpeed) / (SoftStartTime / ResolutionTime);

  bool EMG_EN = digitalRead(EMG_1_PIN) == SENSOR_ON;
  bool MAX_LIMIT = digitalRead(LIFT_MAX_SENSOR);
  bool MIN_LIMIT = digitalRead(LIFT_MIN_SENSOR);
  bool limitMaxHold = false;  // Deactivate
  bool limitMinHold = false;  // Deactivate
  static bool lastMaxLimit = MAX_LIMIT;
  static bool lastMinLimit = MIN_LIMIT;
  static uint32_t tBeginLiftMaxHitByCmd = 0;
  static uint32_t tBeginLiftMinHitByCmd = 0;

  if (MAX_LIMIT && !lastMaxLimit)
  {
    tBeginLiftMaxHitByCmd = t;
  }
  if (MIN_LIMIT && !lastMinLimit)
  {
    tBeginLiftMinHitByCmd = t;
  }
  lastMaxLimit = MAX_LIMIT;
  lastMinLimit = MIN_LIMIT;

  if (cmd == LIFT_MAX_POSITION && MAX_LIMIT &&
      (t >= tBeginLiftMaxHitByCmd &&
           t - tBeginLiftMaxHitByCmd < LIFT_LIMIT_DELAY ||
       t < tBeginLiftMaxHitByCmd &&
           t + MAX_UINIT32_T - tBeginLiftMaxHitByCmd < LIFT_LIMIT_DELAY))
  {
    limitMaxHold = true;
  }
  if (cmd == LIFT_MIN_POSITION && MIN_LIMIT &&
      (t >= tBeginLiftMinHitByCmd &&
           t - tBeginLiftMinHitByCmd < LIFT_LIMIT_DELAY ||
       t < tBeginLiftMinHitByCmd &&
           t + MAX_UINIT32_T - tBeginLiftMinHitByCmd < LIFT_LIMIT_DELAY))
  {
    limitMinHold = true;
  }

  if (cmd == LIFT_MAX_POSITION && (!MAX_LIMIT || limitMaxHold) &&
      digitalRead(EMG_1_PIN) == SENSOR_ON)
  {
    runLiftMotor(LIFT_UP, StartSpeed);
    // Serial.println("MAX");
  }
  else if (cmd == LIFT_MIN_POSITION && (!MIN_LIMIT || limitMinHold) &&
           digitalRead(EMG_1_PIN) == SENSOR_ON)
  {
    runLiftMotor(LIFT_DOWN, StartSpeed);
    // Serial.println("MIN");
  }
  else
  {
    stopLiftMotor();
    SOFT_START_CMP = 0;
    SOFT_START_RUN = 0;
  }
}

/*
 #       ####### ######
 #       #       #     #
 #       #       #     #
 #       #####   #     #
 #       #       #     #
 #       #       #     #
 ####### ####### ######

*/

void ledSwitch(int type, int duration, int blink_interval, int r, int g, int b)
{
  switch (type)
  {
  case STARTING_UP:
    // Serial.println( "__Start");
    // Serial.println( NewLedEffectRequest);
    // Serial.println( NewLedEffectDone);
    if (!NewLedEffectDone)
    {
      if (myColorWipe(WHITE, 0))
      {
        NewLedEffectDone = true;
        sendLog("Led STARTING_UP done");
        // Serial.println("LED STARTING UP Done");
      }
    }
    break;
  case SHUTING_DOWN:
    if (!NewLedEffectDone)
    {
      if (myColorWipe(ORANGE, 0))
      {
        NewLedEffectDone = true;
        sendLog("Led SHUTING_DOWN done");
      }
    }
    break;
  case RAINBOW:
    if (myRainbow(10))
    {
      sendLog("Led RAINBOW done");
    }
    break;
  case CUSTOM:
    // Only change color
    if (blink_interval == 0)
    {
      if (!NewLedEffectDone)
      {
        if (myColorWipe(strip.Color(r, g, b), duration))
        {
          NewLedEffectDone = true;
          sendLog("Led RGB done");
        }
      }
    }
    // Blink
    if (blink_interval > 0)
    {
      blinkLed(duration, blink_interval, r, g, b);
    }
    break;
  case FAST_1:
    currentPalette = RainbowColors_p;
    currentBlending = LINEARBLEND;
    break;
  case FAST_2:
    currentPalette = RainbowStripeColors_p;
    currentBlending = NOBLEND;
    break;
  case FAST_3:
    currentPalette = RainbowStripeColors_p;
    currentBlending = LINEARBLEND;
    break;
  case FAST_4:
    SetupPurpleAndGreenPalette();
    currentBlending = LINEARBLEND;
    break;
  case FAST_5:
    SetupTotallyRandomPalette();
    currentBlending = LINEARBLEND;
    break;
  case FAST_6:
    SetupBlackAndWhiteStripedPalette();
    currentBlending = NOBLEND;
  case FAST_7:
    SetupBlackAndWhiteStripedPalette();
    currentBlending = LINEARBLEND;
    break;
  case FAST_8:
    currentPalette = CloudColors_p;
    currentBlending = LINEARBLEND;
    break;
  case FAST_9:
    currentPalette = CloudColors_p;
    currentBlending = LINEARBLEND;
    break;
  case FAST_10:
    currentPalette = PartyColors_p;
    currentBlending = LINEARBLEND;
    break;
  case FAST_11:
    currentPalette = myRedWhiteBluePalette_p;
    currentBlending = NOBLEND;
    break;
  case FAST_12:
    currentPalette = myRedWhiteBluePalette_p;
    currentBlending = LINEARBLEND;
    break;
  case CHARGE_MANUAL:
    // BatteryCapacityF = 100; // fake full battery
    ledCharge(duration, BatteryCapacityF, r, g, b,
              CHARGE_MANUAL);  // call with duration and battery capacity
    break;
  case CHARGE_AUTO:
    // BatteryCapacityF = 100; // fake full battery
    ledCharge(duration, BatteryCapacityF, r, g, b,
              CHARGE_AUTO);  // call with duration and battery capacity
    break;
  case CHARGE_READY:
    // BatteryCapacityF = 100; // fake full battery
    ledCharge(
        duration, BatteryCapacityF, r, g, b,
        CHARGE_READY);  // call with duration and battery capacity BatteryCapacityF
    break;

  default:
    break;
  }

  if (Led_State >= FAST_1 and Led_State <= FAST_12)
  {
    FastLedLoop();
  }
}

// LED for Charge
void ledCharge(int _duration, int batt_Capacity, int r, int g, int b, int mode)
{
  int duration = 100;
  int Batt_Status;
  int Led_Display_Num;
  int LedStart, LedEnd, LedOffset, LedInvLength;

  // Calculate Battery Status
  batt_Capacity = constrain(batt_Capacity, 0, 100);
  if (batt_Capacity <= BATT_LOW)
    Batt_Status = 1;  // Battery Low
  if ((batt_Capacity > BATT_LOW) && (batt_Capacity < BATT_HIGH))
    Batt_Status = 2;  // Battery Mid
  if (batt_Capacity >= BATT_HIGH)
    Batt_Status = 3;  // Battery Full

  // Calculate led display number
  Led_Display_Num = LED_BATT_MIN + (batt_Capacity - BATT_LOW) *
                                       (LED_LEFT_NUM - LED_BATT_MIN) /
                                       (BATT_FULL - BATT_LOW);
  LedStart = LED_FRONT_NUM;
  LedEnd = LED_FRONT_NUM + Led_Display_Num - 1;
  LedOffset = LED_LEFT_NUM + LED_BACK_NUM;
  LedInvLength = LED_LEFT_NUM;

  if (!NewLedEffectDone)
  {
    switch (Batt_Status)
    {
    case 1:
      // Slect color
      switch (mode)
      {
      case CHARGE_MANUAL:
        r = 255, g = 0, b = 0;
        break;
      case CHARGE_AUTO:
        r = 255, g = 0, b = 0;
        break;
      case CHARGE_READY:
        r = 255, g = 0, b = 0;
        break;
      default:
        break;
      }
      if (myColorIndex(strip.Color(r, g, b), duration, LedStart, LedEnd,
                       LedOffset, 1, LedInvLength))
      {
        NewLedEffectDone = true;
        sendLog("Battery is LOW");
      }
      break;
    case 2:
      // Slect color
      switch (mode)
      {
      case CHARGE_MANUAL:
        r = 0, g = 0, b = 255;  // Blue
        break;
      case CHARGE_AUTO:
        r = 255, g = 102, b = 0;  // Orange
        break;
      case CHARGE_READY:
        r = 0, g = 255, b = 0;  // Green
        break;
      default:
        break;
      }

      if (myColorIndex(strip.Color(r, g, b), duration, LedStart, LedEnd,
                       LedOffset, 1, LedInvLength))
      {
        NewLedEffectDone = true;
        // sendLog("Battery is MID");
      }
      break;
    case 3:  // Full battery
      // Slect color
      switch (mode)
      {
      case CHARGE_MANUAL:
        r = 0, g = 0, b = 255;
        break;
      case CHARGE_AUTO:
        r = 0, g = 255, b = 0;
        break;
      case CHARGE_READY:
        r = 0, g = 255, b = 0;
        break;
      default:
        break;
      }
      blinkLed(0, 500, r, g, b);
      // {
      //   NewLedEffectDone = true;
      //   sendLog("Battery is FULL");
      // }
      break;
    default:
      break;
    }
  }
  else
  {
    if (offAllLed(200))
    {
      NewLedEffectDone = false;
    }
  }
}

void setTurnSignal(int front_left, int front_right, int rear_left,
                   int rear_right)
{
  digitalWrite(LED_SIGNAL_FOR_TURN_LEFT, front_left);
  digitalWrite(LED_SIGNAL_FOR_TURN_RIGHT, front_right);
  digitalWrite(LED_SIGNAL_BACK_TURN_LEFT, rear_left);
  digitalWrite(LED_SIGNAL_BACK_TURN_RIGHT, rear_right);
}

// Read parameter
float Read_Parameter(int code)
{
  switch (code)
  {
  case ARG_BATT_CAPA:
    return battery_cal.readBatteryCapacity();
    break;
  case ARG_BATT_VOLT:
    return battery_cal.readBatteryVoltage();
    break;
  case ARG_BATT_CURR:
    return battery_cal.readCurrentCharging();
    break;
  default:
    break;
  }
}

// void writeBatteryPercent(int value) {
//   writeable_percent = 1;
//   // RS485_Serial.begin(9600);
//   if (value >= 99)
//   {
//     byte battery_percent_command[13] = {0xA5, 0x40, 0x21, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE8, 0xF9};
//   }
//   else if (value >= 90 )
//   {
//     byte battery_percent_command[13] = {0xA5, 0x40, 0x21, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x84, 0x95};
//   }
//   else if (value >= 75 )
//   {
//     byte battery_percent_command[13] = {0xA5, 0x40, 0x21, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xEE, 0xFE};
//   }
//   else if (value >= 50 )
//   {
//     byte battery_percent_command[13] = {0xA5, 0x40, 0x21, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF4, 0x03};
//   }
//   else if (value >= 25 )
//   {
//     byte battery_percent_command[13] = {0xA5, 0x40, 0x21, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFA, 0x08};
//   }
//   else
//   {
//     byte battery_percent_command[13] = {0xA5, 0x40, 0x21, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E};
//   }
//     // RS485_Serial.write(battery_percent_command, 13);
// }
