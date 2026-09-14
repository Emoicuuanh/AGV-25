#ifndef AGV_CONFIG_H_
#define AGV_CONFIG_H_

#include "io_define.h"

/* Serial port baud rate */
#define BAUDRATE 115200
#define MAX_UINIT32_T 4294967295  // 2^32 - 1

// #define OUTPUT_SINK
#define OUTPUT_SOURCE

#ifdef OUTPUT_SINK
#define BREAK 0
#define RELEASE_BREAK 1

#define MOTOR_ENABLE 0
#define MOTOR_DISABLE 1

#define LED_ON 0
#define LED_OFF 1

#define LED_RIGHT_NUM_START 0
#define LED_LEFT_NUM_START 20

#define BATTERY_LEVEL_RED 10
#define BATTERY_LEVEL_YELLOW 80

#define LIFT_ON 0
#define LIFT_OFF 1

#define OUT_ON 0
#define OUT_OFF 1

#define MOTOR_RESET 0
#define MOTOR_RESET_RELEASE 1

#define MOTOR_ALARM 1
#define MOTOR_NORMAL 0
#endif

#ifdef OUTPUT_SOURCE
#define BREAK 1
#define RELEASE_BREAK 0

#define MOTOR_ENABLE 1
#define MOTOR_DISABLE 0

#define LED_ON 1
#define LED_OFF 0

#define LIFT_ON 1
#define LIFT_OFF 0

#define OUT_ON 1
#define OUT_OFF 0

#define MOTOR_RESET 1
#define MOTOR_RESET_RELEASE 0

#define MOTOR_ALARM 0
#define MOTOR_NORMAL 1
#endif

// Output directly from CPU => don't case SINK, SOURCE
#define LIFT_UP 0
#define LIFT_DOWN 1

// Sink input
#define EMG_CASE 1     // NC - sink logic
#define BUMPER_CASE 0  // NO - sink logic
#define SENSOR_ON 0
#define SENSOR_OFF 1

#define LIFT_MAX_POSITION 1
#define LIFT_MIN_POSITION 2
#define LIFT_MID_POSITION 3

#define CONTROL_LIFTUP_TIMEOUT 200

#define FORWARD false
#define BACKWARD true

#define LEFT 0
#define RIGHT 1

// Arduino bridge
#define USE_LED
// #define USE_ENCODER
#define USE_CART
#define USE_READ_PARAM
// #define USE_TOWER_LAMP
// #define USE_CMD_VEL
// #define USE_DYP_ULTRA_SONIC
#define USE_TURN_SIGNAL

#define ARDUINO_CONNECT_TIMEOUT 500
#define SHUTDOWN_HANDLE_TIMEOUT 10000  // 10 secs
#define BUTTON_FILTER_INTERVAL 50
#define UPDATE_POWER_TIME 300
#define EEPROM_SAVE_DATA_TIME 30000
#define LIFT_LIMIT_DELAY 2000
#define RS232_SERIAL Serial1
enum enum_led_status
{
  STARTING_UP,   // 0
  SHUTING_DOWN,  // 1
  RAINBOW,       // 2
  CUSTOM,        // 3
  // All FAST_LED effect must be define in last of this enum
  FAST_1,  // 4
  FAST_2,
  FAST_3,
  FAST_4,
  FAST_5,
  FAST_6,
  FAST_7,
  FAST_8,
  FAST_9,
  FAST_10,
  FAST_11,
  FAST_12,        // 15
  CHARGE_MANUAL,  // 16
  CHARGE_AUTO,    // 17
  CHARGE_READY    // 18
};

volatile int32_t counter_left = 0, counter_right = 0;
bool ShutDownRequest = false;
int32_t ShutdownFilter = 0;
int32_t tLiftupTimeout = 0;
static uint32_t tShutdownBegin;

// LED
enum enum_led_status Led_State = STARTING_UP;
int Led_Duration = 0;
int Led_R = 0;
int Led_G = 0;
int Led_B = 0;
int Led_Blink_Cycle = 0;

// Battery Status
#define BATT_LOW 10
#define BATT_HIGH 100
#define BATT_FULL 100

// Define arg for READ_PARAMETER
#define ARG_BATT_CAPA 1
#define ARG_BATT_VOLT 2
#define ARG_BATT_CURR 3
// Voltage spike when change state Charge <-> Dischanrge (Unit : Volt)
#define BATT_VOLT_SPIKE 0.15
// Time for stable when change between charge and discharge (Unit : ms)
#define BATT_STABLE_TIME 10000

// clang-format off
// State of Discharge Index table from 0 -> 100%
// Capacity                     00     10     20     30     40     50     60     70     80     90    100
float BATT_SOC[11]        = {22.72, 23.02, 23.32, 23.62, 23.92, 24.2, 24.48, 24.74, 25.00, 25.24, 25.46};
//                          {22.00, 22.80, 23.32, 23.90, 24.20, 24.80, 25.35, 25.65, 26.10, 26.35, 26.50};
// State of Charge Index table from 10 - 100%
float BATT_SOC_CHARGE[11] = {23.80, 24.65, 25.10, 25.50, 25.83, 26.15, 26.48, 26.85, 27.22, 27.58, 27.78};
//                          {24.00, 24.52, 24.95, 25.35, 25.86, 26.15, 26.40, 26.62, 26.83, 27.35, 28.28};
// clang-format on

// Battery state variable
int BatteryCapacity;  // 0--> 100 %
int BienDem = 0;
int CountDataCurrentOneMinute = 0;
int CountDataVolOneMinute = 0;
int CountDataPercentOneMinute = 0;

float BatteryCapacityF = -1;
float BatteryCapacityKalman;
float BattVoltage = 0;
float OldBattVoltage = 0;
float BattDeltaVol = 0;
float BattCurrent = 0;
float BattCapacity = 0;
float SumDataVolOneMinute = 0.0;
float SumDataAmpeOneMinute = 0.0;
float SumDataPercentOneMinute = 0.0;

bool BATT_CHARGE = 0;
bool BATT_DISCHARGE = 0;
bool BattChangeState = 0;  //  = True when switch between charge and discharge
bool BATTERY_FULL = 0;     // Battery full

// Board CPU separate (similar AGV300)
// Tính VOLT_DIV_CURR_PIN và VOLT_DIV_VOLT_PIN dựa trên điện áp thực tế tại các
// chân terminal mà không cần quan tâm tới điện trở phân áp.

// Curent measurement pin
// Sensor ACS7123: 66mV/A
// (1023 * điện áp thực tế chân AD1) / (5 * analogRead)
// = (1023 * 2.958) / (5 * 42)
// Hoặc
// 1023 * (Dòng thực tế (A) * 0.066 + 2.5)
// ---------------------------------------
// 5 * analogRead(POWER_CURRENT_CHARGE_PIN)
#define VOLT_DIV_CURR_PIN 0.977
// Voltage measurement pin
// = (820k + 68k) / 68k = (1023 * điện áp thực tế chân AD3) / (5 * analogRead)
// = (1023 * 26.84) / (5 * 420)
#define VOLT_DIV_VOLT_PIN 13.995
#define VOLT_DIV_VOLT_PIN_WHEN_CHARGER 13.995

#endif