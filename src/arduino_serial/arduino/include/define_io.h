#ifndef AGV_IO_H_
#define AGV_IO_H_

// #Arduino MEGA2560 Analog pin mapping:
// #A0 : 54,
// #A1 : 55,
// #A2 : 56,
// #A3 : 57,
// #A4 : 58,
// #A5 : 59,
// #A6 : 60,
// #A7 : 61,
// #A8 : 62,
// #A9 : 63,
// #A10 : 64,
// #A11 : 65,
// #A12 : 66,
// #A13 : 67,
// #A14 : 68,
// #A15 : 69,

// BJT socket board address mapping
#define X0 A15  // 69
#define X1 A14  // 68
#define X2 A13  // 67
#define X3 A12  // 66
#define X4 A11  // 65
#define X5 A10  // 64
#define X6 A9   // 63
#define X7 A8   // 62

#define X8 A7   // 61
#define X9 A6   // 60
#define X10 A5  // 59
#define X11 A4  // 58
#define X12 43
#define X13 4
#define X14 5
#define X15 2

#define X16 3
#define X17 6
#define X18 7
#define X19 8
#define X20 9
#define X21 49
#define X22 48
#define X23 47

#define Y0 24
#define Y1 25
#define Y2 26
#define Y3 27
#define Y4 28
#define Y5 29
#define Y6 39

#define Y7 30
#define Y8 31
#define Y9 32
#define Y10 33
#define Y11 34
#define Y12 35
#define Y13 36

#define AD0 A3  // 57
#define AD1 A2  // 56
#define AD2 A1  // 55
#define AD3 A0  // 54

// Define Value
#define SENSOR_ON 0
#define SENSOR_OFF 1
#define LIFT_ON 0
#define LIFT_OFF 1
#define LIFT_MAX_POSITION 0
#define LIFT_MIN_POSITION 1
#define LIFT_UP 0
#define LIFT_DOWN 1
#define LED_ON 1
#define LED_OFF 0

#define BUMPER_1_PIN X23
#define EMG_PIN X0

#define START_1_PIN X4
#define START_2_PIN X5
#define STOP_1_PIN X6
#define STOP_2_PIN X7

#define LIFT_MIN_SENSOR X3
#define LIFT_MAX_SENSOR X2

#define LED_START_FORWARD Y2
#define LED_START_BACKWARD Y3
#define LED_STOP_FORWARD Y4
#define LED_STOP_BACKWARD Y5

#define LIFT_DIR 11
#define PWM_M1 10

#define EN_AT_CHARGING Y8
#define EN_MOTOR Y9

#define AUTO_MAN_SW X9

#define LED_SIGNAL_FOR_TURN_LEFT Y10
#define LED_SIGNAL_FOR_TURN_RIGHT Y11
#define LED_SIGNAL_BACK_TURN_LEFT Y12
#define LED_SIGNAL_BACK_TURN_RIGHT Y13

#define POWER_VOL_PIN AD3
#define POWER_CURRENT_CHARGE_PIN AD1

#define CHARGING_MAN X21
#define CHARGING_AUTO X22
#define BLINK_PIN 13

typedef struct
{
  const int PIN;
  uint8_t TYPE_IO;
  bool DEFAULT_VALUE;
} PIN_Dictionary;

typedef struct IO_PIN_RETURN
{
  int charging;
  int beginpositie;
  int lengte;
};

IO_PIN_RETURN return_io;

void settingIO();
void calcChargingStatus();
// INPUT 0
// INPUT_PULLUP 2
// OUTPUT 1

const PIN_Dictionary PinIOArr[]{{BUMPER_1_PIN, INPUT_PULLUP, 0},
                                {EMG_PIN, INPUT_PULLUP, 0},
                                {AUTO_MAN_SW, INPUT_PULLUP, 0},
                                {START_1_PIN, INPUT_PULLUP, 0},
                                {START_2_PIN, INPUT_PULLUP, 0},
                                {STOP_1_PIN, INPUT_PULLUP, 0},
                                {STOP_2_PIN, INPUT_PULLUP, 0},
                                {CHARGING_MAN, INPUT_PULLUP, 0},
                                {CHARGING_AUTO, INPUT_PULLUP, 0},
                                {LIFT_MIN_SENSOR, INPUT_PULLUP, 0},
                                {LIFT_MAX_SENSOR, INPUT_PULLUP, 0},
                                {LED_START_FORWARD, OUTPUT, 0},
                                {LED_START_BACKWARD, OUTPUT, 0},
                                {LED_STOP_FORWARD, OUTPUT, 0},
                                {LED_STOP_BACKWARD, OUTPUT, 0},
                                {BLINK_PIN, OUTPUT, 0},
                                {LIFT_DIR, OUTPUT, 0},
                                {PWM_M1, OUTPUT, 0},
                                {LED_SIGNAL_FOR_TURN_LEFT, OUTPUT, 0},
                                {LED_SIGNAL_FOR_TURN_RIGHT, OUTPUT, 0},
                                {LED_SIGNAL_BACK_TURN_LEFT, OUTPUT, 0},
                                {LED_SIGNAL_BACK_TURN_RIGHT, OUTPUT, 0},
                                {EN_MOTOR, OUTPUT, 1},
                                {EN_AT_CHARGING, OUTPUT, 0}};

int length_dictionary = sizeof(PinIOArr) / sizeof(PIN_Dictionary);

void settingIO()
{
  for (uint8_t i = 0; i < length_dictionary; ++i)
  {
    pinMode(PinIOArr[i].PIN, PinIOArr[i].TYPE_IO);
    if (PinIOArr[i].DEFAULT_VALUE == 1)
    {
      digitalWrite(PinIOArr[i].PIN, PinIOArr[i].DEFAULT_VALUE);
    }
  }
}

void calcChargingStatus()
{
  static int prevChargeState = -1;
  if (digitalRead(CHARGING_MAN) == SENSOR_ON)
  {
    return_io.charging = 2;  // Manual
  }
  else if (digitalRead(CHARGING_AUTO) == SENSOR_ON &&
           digitalRead(CHARGING_MAN) == SENSOR_OFF)
  {
    return_io.charging = 1;  // Auto
  }
  else
  {
    return_io.charging = 3;  // Non charge
  }
  if (prevChargeState != return_io.charging)
  {
    prevChargeState = return_io.charging;
    Serial.print("Charging state: ");
    Serial.println(return_io.charging);
  }
}
#endif
