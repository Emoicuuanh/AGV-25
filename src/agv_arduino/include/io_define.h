#ifndef AGV_IO_H_
#define AGV_IO_H_

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
#define Y19 15
#define AD0 A3  // 57
#define AD1 A2  // 56
#define AD2 A1  // 55
#define AD3 A0  // 54

// Define IO
#define START_1_PIN X4
#define START_2_PIN X5
#define STOP_1_PIN X6
#define STOP_2_PIN X7
#define RELEASE_PIN X8
#define EMG_1_PIN X0

#define MAN_CHARGING_PIN X21
#define AUTO_CHARGING_PIN X22

#define AUTO_MAN_SW X9

#define DETECT_CART_SS X1
#define AUTO_CHARGING_RELAY_PIN Y8
#define MOTOR_ENABLE_PIN Y9

#define LED_START_FORWARD Y2
#define LED_START_BACKWARD Y3
#define LED_STOP_FORWARD Y4
#define LED_STOP_BACKWARD Y5

#define LED_SIGNAL_FOR_TURN_LEFT Y10
#define LED_SIGNAL_FOR_TURN_RIGHT Y11
#define LED_SIGNAL_BACK_TURN_LEFT Y12
#define LED_SIGNAL_BACK_TURN_RIGHT Y13
#define BREAK_MOTOR_PIN Y19
#define POWER_CURRENT_CHARGE_PIN AD1
#define POWER_VOL_PIN AD3

#define LIFT_DIR 46
#define PWM_M1 45

#define LIFT_MIN_SENSOR X3
#define LIFT_MAX_SENSOR X2

/*LED*/
#define LED_PIN 37
#define LED_FRONT_NUM 16
#define LED_BACK_NUM 16
#define LED_RIGHT_NUM 28
#define LED_LEFT_NUM 28
#define LED_COUNT LED_FRONT_NUM + LED_BACK_NUM + LED_RIGHT_NUM + LED_LEFT_NUM
#define LED_BATT_MIN 10

#endif