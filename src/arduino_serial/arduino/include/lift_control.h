#include "define_io.h"
#include <ros.h>
#include <std_msgs/Int8.h>

#define CONTROL_LIFTUP_TIMEOUT 200

// #define LIFT_MIN_SENSOR A12
// #define LIFT_MAX_SENSOR A13
// #define LIFT_DIR  11
// #define PWM_M1    10

void liftControl(uint32_t, int, int, int);
void liftControlCb(const std_msgs::Int8& control_lifting_msg);

int PWM_TCNT = 0;
int PWM_DIV = 0;
int liftDir = 0;
int32_t tLiftupTimeout = 0;
// Lift motor variable
int PWM_M1_Valve = 0;
int PWM_TargetSpeed = 255;

// taget value PWM
bool LIFT_ENABLE = LIFT_OFF;  // ON/OFF Lift Motor
bool SOFT_START_RUN = 0;      // Soft start running
bool SOFT_START_CMP = 0;      // Soft start complete

//____ Speed Parameter ____
int MinSpeed = 100;
int MaxSpeed = 255;

//____ Time Parameter  ______
unsigned char ResolutionTime = 20;  // (1-255) ms
/* Timer Interrupt */

// ROS Parameter
int Debug_Level = 0;
// ros::Publisher chatter("chatter", &lifting_status_pub);
ros::Subscriber<std_msgs::Int8> lifting_control_sub("lifting_control",
                                                    liftControlCb);
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
      TIMSK1 &= (0 << TOIE1);          // Disable Timer 1 interrupt
    }
  }
  analogWrite(PWM_M1, PWM_M1_Valve);
}

/* Lift Motor RUN */
void runLiftMotor(bool _liftDir, int _StartSpeed)
{
  digitalWrite(LIFT_DIR, _liftDir);  // DIR
  // for (int i = 0; i <= 255; i++)
  // {
  //   analogWrite(10, 255);
  // }
  digitalWrite(10, 1);  //
  // analogWrite(10, 255);

  // ENABLE LIFT
  LIFT_ENABLE = LIFT_ON;

  // On PWM
  // if (!SOFT_START_RUN && !SOFT_START_CMP)
  // {
  //   PWM_M1_Valve = _StartSpeed;
  //   // analogWrite(10,255);
  //   analogWrite(PWM_M1, PWM_M1_Valve);  // PWM
  //   TCNT1 = PWM_TCNT;                   // Duty update PWM value time
  //   // enable timer Overflow interrupt
  //   TIMSK1 |= (1 << TOIE1);
  //   SOFT_START_RUN = 1;
  //   SOFT_START_CMP = 0;
  // }
}

/* Lift Motor OFF */
void stopLiftMotor()
{
  LIFT_ENABLE = LIFT_OFF;
  PWM_M1_Valve = 0;        // off PWM
  analogWrite(PWM_M1, PWM_M1_Valve);
  TIMSK1 &= (0 << TOIE1);  // Disable Timer 1 interrupt
  SOFT_START_CMP = 0;
  SOFT_START_RUN = 0;
  // Serial.println("OFF");
}

/* Lift control*/
void liftControl(uint32_t t, int StartSpeed = 255, int TargetSpeed = 255,
                 int SoftStartTime = 1000)
{
  // Limit parameter input
  if (StartSpeed < MinSpeed)
    StartSpeed = MinSpeed;
  if (TargetSpeed > MaxSpeed)
    TargetSpeed = MaxSpeed;
  // Serial.println("LIFT_MAX_SENSOR");
  // Serial.println(digitalRead(LIFT_MAX_SENSOR));
  // Serial.println("LIFT_MIN_SENSOR");
  // Serial.println(digitalRead(LIFT_MIN_SENSOR));

  // result for timer parameter
  PWM_TargetSpeed = TargetSpeed;
  PWM_TCNT =
      65536 - ResolutionTime * 250;  // Reload value for timer 250*4us = 1ms
  PWM_DIV = (TargetSpeed - StartSpeed) / (SoftStartTime / ResolutionTime);

  bool LIFT_MAX_PUSH = digitalRead(START_1_PIN) == SENSOR_ON ||
                       digitalRead(START_2_PIN) == SENSOR_ON;
  bool LIFT_MIN_PUSH = digitalRead(STOP_1_PIN) == SENSOR_ON ||
                       digitalRead(STOP_2_PIN) == SENSOR_ON;
  bool LIFT_MID_PUSH = digitalRead(START_1_PIN) == SENSOR_ON &&
                           digitalRead(STOP_1_PIN) == SENSOR_ON ||
                       digitalRead(START_2_PIN) == SENSOR_ON &&
                           digitalRead(STOP_2_PIN) == SENSOR_ON;
  bool EMG_EN = digitalRead(EMG_PIN) == SENSOR_ON;
  bool AU_MAN_SEL =
      digitalRead(AUTO_MAN_SW) == SENSOR_ON;  // Auto : 1 ; Manual : 0
  bool MAX_LIMIT = digitalRead(LIFT_MAX_SENSOR);
  bool MIN_LIMIT = digitalRead(LIFT_MIN_SENSOR);

  // str_msg.data = hello;
  // status_lifing.publish(&lifting_status_pub);
  if (LIFT_MAX_PUSH && !LIFT_MIN_PUSH && EMG_EN && !AU_MAN_SEL and !MAX_LIMIT)
  {
    // Serial.println(MAX_LIMIT);
    // Serial.println("UP");
    runLiftMotor(LIFT_UP, StartSpeed);
    // Serial.println("MAX");
  }
  else if (LIFT_MIN_PUSH && !LIFT_MAX_PUSH && EMG_EN && !AU_MAN_SEL and
           !MIN_LIMIT)
  {
    // Serial.println(MIN_LIMIT);
    // Serial.println("DOWN");
    runLiftMotor(LIFT_DOWN, StartSpeed);
    // Serial.println("MIN");
  }
  else
  {
    if ((t > (tLiftupTimeout + CONTROL_LIFTUP_TIMEOUT)) ||
        !EMG_EN)  // Over time or EMG
    {
      stopLiftMotor();
      tLiftupTimeout = t;
    }
  }
}

// ROS
void liftControlCb(const std_msgs::Int8& control_lifting_msg)
{
  bool MAX_LIMIT = digitalRead(LIFT_MAX_SENSOR);
  bool MIN_LIMIT = digitalRead(LIFT_MIN_SENSOR);

  int lifting_cmd = control_lifting_msg.data;

  tLiftupTimeout = millis();
  // Serial.println("LC");
  if (digitalRead(START_1_PIN) == SENSOR_ON or
      digitalRead(START_2_PIN) == SENSOR_ON)
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

  if (lifting_cmd == LIFT_MAX_POSITION and !MAX_LIMIT)
  {
    // runLiftMotor(LIFT_UP, StartSpeed);
    runLiftMotor(LIFT_UP, 50);
    // Serial.println(digitalRead("lifing max"));
    // Serial.println(digitalRead(LIFT_MAX_SENSOR));
  }
  else if (lifting_cmd == LIFT_MIN_POSITION and !MIN_LIMIT)
  {
    // runLiftMotor(LIFT_DOWN, StartSpeed);
    runLiftMotor(LIFT_DOWN, 100);
  }
  else
  {
    stopLiftMotor();
    SOFT_START_CMP = 0;
    SOFT_START_RUN = 0;
  }
}
