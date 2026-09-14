// #include <ArduinoUniqueID.h>
#include <math.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_stamped_msgs/BoolStamped.h>
#include <std_stamped_msgs/EmptyStamped.h>
#include <std_stamped_msgs/Float32Stamped.h>
#include <std_stamped_msgs/StringStamped.h>

ros::NodeHandle nh;

#include "battery_control.h"
#include "define_io.h"
#include "io_arduino.h"
#include "led.h"
#include "lift_control.h"

void initROS();
void settingParameter();

uint32_t tLastArduinoRequest;
uint32_t tLogDebug = millis();
uint32_t tUpdateVolCurrent = millis();
uint32_t t = millis();

bool rosConnected = false;
int percentBatety = 0;

void arduinoLedBlinkCb(const std_stamped_msgs::EmptyStamped& toggle_msg)
{
  digitalWrite(BLINK_PIN, HIGH - digitalRead(BLINK_PIN));
  tLastArduinoRequest = millis();
}

ros::Subscriber<std_stamped_msgs::EmptyStamped>
    arduinoBlink("/arduino_driver/led_blink", &arduinoLedBlinkCb);

void setup()
{
  nh.getHardware()->setBaud(57600);
  initROS();
  // Serial.begin(115200);
  settingParameter();
}

void initROS()
{
  nh.initNode();
  nh.subscribe(ledCtrl);
  nh.subscribe(lifting_control_sub);
  nh.subscribe(arduinoBlink);
  nh.subscribe(setPinDigital);
  nh.subscribe(setPinCharging);
  nh.subscribe(setLedTurnSignal);
  nh.advertise(battCapa);
  nh.advertise(battVolt);
  nh.advertise(battCurrCharge);
  nh.advertise(digiPinArray);
}

void settingParameter()
{
  settingIO();
  settingLed();
  settingBattery(CHARGING_MAN, CHARGING_AUTO);
  // lifting_setting(LIFT_DIR, PWM_M1, LIFT_MIN_SENSOR, LIFT_MAX_SENSOR);
}

void loop()
{
  t = millis();

  if (t - tLastArduinoRequest >= 500)
  {
    rosConnected = false;
  }
  else
  {
    rosConnected = true;
  }

  controlChaging(rosConnected);
  calcChargingStatus();
  liftControl(t, 50, 50, 500);
  pubArrayIO(t, 10);
  if (t > tLogDebug and t - tUpdateVolCurrent >= 1000)
  {
    tUpdateVolCurrent = t;
    percentBatety = batteryUpdate(t, rosConnected, return_io.charging);
  }
  int led = ledSwitch(t, rosConnected, return_io.charging, percentBatety);

  // if (t > tLogDebug and t - tLogDebug >= 1000)
  // {
  //   tLogDebug = t;
  //   Serial.print("Current digital value: ");
  //   Serial.println(analogRead(POWER_CURRENT_CHARGE_PIN));
  //   float current = readCurrentCharging();
  //   Serial.print("Current: ");
  //   Serial.print(current);
  //   Serial.println(" (A)");
  // }

  nh.spinOnce();
}
