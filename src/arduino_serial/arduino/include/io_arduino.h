#include "define_io.h"
#include <arduino_serial/BoolArrayStamped.h>
#include <arduino_serial/SetPinStamped.h>
#include <ros.h>
#include <std_stamped_msgs/BoolStamped.h>
#include <std_stamped_msgs/Int16MultiArrayStamped.h>
// arduino_serial::BoolArrayStamped digi_pin_array_msg;
// ros::Publisher digiPinArray("/arduino_driver/pin_status_array",
//                             &digi_pin_array_msg);
#define NUM_PIN 70
#define TIMER_HZ 1000
static uint32_t tUpdate_IO = 0;
int ledState = LOW;
// unsigned long led_previousMillis = 0;

int led_turnSignal[] = {LED_SIGNAL_FOR_TURN_LEFT, LED_SIGNAL_FOR_TURN_RIGHT,
                        LED_SIGNAL_BACK_TURN_LEFT, LED_SIGNAL_BACK_TURN_RIGHT};

arduino_serial::BoolArrayStamped digi_pin_array_msg;
// std_stamped_msgs::BoolStamped
ros::Publisher digiPinArray("/arduino_driver/pin_status_array",
                            &digi_pin_array_msg);

bool blinkWithoutDelay(int, int);
void otherIO_control_part();
void display_led();
void set_pin_digital_cb(const arduino_serial::SetPinStamped& set_pin_msg)
{
  pinMode(set_pin_msg.pin, OUTPUT);
  digitalWrite(set_pin_msg.pin, set_pin_msg.value);
}
void set_pin_charging_cb(
    const std_stamped_msgs::BoolStamped& chargingset_pin_msg)
{
  digitalWrite(EN_AT_CHARGING, chargingset_pin_msg.data);
}

void set_led_turn_signal_cb(
    const std_stamped_msgs::Int16MultiArrayStamped& signal_turn_led_msg)
{
  // izeof(call_back.data) / sizeof(int); = 2
  int Led_turnCount = sizeof(signal_turn_led_msg.data) / sizeof(int);
  for (int thisPin = 0; thisPin < 4; thisPin++)
  {
    // if(signal_turn_led_msg.data[thisPin] == 0 ){
    //   digitalWrite(led_turnSignal[thisPin], signal_turn_led_msg.data);
    // }
    // else if (signal_turn_led_msg.data[thisPin] == 1){
    //   digitalWrite(led_turnSignal[thisPin], signal_turn_led_msg.data);
    //   // blinkWithoutDelay(led_turnSignal[thisPin], TIMER_HZ/2 );
    // }
    // else {
    //   blinkWithoutDelay(led_turnSignal[thisPin], TIMER_HZ/2 );
    //   // digitalWrite(led_turnSignal[thisPin], HIGH);
    // }
    // }
    digitalWrite(led_turnSignal[thisPin], signal_turn_led_msg.data[thisPin]);
    // if (signal_turn_led_msg.data == 2 ){
    //   // pass
    //   //  blinkWithoutDelay(led_turnSignal[thisPin], TIMER_HZ/2 );
    // }
    // else {
    //   digitalWrite(led_turnSignal[thisPin], signal_turn_led_msg.data[thisPin]);
    // }
  }
}

ros::Subscriber<arduino_serial::SetPinStamped>
    setPinDigital("/arduino_driver/set_pin", &set_pin_digital_cb);

ros::Subscriber<std_stamped_msgs::BoolStamped>
    setPinCharging("/arduino_driver/write_auto_charge", &set_pin_charging_cb);

ros::Subscriber<std_stamped_msgs::Int16MultiArrayStamped>
    setLedTurnSignal("/arduino_driver/led_turn", &set_led_turn_signal_cb);

void pubArrayIO(uint32_t time, float hz)
{
  if ((time - tUpdate_IO) >= TIMER_HZ / hz)
  {
    display_led();

    // publish digital pin status array
    digi_pin_array_msg.stamp = nh.now();
    digi_pin_array_msg.data_length = NUM_PIN;
    bool arr[NUM_PIN];
    for (int num = 0; num < NUM_PIN; num++)
    {
      arr[num] = digitalRead(num);
    }
    digi_pin_array_msg.data = arr;
    digiPinArray.publish(&digi_pin_array_msg);
    tUpdate_IO = time;
  }
  otherIO_control_part();
}

bool blinkWithoutDelay(int pinout, int wait)
{
  // int ledState = LOW;
  unsigned long led_previousMillis = 0;
  unsigned long led_currentMillis = millis();
  if (led_currentMillis - led_previousMillis >= wait)
  {
    led_previousMillis = led_currentMillis;

    if (ledState == LOW)
    {
      ledState = HIGH;
    }
    else
    {
      ledState = LOW;
    }
    // Serial.print("ledState  ");
    // Serial.println(ledState);
    digitalWrite(pinout, ledState);
  }
}

void display_led()
{
  // Display LED
  bool _START_1_PIN = digitalRead(START_1_PIN) == SENSOR_ON;
  bool _STOP_1_PIN = digitalRead(STOP_1_PIN) == SENSOR_ON;
  bool _START_2_PIN = digitalRead(START_2_PIN) == SENSOR_ON;
  bool _STOP_2_PIN = digitalRead(STOP_2_PIN) == SENSOR_ON;

  if (_START_1_PIN)
  {
    digitalWrite(LED_START_FORWARD, LED_ON);
  }
  else
    digitalWrite(LED_START_FORWARD, LED_OFF);
  if (_STOP_1_PIN)
    digitalWrite(LED_STOP_FORWARD, LED_ON);
  else
    digitalWrite(LED_STOP_FORWARD, LED_OFF);

  if (_START_2_PIN)
    digitalWrite(LED_START_BACKWARD, LED_ON);
  else
    digitalWrite(LED_START_BACKWARD, LED_OFF);
  if (_STOP_2_PIN)
    digitalWrite(LED_STOP_BACKWARD, LED_ON);
  else
    digitalWrite(LED_STOP_BACKWARD, LED_OFF);
}

void otherIO_control_part() {}