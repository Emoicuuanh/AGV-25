// #include <ArduinoUniqueID.h>
#include "common.h"
#include <Adafruit_NeoPixel.h>
#include <agv_msgs/LedControl.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#ifdef __AVR__
#include <avr/power.h>  // Required for 16 MHz Adafruit Trinket
#endif

#define LED_PIN 12
#define LED_FRONT_NUM 16
#define LED_RIGHT_NUM 28
#define LED_BACK_NUM 16
#define LED_LEFT_NUM 28
#define NUMPIXELS LED_FRONT_NUM + LED_RIGHT_NUM + LED_BACK_NUM + LED_LEFT_NUM
#define LED_COUNT NUMPIXELS  // For FastLed
#include <fast_led.h>
#define BRIGHTNESS 50
#define TIMER_HZ 1000
#define PERCENT_LOW_BATTERY 20

Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

enum enum_led_status
{
  STARTING_UP,    // 0
  SHUTING_DOWN,   // 1
  RAINBOW,        // 2
  CUSTOM,         // 3
  FAST_1,         // 4 // Rainbow fast_led
  FAST_2,         // Đuổi nhiều màu dải màu ngắn
  FAST_3,         // Đuổi fade nhiều màu
  FAST_4,         // Đuổi nhiều màu dải màu dài
  FAST_5,         // Chớp trắng
  FAST_6,         // Đuổi màu trắng
  FAST_7,         // Đuổi trắng
  FAST_8,         // Fade xanh lam trắng
  FAST_9,         // Fade xanh lam trắng
  FAST_10,        // Rainbow fast_led
  FAST_11,        // Nhiều màu đuổi
  FAST_12,        // 15 // Nhiều màu đuổi fade
  CHARGE_MANUAL,  // 16
  CHARGE_AUTO,    // 17
  CHARGE_READY,   // 18
  BLINK_ALL,      // 19
  LIGHT_ALL,      // 20
  CHASE,          // 21 // Led đuổi
};

enum enum_battery_level
{
  LOW_BATTERY = 20,
  FULL_BATTERY = 98,
};

bool NewLedEffectRequest = true;
bool NewLedEffectDone = false;
enum enum_led_status Led_State = STARTING_UP;
int Led_Duration = 100;
int Led_R = 127;
int Led_G = 127;
int Led_B = 127;
int Led_Blink_Cycle = 0;
// int LED_INDEX = 0;
int old_percent_battery = 5;
int led_end = NUMPIXELS;
int led_start = 0;
bool isCharging = false;
bool rosSerialConnected = false;

static uint32_t tUpdate_led = 0;
unsigned long tUpdate_theater = 0;

void loginfoNumber(char* start_msg, int num, char* stop_msg)
{
  if (!rosSerialConnected)
    return;
  char log_msg[20];
  char result[8];
  dtostrf(num, 0, 0, result);
  sprintf(log_msg, "%s%s%s", start_msg, result, stop_msg);
  nh.loginfo(log_msg);
}
void settingLed();
void led_control_cb(const agv_msgs::LedControl& led_control_msg);
int ledSwitch(uint32_t, bool*, int, float);
bool delay_time_led(uint32_t);
// bool led_control_battery(int);

// LED CONTROL
uint32_t Wheel(byte);
bool colorWipe(uint32_t);
bool led_reset();
bool chase_led(uint32_t, int);
bool rainbow(uint32_t, uint8_t);
void rainbowCycle(uint32_t, uint8_t);
int led_battery_control(enum_led_status, int);
void ledCharge(int, float, int, bool);
int led_state_switching(enum_led_status, bool, int, int, bool);
bool theaterChase(uint32_t, uint32_t, int);
bool blinkLed(uint32_t, int, int, int, int);
bool myColorIndex(uint32_t, int, int, int, int, int, int);
bool myColorWipe(uint32_t color, int wait, int num_of_pin = 0,
                 int LedPins[] = {});
bool myRainbow(int wait, int num_of_pin = 0, int LedPins[] = {});

ros::Subscriber<agv_msgs::LedControl> ledCtrl("/led_control", &led_control_cb);

void settingLed()
{
  FastLedSetup();
  pixels.begin();  // INITIALIZE NeoPixel pixels object (REQUIRED)
  pixels.show();   // Turn OFF all pixels ASAP
  pixels.setBrightness(BRIGHTNESS);  // Set BRIGHTNESS to about 1/5 (max = 255)
}

bool led_reset()
{
  pixels.clear();
  pixels.show();
  return true;
}

void led_control_cb(const agv_msgs::LedControl& led_control_msg)
{
  NewLedEffectDone = true;
  int type, duration, blink_cycle, r, g, b;
  type = led_control_msg.type;
  r = led_control_msg.r;
  g = led_control_msg.g;
  b = led_control_msg.b;
  duration = led_control_msg.duration;
  blink_cycle = led_control_msg.blink_interval;
  loginfoNumber("Led type: ", type, "");
  loginfoNumber("Led r: ", r, "");
  loginfoNumber("Led g: ", g, "");
  loginfoNumber("Led b: ", b, "");
  loginfoNumber("Led duration: ", duration, "");
  loginfoNumber("Led blink_cycle: ", blink_cycle, "");
  if (Led_R != r || Led_G != g || Led_B != b || type != Led_State ||
      duration != Led_Duration || blink_cycle != Led_Blink_Cycle)
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

int ledSwitch(uint32_t time, bool roscb, int chargingType,
              float percent_volBattery = old_percent_battery)
{
  static int prevLedState = -1;
  static int prevChargingState = -1;
  int length = 0;
  int duraction_charging = 1000;
  int LedPin[] = {LED_PIN};

  if (chargingType != 3)
  {
    isCharging = true;
  }

  if (prevChargingState != chargingType)
  {
    NewLedEffectDone = false;
    prevChargingState = chargingType;
  }

  int ledState = led_state_switching(Led_State, isCharging, chargingType,
                                     percent_volBattery, roscb);
  if (prevLedState != ledState)
  {
    Serial.print("Led state: ");
    Serial.println(ledState);
    loginfoNumber("Led: ", ledState, "");
    prevLedState = ledState;
  }

  switch (ledState)
  {
  case STARTING_UP:
    if (!NewLedEffectDone)
    {
      if ((colorWipe(pixels.Color(127, 127, 127))))
      {
        NewLedEffectDone = true;
      }
    }
    break;
  case SHUTING_DOWN:
    if (!NewLedEffectDone)
    {
      if ((colorWipe(pixels.Color(255, 40, 0))))
      {
        NewLedEffectDone = true;
      }
    }
    break;
  case RAINBOW:
    if (!NewLedEffectDone)
    {
      length = 1;
      LedPin[0] = LED_PIN;
      if (myRainbow(10, length, LedPin))
      {
        NewLedEffectDone = true;
      }
    }
    break;
  case CUSTOM:
    if (Led_Blink_Cycle == 0)
    {
      if (!NewLedEffectDone)
      {
        length = 1;
        LedPin[0] = LED_PIN;

        if (myColorWipe(pixels.Color(Led_R, Led_G, Led_B), Led_Duration, length,
                        LedPin))
        {
          NewLedEffectDone = true;
        }
      }
    }
    if (Led_Blink_Cycle > 0)
    {
      blinkLed(time, Led_R, Led_G, Led_B, Led_Duration);
    }
    break;
  case BLINK_ALL:
    if (!NewLedEffectDone || !NewLedEffectRequest)
    {
      if (blinkLed(time, Led_R, Led_G, Led_B, Led_Duration))
      {
        NewLedEffectDone = true;
        NewLedEffectRequest = false;
      }
    }
    break;
  case LIGHT_ALL:
    if (!NewLedEffectDone)
    {
      if ((colorWipe(pixels.Color(Led_R, Led_G, Led_B))))
      {
        NewLedEffectDone = true;
      }
    }
    break;
  case CHASE:
    if (!NewLedEffectDone || !NewLedEffectRequest)
    {
      if (chase_led(pixels.Color(Led_R, Led_G, Led_B), Led_Duration))
      {
        NewLedEffectDone = true;
        NewLedEffectRequest = false;
      }
    }
    break;
  case CHARGE_MANUAL:
    // BatteryCapacity = 100; // fake full battery
    ledCharge(Led_Duration, percent_volBattery, CHARGE_MANUAL, true);
    break;
  case CHARGE_AUTO:
    // BatteryCapacity = 100; // fake full battery
    ledCharge(Led_Duration, percent_volBattery, CHARGE_AUTO, true);
    break;
  case CHARGE_READY:
    ledCharge(Led_Duration, percent_volBattery, CHARGE_READY, true);
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
  default:
    break;
  }

  if (ledState >= FAST_1 and ledState <= FAST_12)
  {
    FastLedLoop();
  }

  return ledState;
}

int led_state_switching(enum_led_status _led_state, bool charging_,
                        int type_charging_, int percent_volbattery_,
                        bool roscb_)
{
  // Serial.print("charging_ chargne  : ");

  // Serial.println(charging_);

  if (!roscb_ && !charging_)
  {
    // Serial.println("this funtion normal");
    return STARTING_UP;
  }
  else if (!roscb_ && charging_)
  {
    // Serial.println("this funtion ");
    //  Serial.println("this chargne ");
    return led_battery_control(_led_state, type_charging_);
  }
  else
  {
    // Serial.println("ros");
    return _led_state;
  }
}

int led_battery_control(enum_led_status _led_state, int type_charging)
{
  if (type_charging == 1)
  {
    return CHARGE_MANUAL;
    // Led_State = CHARGE_MANUAL;
  }
  else if (type_charging == 2)
  {
    return CHARGE_AUTO;
    // Led_State = CHARGE_AUTO;
  }
  else
  {
    // Led_State = CHARGE_MANUAL;
    // NewLedEffectDone = true;
    return _led_state;
  }
}

bool colorWipe(uint32_t c)
{
  for (uint16_t i = 0; i < pixels.numPixels(); i++)
  {
    pixels.setPixelColor(i, c);
  }
  pixels.show();
  return true;
}

bool chase_led(uint32_t c, int wait)
{
  uint16_t static chaese_i = 0;
  // chaese_i = 0;
  if (delay_time_led(wait))
  {
    pixels.setPixelColor(chaese_i, c);
    pixels.show();
    chaese_i = chaese_i + 1;
    if (chaese_i >= NUMPIXELS)
    {
      led_reset();
      chaese_i = 0;
    }
  }
  return true;
}

bool rainbow(uint32_t time, uint8_t wait = 2000)
{
  uint16_t i, j;
  bool rainbow_on = false;
  static uint32_t over_time = millis();

  if ((time - over_time) >= wait)
  {
    rainbow_on = true;
  }

  if (rainbow_on)
  {
    // Serial.println(" this funtion");
    for (j = 0; j < 256; j++)
    {
      for (i = 0; i < pixels.numPixels(); i++)
      {
        pixels.setPixelColor(i, Wheel((i + j) & 255));
        pixels.show();
      }
    }
    return true;
  }
}

bool myRainbow(int wait, int num_of_pin = 0, int LedPins[] = {})
{
  static uint32_t t = millis();
  static int i = 0;
  static long firstPixelHue = 0;

  if (NewLedEffectRequest)
  {
    NewLedEffectRequest = false;
    i = 0;
  }

  if (firstPixelHue < 5 * 65536)
  {
    for (int i = 0; i < pixels.numPixels(); i++)
    {
      int pixelHue = firstPixelHue + (i * 65536L / pixels.numPixels());
      pixels.setPixelColor(i, pixels.gamma32(pixels.ColorHSV(pixelHue)));
    }
    if (num_of_pin == 0)
    {
      pixels.setPin(LED_PIN);
      pixels.show();
    }
    else
    {
      for (int count = 0; count < num_of_pin; count++)
      {
        pixels.setPin(LedPins[count]);
        pixels.show();
      }
      pixels.setPin(LED_PIN);  // return control LED_PIN for other function
    }
    if (millis() - t >= wait)
    {
      firstPixelHue += 256;
      t = millis();
    }
  }
  else
  {
    firstPixelHue = 0;
    return true;
  }
  return false;
}

bool theaterChase(uint32_t time, uint32_t c, int wait)
{
  bool theater_chase = false;
  for (int j = 0; j < 10; j++)
  {
    for (int q = 0; q < 3; q++)
    {
      for (uint16_t i = 0; i < pixels.numPixels(); i = i + 3)
      {
        pixels.setPixelColor(i + q, c);
      }
      pixels.show();
      // delay(100);
      for (uint16_t i = 0; i < pixels.numPixels(); i = i + 3)
      {
        pixels.setPixelColor(i + q, 0);
      }
    }
  }
  return true;
}

uint32_t Wheel(byte WheelPos)
{
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85)
  {
    return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170)
  {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

bool blinkLed(uint32_t time, int led_r, int led_g, int led_b,
              int blink_interval)
{
  static uint32_t t_start_blink = 0;
  static int blink_state = 0;

  if (blink_state == 0)
  {
    if (colorWipe(pixels.Color(led_r, led_g, led_b)))
    {
      blink_state = 1;
      t_start_blink = time;
    }
  }
  else if (blink_state == 1)
  {
    if (time - t_start_blink >= blink_interval)
    {
      blink_state = 2;
      t_start_blink = time;
    }
  }
  if (blink_state == 2)
  {
    if (led_reset())
    {
      blink_state = 3;
      t_start_blink = time;
    }
  }
  else if (blink_state == 3)
  {
    if (time - t_start_blink >= blink_interval)
    {
      blink_state = 0;
      t_start_blink = time;
    }
  }
  return true;
}

void ledCharge(int duration, float percent_battery, int mode, bool led_charging)
{
  int __r, __g, __b;
  int Batt_Status;
  int _ledStart = LED_FRONT_NUM + LED_RIGHT_NUM - 1;               // 43;
  int _ledEnd = LED_FRONT_NUM + LED_RIGHT_NUM + LED_BACK_NUM + 1;  // 61;
  int number_led_charging = percent_battery * 0.29;

  // Calculate Battery Status
  // percent_battery = constrain(batt_Capacity, 0, 100);

  if (percent_battery <= LOW_BATTERY)
    Batt_Status = 1;  // Battery Low
  if ((percent_battery > LOW_BATTERY) && (percent_battery < FULL_BATTERY))
    Batt_Status = 2;  // Battery Mid
  if (percent_battery >= FULL_BATTERY)
    Batt_Status = 3;  // Battery Full

  if (led_charging)
  {
    switch (Batt_Status)
    {
    case 1:
      // Slect color
      switch (mode)
      {
      case CHARGE_MANUAL:
        __r = 255, __g = 0, __b = 0;
        break;
      case CHARGE_AUTO:
        __r = 255, __g = 0, __b = 0;
        break;
      case CHARGE_READY:
        __r = 255, __g = 0, __b = 0;
        break;
      default:
        break;
      }
      if (myColorIndex(pixels.Color(__r, __g, __b), duration, 0,
                       number_led_charging, _ledStart, 2, _ledEnd))
      {
        led_charging = false;
        led_reset();
      }
      break;
    case 2:
      // Slect color
      switch (mode)
      {
      case CHARGE_MANUAL:
        __r = 0, __g = 0, __b = 255;  // Blue
        break;
      case CHARGE_AUTO:
        __r = 255, __g = 102, __b = 0;  // Orange
        break;
      case CHARGE_READY:
        __r = 0, __g = 255, __b = 0;  // Green
        break;
      default:
        break;
      }

      if (myColorIndex(pixels.Color(__r, __g, __b), duration, 0,
                       number_led_charging, _ledStart, 2, _ledEnd))
      {
        led_charging = false;
        led_reset();
      }
      break;
    case 3:  // Full battery
      // Slect color
      switch (mode)
      {
      case CHARGE_MANUAL:
        __r = 0, __g = 0, __b = 255;
        break;
      case CHARGE_AUTO:
        __r = 0, __g = 255, __b = 0;
        break;
      case CHARGE_READY:
        __r = 0, __g = 255, __b = 0;
        break;
      default:
        break;
      }
      blinkLed(millis(), __r, __g, __b, Led_Duration);

      break;
    default:
      break;
    }
  }
  else
  {
    if (led_reset())
    {
      // NewLedEffectDone = false;
      // Serial.println("keep false");
    }
  }
  // Serial.println("on this case");
}

bool myColorIndex(uint32_t color, int wait, int startLED,
                  int number_leds_running, int offsetLED, int invert,
                  int invLength)
{
  static uint32_t t = millis();
  static int i = startLED;

  if (NewLedEffectRequest)
  {
    NewLedEffectRequest = false;
    i = startLED;
  }
  if ((i >= startLED) && (i <= (number_leds_running + 1)))
  {
    if (millis() > t + wait)
    {
      if (i > number_leds_running)
      {
        i = startLED;
        return true;
      }
      switch (invert)  // select led follow type invert
      {
      case 1:
        pixels.setPixelColor((2 * startLED) + invLength - i - 1, color);
        pixels.setPixelColor(i + offsetLED, color);
        break;

      case 2:
        pixels.setPixelColor((2 * startLED) + invLength + i - 1, color);
        pixels.setPixelColor(offsetLED - i, color);
        break;

      default:
        pixels.setPixelColor(i, color);
        break;
      }

      pixels.show();
      i++;
      t = millis();
    }
    // LED_INDEX = i;
  }
  return false;
}

bool delay_time_led(uint32_t time_delay)
{
  static uint32_t tUpdate_led = 0;
  if ((millis() - tUpdate_led) >= time_delay)
  {

    tUpdate_led = millis();
    return true;
  }
  else
  {
    return false;
  }
}

bool myColorWipe(uint32_t color, int wait, int num_of_pin = 0,
                 int LedPins[] = {})
{
  static uint32_t t = millis();
  static int i = 0;

  if (NewLedEffectRequest)
  {
    NewLedEffectRequest = false;
    i = 0;
  }
  if (wait == 0)
  {
    for (int j = 0; j < pixels.numPixels(); j++)
    {
      pixels.setPixelColor(j, color);  //  Set pixel's color (in RAM)
    }
    if (num_of_pin == 0)
    {
      pixels.setPin(LED_PIN);
      pixels.show();
    }
    else
    {
      for (int count = 0; count < num_of_pin; count++)
      {
        pixels.setPin(LedPins[count]);
        pixels.show();
      }
      pixels.setPin(LED_PIN);  // return control LED_PIN for other function
    }

    return true;
  }
  else if (i <= pixels.numPixels())
  {
    if (millis() - t >= wait)
    {
      pixels.setPixelColor(i, color);  //  Set pixel's color (in RAM)
      if (num_of_pin == 0)
      {
        pixels.setPin(LED_PIN);
        pixels.show();
      }
      else
      {
        for (int count = 0; count < num_of_pin; count++)
        {
          pixels.setPin(LedPins[count]);
          pixels.show();
        }
        pixels.setPin(LED_PIN);  // return control LED_PIN for other function
      }
      if (i++ == pixels.numPixels())
      {
        i = 0;
        return true;
      }
      t = millis();
    }
  }
  return false;
}
