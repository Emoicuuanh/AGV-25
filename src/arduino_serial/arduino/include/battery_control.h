#include "define_io.h"
#include <SimpleKalmanFilter.h>
#include <ros.h>

SimpleKalmanFilter VoltageKalman(1, 1, 0.02);
SimpleKalmanFilter CapacityKalman(1, 1, 0.3);

std_stamped_msgs::StringStamped arduino_msg;
std_stamped_msgs::Float32Stamped batt_capa_msg, batt_capa_raw_msg,
    batt_volt_msg;
std_stamped_msgs::Float32Stamped batt_current_msg;

static uint32_t tUpdateBattery = 0;
#define TIMER_HZ 1000
#define BATT_LOW 10
#define BATT_HIGH 80
#define BATT_FULL 100
// Voltage spike when change state Charge <-> Discharge (Unit : Volt)
#define BATT_VOLT_SPIKE 0.25
#define BATT_LOW_VOL_THRESH 23.0  // Threshold of battery low to protect battery

// = (820k + 68k) / 68k = (1023 * điện áp thực tế chân AD3) / (5 * analogRead)
// = (1023 * 26.84) / (5 * 420)
#define VOLT_DIV_VOLT_PIN 13.07

// 1023 * (Dòng thực tế (A) * 0.066 + 2.5)
// -------------------/-------------------
// 5 * analogRead(POWER_CURRENT_CHARGE_PIN)
#define VOLT_DIV_CURR_PIN 0.99

// Define arg for READ_PARAMETER
// Voltage spike when change state Charge <-> Discharge (Unit : Volt)
#define BATT_VOLT_SPIKE 0.15
#define BATT_STABLE_TIME 300000

// clang-format off
// State of Discharge Index table from 0 -> 100%
// Capacity                     00     10     20     30     40     50     60     70     80     90    100
float BATT_SOC[11] =        {22.00, 23.00, 23.30, 23.55, 23.80, 24.04, 24.35, 24.63, 24.92, 25.18, 25.40};

// State of Charge Index table from 10 - 100%
float BATT_SOC_CHARGE[11] = {23.80, 24.65, 25.10, 25.50, 25.83, 26.15, 26.48, 26.85, 27.22, 27.58, 27.78};
// clang-format on

// Battery state variable
int BatteryCapacity;  // 0--> 100 %
float BatteryCapacityKalman;
bool BATT_CHARGE = 0;
bool BATT_DISCHARGE = 0;
bool BattChangeState = 0;  //  = True when switch between charge and discharge
bool BATTERY_FULL = 0;     // Battery full
bool AUTO_CHARGING_RELAY_STATUS = 0;  // Relay for auto charge jack ON/OFF
bool AUTO_CHARGING_COMPLETE = 1;      // State when auto charge complete
float OldBattVoltage = 0;
float BattVoltageKalman;
float BattDeltaVol = 0;
float BattCapacity = 0;
float LowChargingprocess = 3.6;
int percent_battery = 0;
float old_battery_befor = 0;
int time_update_battery = 5000;
int hz = 1;
uint8_t PIN_CHARGING_MAN = 100;
uint8_t PIN_CHARGING_AUTO = 100;
float ttd_ampe;
unsigned long t_auto_charging_jack_connect = millis();

// Function Delace for Battery
void settingBattery(uint8_t, uint8_t);
float vol_updateBattery(float, int);
bool delay_time_update_battery(uint32_t);
float readCurrentCharging();
float readBatteryVoltage();
float checkFullBatteryCapacity(float, bool);
float readRealBattCapa(float, bool);
float batteryUpdate(uint32_t, bool, int);

ros::Publisher battCapa("/arduino_driver/float_param/battery_percent",
                        &batt_capa_msg);
ros::Publisher battVolt("/arduino_driver/float_param/battery_voltage",
                        &batt_volt_msg);
ros::Publisher battCapaRaw("/arduino_driver/battery_capacity",
                           &batt_capa_raw_msg);
ros::Publisher battCurrCharge("/arduino_driver/float_param/battery_ampe",
                              &batt_current_msg);

void settingBattery(uint8_t charging_man, uint8_t charging_auto)
{
  PIN_CHARGING_MAN = charging_man;
  PIN_CHARGING_AUTO = charging_auto;
}

int check_charging(float BattCurrent)
{
  bool charging = false;
  if (BattCurrent > 0)
  {
    charging = true;
  }
}

float batteryUpdate(uint32_t time, bool roscb, int typeCharging)
{
  bool charging = false;
  float BattAmpe = readCurrentCharging();
  ttd_ampe = abs(BattAmpe);
  hz = ttd_ampe;
  float BattVoltage = readBatteryVoltage();
  float actualValueVol = BattVoltage * 1.0148;
  float vol_update = vol_updateBattery(actualValueVol, time_update_battery);
  int charging_or_not = check_charging(BattAmpe);
  percent_battery = readRealBattCapa(actualValueVol, charging);

  if (typeCharging != 3)
  {
    charging = true;
  }
  else
  {
    if (roscb)
    {
      if (BattAmpe <= LowChargingprocess && BattAmpe > 0)
      {
        // digitalWrite(EN_AT_CHARGING, LOW);
      }
    }
  }

  if ((time - tUpdateBattery) >= TIMER_HZ / hz)
  {
    batt_capa_msg.data = percent_battery;
    batt_capa_msg.stamp = nh.now();
    battCapa.publish(&batt_capa_msg);

    // Publish battery Voltage
    batt_volt_msg.data = actualValueVol;
    batt_volt_msg.stamp = nh.now();
    battVolt.publish(&batt_volt_msg);

    // Publish current charge
    batt_current_msg.data = BattAmpe;
    batt_current_msg.stamp = nh.now();
    battCurrCharge.publish(&batt_current_msg);

    tUpdateBattery = time;
  }
  return percent_battery;
}

// Read Current Chaging
float readCurrentCharging()
{
  float average = 0;
  // Measure 5 time continues
  for (int i = 0; i < 5; i++)
  {
    // Sensor ACS7123: 66mV/A
    average +=
        (analogRead(POWER_CURRENT_CHARGE_PIN) * VOLT_DIV_CURR_PIN * 5.0 / 1023 -
         2.5) /
        0.066;
  }
  return average / 5;
}

// Read Battery Voltage
float readBatteryVoltage()
{
  float average = 0;
  float batteryVoltage = 0;
  // Measure 5 time continues
  for (int i = 0; i < 5; i++)
  {
    average += (analogRead(POWER_VOL_PIN) / 1023.0 * 5.0) * VOLT_DIV_VOLT_PIN;
  }

  batteryVoltage = average / 5;
  BattDeltaVol = abs(batteryVoltage - BattVoltageKalman);
  if (BattDeltaVol > BATT_VOLT_SPIKE)
  {
    for (int temp = 0; temp < 50; temp++)
    {
      BattVoltageKalman = VoltageKalman.updateEstimate(batteryVoltage);
    }
  }
  BattVoltageKalman = VoltageKalman.updateEstimate(batteryVoltage);

  return batteryVoltage;
}

float vol_updateBattery(float percent_battery, int wait)
{
  // Serial.println(percent_battery);
  if (delay_time_update_battery(wait))
  {
    // Serial.println(wait);
    old_battery_befor = percent_battery;
  }
  return old_battery_befor;
}

bool delay_time_update_battery(uint32_t time_delay)
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

// Manager  battery capacity
float checkFullBatteryCapacity(float real_percent_battery, bool _charging)
{
  // Battery status
  int check_battety;
  static uint32_t t_battery_full = millis();

  if (real_percent_battery >= 100 && _charging)
  {
    if (millis() > (t_battery_full + 20000))
    {
      // real_percent_battery = 100;
      check_battety = 100;
      BATTERY_FULL = true;
    }
    else
    {
      check_battety = 99;
      // real_percent_battery = 99;
      BATTERY_FULL = false;
    }
  }
  else if (real_percent_battery >= 100 && !_charging)
  {
    // Serial.println(_charging);
    check_battety = 93;
  }
  else
  {
    check_battety = real_percent_battery;
    t_battery_full = millis();
    BATTERY_FULL = false;
  }
  // Serial.print("real_percent_battery");
  // Serial.println(real_percent_battery);
  // Serial.print("check_battety");
  // Serial.println(check_battety);
  return check_battety;
}

// Read Battery capacity
float readRealBattCapa(float voltage_battery, bool CHARGING)
{
  static uint32_t nowTime = 0;
  if (BattChangeState == 0)           // Charge or discharge
  {
    if (abs(voltage_battery) <= 0.5)  // remove error of current measurement
      voltage_battery = 0;

    if ((voltage_battery >= 0) && CHARGING)
    {
      // Charging state

      if (BATT_CHARGE == 0)
      {
        nowTime = millis();
        BattChangeState = 1;  // switch from discharge to charge
        BATT_CHARGE = 1;      // Charging
        BATT_DISCHARGE = 0;   // not discharging
        return BatteryCapacity;
      }

      // BATT_CHARGE_STAGE = 0;
      // Serial.println(" charge");
      if (voltage_battery > 0)
      {
        for (int i = 0; i < 10; i++)
        {
          if (BattVoltageKalman >= BATT_SOC_CHARGE[i])
          {
            BatteryCapacity =
                i * 10 + (BattVoltageKalman - BATT_SOC_CHARGE[i]) /
                             (BATT_SOC_CHARGE[i + 1] - BATT_SOC_CHARGE[i]) *
                             10.0;
          }
        }
      }
      if (BattVoltageKalman <= BATT_SOC_CHARGE[0])
        BatteryCapacity = 0;
      if (BattVoltageKalman >= BATT_SOC_CHARGE[10])
      {
        if (voltage_battery == 0)
        {
          BatteryCapacity = 100;
        }
        else
        {
          BatteryCapacity = 99;
        }
      }
    }
    else
    {
      if (BATT_CHARGE == 1)
      {
        nowTime = millis();
        BattChangeState = 1;  // switch from charge to discharge
        // Discharging state
        BATT_CHARGE = 0;     // Not Charging
        BATT_DISCHARGE = 1;  // Discharging
        return BatteryCapacity;
      }

      // Calculate SOC of Battery
      // Serial.println("no charge");
      for (int i = 0; i < 10; i++)
      {
        if (BattVoltageKalman >= BATT_SOC[i])
        {
          BatteryCapacity = i * 10 + (BattVoltageKalman - BATT_SOC[i]) /
                                         (BATT_SOC[i + 1] - BATT_SOC[i]) * 10.0;
        }
      }
      if (BattVoltageKalman <= BATT_SOC[0])
        BatteryCapacity = 0;
      if (BattVoltageKalman >= BATT_SOC[10])
        BatteryCapacity = 100;
    }
  }
  else  // Switch state between charge and discharge
  {
    if (((millis() - nowTime) > 5000))
    {
      BattChangeState = 0;
    }
  }
  BatteryCapacityKalman = CapacityKalman.updateEstimate(BatteryCapacity);
  BatteryCapacity =
      checkFullBatteryCapacity(int(BatteryCapacityKalman + 0.5), CHARGING);
  return BatteryCapacity;
}

void controlChaging(bool rosConnected)
{
  // Auto charging control
  if (digitalRead(CHARGING_AUTO) == SENSOR_ON)
  {
    if ((millis() > (3000 + t_auto_charging_jack_connect)) && !BATTERY_FULL)
    {
      if (rosConnected)
      // when arduino connect with PC
      {
        if (digitalRead(AUTO_MAN_SW) == SENSOR_OFF)  // Manual mode
        {
          digitalWrite(EN_AT_CHARGING, HIGH);
          AUTO_CHARGING_RELAY_STATUS = true;
        }
      }
      else
      {
        digitalWrite(EN_AT_CHARGING, HIGH);
        AUTO_CHARGING_RELAY_STATUS = true;
      }
    }
    else
    {
      digitalWrite(EN_AT_CHARGING, LOW);
      AUTO_CHARGING_RELAY_STATUS = false;
    }
  }
  else
  {
    digitalWrite(EN_AT_CHARGING, LOW);
    AUTO_CHARGING_RELAY_STATUS = false;
    t_auto_charging_jack_connect = millis();
  }
}
