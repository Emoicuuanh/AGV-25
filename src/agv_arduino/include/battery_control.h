#include "agv_arduino_define.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <SimpleKalmanFilter.h>
#define RS485_Serial Serial2

class BatteryControl
{
private:
  bool inChargingRange_;
  bool chargeToMaxLimit;

  float maxChargeLimit;
  float minChargeLimit;
  float maxBatteryPercent;
  unsigned long t_1;
  unsigned long r_1;
  float ampe;
  float vol;
  int state;
  int INIT;
  int CHECK_FIRST_TIME;
  int UPDATE;
  float stable_ample;
  float stable_ample_when_charge;
  uint32_t time_detect_ample_is_full_charge;
  bool is_charge_when_first_init;
  float addr_batt_percent;
  float addr_batt_ampe;
  float addr_batt_vol;
  float last_batt_percent;
  float last_batt_ampe;
  float last_batt_vol;
  uint32_t last_time_save_data;
  uint32_t time_update_battery_average;
  float vol_average;
  float ampe_average;
  int counter_update_battery_average;
  float time_waiting_when_init;
  int bytesRead;
  int false_couting;
  bool read;
  bool reset_100;

  // byte Vol_Command[13] ;
  byte Data_Modbus[13];
  byte battery_percent_command[13];

  // byte Vol_Command[13] = {0xA5,0x40,0x92,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7F}; //chua su dung
  // byte Current_charge[13] = {0xA5,0x40,0x93,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80};  //chua su dung
  byte vol_read[2];
  byte ampe_read[2];
  byte percent_read[2];
  float percent_return;
  byte z[2];

  // Read Current
  long convert_byte_to_dec(byte z[])
  {
    long value = 0;
    value += (long)z[0] << 8;
    value += (long)z[1];
    return value;
  }

public:
  BatteryControl();
  void dataHandle();
  float readCurrentCharging();
  float readBatteryVoltage();
  float readBatteryCapacity();
  bool inChargingRange();

};

BatteryControl::BatteryControl()
{
  inChargingRange_ = false;
  chargeToMaxLimit = false;

  maxChargeLimit = 100.0;
  minChargeLimit = 100.0;
  maxBatteryPercent = 100.0;

  ampe = 0.0;
  vol = 0.0;
  state = 0;
  INIT = 0;
  CHECK_FIRST_TIME = 1;
  UPDATE = 2;
  stable_ample = 4;
  stable_ample_when_charge = 1.5;
  time_detect_ample_is_full_charge = 0;
  is_charge_when_first_init = 0;
  addr_batt_percent = 0;
  addr_batt_ampe = 5;
  addr_batt_vol = 10;
  last_batt_percent = 0;
  last_batt_ampe = 0;
  last_batt_vol = 0;
  last_time_save_data = 0;
  time_update_battery_average = 0;
  vol_average = 0;
  ampe_average = 0;
  counter_update_battery_average = 0;
  time_waiting_when_init = 5000;
  percent_return = 0;
  bytesRead = 0;
  false_couting = 0;
  read = 0;
  t_1 = millis();
  reset_100 = 0;
}

void BatteryControl::dataHandle()
{

  // RS485_Serial.write(battery_percent_command,13);
  byte Vol_Command[13] = { 0xA5, 0x40, 0x90, 0x08, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x7D };
  if (RS485_Serial.availableForWrite() >= 13 && read == 0)
  {
    RS485_Serial.write(Vol_Command, 13);
  }

  // Kiểm tra và đọc dữ liệu từ RS485 Serial

  while (RS485_Serial.available() > 0 && read == 1)
  {
    byte incomingByte = RS485_Serial.read();

    Data_Modbus[bytesRead] = incomingByte;
    bytesRead++;
    // if (bytesRead >= 14) return void();
  }

  // Khi đã đọc đủ 13 byte, xử lý dữ liệu
  if (bytesRead == 13)
  {
    false_couting = 0;
    // Xử lý dữ liệu trong Data_Modbus

    ampe_read[0] = Data_Modbus[8];
    ampe_read[1] = Data_Modbus[9];
    ampe_average =
      static_cast<float>(convert_byte_to_dec(ampe_read)) / 10 - 3000;

    vol_read[0] = Data_Modbus[4];
    vol_read[1] = Data_Modbus[5];
    vol_average = static_cast<float>(convert_byte_to_dec(vol_read)) / 10;

    percent_read[0] = Data_Modbus[10];
    percent_read[1] = Data_Modbus[11];
    percent_return = static_cast<float>(convert_byte_to_dec(percent_read)) / 10;
    // Set BATTERY_FULL when percent_return >= 100
    if (percent_return >= maxBatteryPercent)
    {
      BATTERY_FULL = true;
    }
    else
    {
      BATTERY_FULL = false;
    }
    if (percent_return >= maxChargeLimit)
    {
      inChargingRange_ = false;
      chargeToMaxLimit = false;
    }
    else if (percent_return < minChargeLimit)
    {
      inChargingRange_ = true;
      chargeToMaxLimit = true;
    }
    else if (percent_return >= minChargeLimit &&
      percent_return < maxChargeLimit && chargeToMaxLimit)
    {
      //When battery percent is at minChargeLimit --> charge until reach maxChargeLimit
      inChargingRange_ = true;
    }
  }
  else
  {
    false_couting += 1;
  }
  bytesRead = 0;
  if (false_couting >= 5)
  {
    ampe_average = -99;
  }

  if (percent_return < 95) {
    reset_100 = 1;
  }

  // RESET 100% battery
  r_1 = millis() - t_1;

  if (vol_average >= 28.1 && (ampe_average >= -0.1 && ampe_average <= 0.1)) {
    if (reset_100 == 1) {
      if (r_1 > 300000) {

        byte reset_100_command[13] = { 0xA5, 0x40, 0x21, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE8, 0xF9 };
        if (RS485_Serial.availableForWrite() >= 13 && read == 0)
        {
          RS485_Serial.write(reset_100_command, 13);
          reset_100 = 0;
        }
      }
    }
  }
  else {
    t_1 = millis();
  }

  if (read == 0)
    read = 1;
  else
    read = 0;
}

float BatteryControl::readCurrentCharging() { return ampe_average; }

// Read Voltage
float BatteryControl::readBatteryVoltage() { return vol_average; }

// Manager battery capacity
float BatteryControl::readBatteryCapacity() { return percent_return; }

bool BatteryControl::inChargingRange() { return inChargingRange_; }
