#ifndef DYP_ULTRA_SONIC_H_
#define DYP_ULTRA_SONIC_H_

#define ULTRASONIC_TIMER 100
#define ALL_ULTRASONIC_TIMER 250
#define ALL_SAFETY_SENSOR 0
#define ULTRASONIC_START_CODE 0x55

bool Read_Ultrasonic(int sensor_head);
bool Request_Ultrasonic(int sensor_head);

uint16_t UltraSonicData[8] = {0, 0, 0, 0, 0, 0, 0, 0};
static uint32_t tUpdateUltraSonic;

#ifdef ULTRASONIC_PORT

bool Request_Ultrasonic(int sensor_head)
{
  byte request_data[6] = {0x55, 0xAA, 0x01, 0x01, 0x01};

  if (sensor_head == 1)
  {
    request_data[3] = request_data[4] = 0x10;
  }
  else if (sensor_head == 2)
  {
    request_data[3] = request_data[4] = 0x11;
  }
  else if (sensor_head == 3)
  {
    request_data[3] = request_data[4] = 0x12;
  }
  else if (sensor_head == 4)
  {
    request_data[3] = request_data[4] = 0x13;
  }

  for (int i = 0; i < 5; i++)
  {
    ULTRASONIC_PORT.write(request_data[i]);
  }
  return true;
}

bool Read_Ultrasonic(int sensor_head)
{
  // 0X55 0XAA 0X01 0X01 0X03 0XE8 SUM
  // 0X55 0XAA 0X01 0X01 0X03 0XE8 0X07 0XD0 0X07 0XA1 0X0D 0XEA SUM
  // ['U', '\xaa', '\x01', '\x01', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x01']
  static int ReceiveStep = 0;
  static int DataCounter = 0;
  static byte data_ultrasonic[14];
  int ReceiveData = 0;
  int dataLen = 0;

  if (sensor_head == ALL_SAFETY_SENSOR)
  {
    dataLen = 12;
  }
  else
  {
    dataLen = 6;
  }

  if (ULTRASONIC_PORT.available() > 0)
  {
    ReceiveData = ULTRASONIC_PORT.read();

    switch (ReceiveStep)
    {
    // Header: 0x55
    case 0:
      if (ReceiveData == ULTRASONIC_START_CODE)
      {
        data_ultrasonic[0] = ReceiveData;
        DataCounter = 1;
        ReceiveStep = 1;
        // Serial.print(ReceiveData);
        // Serial.print(", ");
      }
      break;
    // Data Ultrasonic
    case 1:
      data_ultrasonic[DataCounter ++] = ReceiveData;
      // Serial.print(ReceiveData);
      if (DataCounter > dataLen)
      {
        ReceiveStep = 2;
      }
      else
      {
        // Serial.print(", ");
      }
      break;
    // Sum check
    case 2:
      // Serial.println("");
      int sum_calc = 0;
      for (int i = 0; i < dataLen; i++)
      {
        sum_calc += data_ultrasonic[i];
      }
      sum_calc &= 0x00FF;
      if (sum_calc == data_ultrasonic[dataLen])
      {
        if (sensor_head == 0)
        {
          UltraSonicData[0] = (data_ultrasonic[4] << 8) | (data_ultrasonic[5]);
          UltraSonicData[1] = (data_ultrasonic[6] << 8) | (data_ultrasonic[7]);
          UltraSonicData[2] = (data_ultrasonic[8] << 8) | (data_ultrasonic[9]);
          UltraSonicData[3] = (data_ultrasonic[10] << 8) | (data_ultrasonic[11]);
        }
        else
        {
          uint16_t sensor_value = (data_ultrasonic[4] << 8) | (data_ultrasonic[5]);
          if (sensor_head == 1)
          {
            UltraSonicData[0] = sensor_value;
          }
          else if (sensor_head == 2)
          {
            UltraSonicData[1] = sensor_value;
          }
          else if (sensor_head == 3)
          {
            UltraSonicData[2] = sensor_value;
          }
          else if (sensor_head == 4)
          {
            UltraSonicData[3] = sensor_value;
          }
        }
        // Serial.print(UltraSonicData[0]);
        // Serial.print(", ");
        // Serial.print(UltraSonicData[1]);
        // Serial.print(", ");
        // Serial.print(UltraSonicData[2]);
        // Serial.print(", ");
        // Serial.println(UltraSonicData[3]);
      }
      ReceiveStep = 0;
      return true;
      break;
    }
  }
  return false;
}

#endif
#endif