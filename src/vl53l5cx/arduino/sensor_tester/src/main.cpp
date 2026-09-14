#include <SparkFun_VL53L5CX_Library.h>
#include <Wire.h>

// Function declace
void i2cScanner();
void vl53l5cxSensorConfig(SparkFun_VL53L5CX& myImager_, byte sensorAddr,
                          byte oldAddr);
void vl53l5cxSensorGetData(SparkFun_VL53L5CX&);
void vl53l5cxSensorGetData(int id);
void sensorOn(int id);
void sensorOff(int id);
void enableSensor(int id);
void printData(int16_t* data);
byte getI2CAddress();
void errorCallback(SF_VL53L5CX_ERROR_TYPE errorCode, uint32_t errorValue);

// Global Variable
const int SENSOR_NUM = 4;
int imageResolution = 64;
int imageWidth = 8;
bool FORWARD = false;
const int SENSOR_ONLY = -1;  // -1: enable all sensors

int sensorAddress[SENSOR_NUM] = {0x15, 0x20, 0x25, 0x30};
int sensorResetPin[SENSOR_NUM] = {15, 4, 14, 12};
bool sensorEnable[SENSOR_NUM] = {true, false, false, false};

int dir = 2;
const int FILTER_NUM = 3;
const int ERROR_THRESH = 200;
float data[64], sumError[64];
int error = 50;
SparkFun_VL53L5CX myImager[SENSOR_NUM];
VL53L5CX_ResultsData measurementData;

// Mảng kết quả hiện tại và trước đó int16_t 8x8
int16_t results[64];
int16_t resultsError[64];

#define LED 2
unsigned long previousMillis = 0;
const long intervalLed = 500;
int ledState = LOW;

void blinkOnce()
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
}

void blinkTwice()
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
  delay(200);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
}

void setup()
{
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  blinkTwice();
  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("-----------------------Starting up-----------------------");
  if (SENSOR_ONLY == -1)
  {
    Serial.println("Use all sensors");
  }
  else
  {
    Serial.print("Only use sensor with index: ");
    Serial.println(SENSOR_ONLY);
  }
  Wire.begin();
  // Serial.println("Get ID first time");
  // i2cScanner();
  Wire.setClock(200000);
  Serial.println("Setup I2C done!");
  Serial.println("Begin setup ID");
  // Setup enable pin for each sensor
  for (int id = 0; id < SENSOR_NUM; id++)
  {
    pinMode(sensorResetPin[id],
            OUTPUT);  // setmode output for reset pin of sensor
    digitalWrite(sensorResetPin[id], HIGH);  // Hold sensor all in reset
  }
  // Setup ID for each sensor
  for (int id = 0; id < SENSOR_NUM; id++)
  {
    Serial.println("");
    Serial.print("Begin setup sensor index: ");
    Serial.println(id);
    {  // Enable only 1 sensor, disable other
      for (int i = 0; i < SENSOR_NUM; i++)
      {
        if (i == id)
          sensorOn(i);   // Bật sensor thứ i
        else
          sensorOff(i);  // Tắt sensor thứ i
      }
      delay(100);
      // Lấy địa chỉ của sensor
      bool enable = false;
      if (SENSOR_ONLY == -1)
      {
        enable = true;
      }
      else
      {
        if (id == SENSOR_ONLY)
        {
          enable = true;
        }
      }
      if (!enable)
      {
        Serial.print("Ignore setup sensor index: ");
        Serial.println(id);
        continue;
      }

      byte sensorAddr = getI2CAddress();
      if (sensorAddr == 0)
      {
        Serial.print("Sensor ");
        Serial.print(id);
        Serial.println(" not found. Please check again!");
        while (1)
          ;
      }
      else
      {
        vl53l5cxSensorConfig(myImager[id], sensorAddress[id], sensorAddr);
      }
    }
    blinkOnce();
  }
  // Enable all sensors after config
  delay(1000);
  blinkTwice();
  Serial.println("");
  Serial.println("Enable all sensors after config.");
  // Release all sensor
  for (int id = 0; id < SENSOR_NUM; id++)
  {
    bool enable = false;
    if (SENSOR_ONLY == -1)
    {
      enable = true;
    }
    else
    {
      if (id == SENSOR_ONLY)
      {
        enable = true;
      }
    }
    if (!enable)
    {
      Serial.print("Ignore enable sensor index: ");
      Serial.println(id);
      continue;
    }
    Serial.print("Enable sensor ");
    Serial.println(id);
    sensorOn(id);
    delay(100);
    // digitalWrite(sensorResetPin[id], LOW);
    bool result = myImager[id].startRanging();
    Serial.print("Start sensor ");
    Serial.print(id);
    Serial.print(" result: ");
    if (result == 1)
      Serial.println("OK");
    else
      Serial.println("NG");
  }

  Serial.println("Config 4 sensor done.");
  Serial.println("Setup function done.");
  blinkTwice();
}

void loop()
{
  // Serial.println("Imager Excute loop");
  for (int id = 0; id < SENSOR_NUM; id++)
  {
    bool enable = false;
    if (SENSOR_ONLY == -1)
    {
      enable = true;
    }
    else
    {
      if (id == SENSOR_ONLY)
      {
        enable = true;
      }
    }
    if (!enable)
    {
      Serial.print("Ignore read sensor index: ");
      Serial.println(id);
      continue;
    }
    if (sensorEnable[id] && SENSOR_ONLY == -1 ||
        SENSOR_ONLY == id && SENSOR_ONLY > -1)
    {
      vl53l5cxSensorGetData(id);
      Serial.println("---");
    }
  }

  // Blink led
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= intervalLed)
  {
    previousMillis = currentMillis;
    if (ledState == LOW)
    {
      ledState = HIGH;
    }
    else
    {
      ledState = LOW;
    }
    digitalWrite(LED, ledState);
  }
}

void vl53l5cxSensorConfig(SparkFun_VL53L5CX& myImager_, byte sensorAddr,
                          byte oldAddr)
{
  Serial.println("Begin config sensor");
  Serial.print("Old address: 0x");
  Serial.println(oldAddr, HEX);
  Serial.print("New address: 0x");
  Serial.println(sensorAddr, HEX);

  // Create sensor
  Serial.print("Connecting to sensor with old address: 0x");
  Serial.println(oldAddr, HEX);
  if (myImager_.begin(oldAddr) == false)
  {
    Serial.print("Failed to communicate with sensor ");
    Serial.print(oldAddr, HEX);
    Serial.println("! Please check again!");
    while (1)
      ;
  }
  else
  {
    myImager_.setResolution(8 * 8);
    myImager_.setRangingFrequency(20);
    if (oldAddr != sensorAddr)
    {
      Serial.println("Start change address of sensor");
      // Thay đổi địa chỉ của sensor
      if (myImager_.setAddress(sensorAddr))
      {
        Serial.print("Change address of sensor from 0x");
        Serial.print(oldAddr, HEX);
        Serial.print(" to 0x");
        Serial.println(sensorAddr, HEX);
      }
      else
      {
        Serial.print("Failed to change address of sensor ");
        Serial.print(oldAddr, HEX);
        Serial.print(" to ");
        Serial.println(sensorAddr, HEX);
        Serial.println("Please check again!");
        while (1)
          ;
      };
    }
    // myImager_.startRanging();
  }
  Serial.println("Sensor config done.");
  Serial.println("Begin set error callback");
  myImager_.setErrorCallback(errorCallback);
  Serial.println("Finish set error callback");
}

void vl53l5cxSensorGetData(int id)
{
  // int data_num = 0;
  // while (data_num < FILTER_NUM)
  // {
  if (Serial.available())
  {
    int eSensor = Serial.read();
    if (eSensor > 3)
    {
      Serial.print("Set error from ");
      Serial.print(error);
      Serial.print(" to ");
      Serial.println(eSensor);
      error = eSensor;
    }
    else
    {
      enableSensor(eSensor);
    }
  }

  if (myImager[id].isDataReady() == true)
  {
    // Serial.println("Data is ready");
    // lidar_msg.status = 1;
    if (myImager[id].getRangingData(&measurementData))
    {
      bool isPrint = false;
      for (int id = 0; id < 64; id++)
      {
        // if ((id % 8) == 0)
        // {
        //   Serial.println();
        // }
        // Serial.print("\t");
        // Serial.print(measurementData.distance_mm[id]);
        // nếu như giá trị đo được lêch quá nhiều so với giá trị trước đó thì in ra
        if (abs(measurementData.distance_mm[id] - results[id]) > error)
        {
          isPrint = true;
        }
        resultsError[id] = abs(measurementData.distance_mm[id] - results[id]);
        results[id] = measurementData.distance_mm[id];
      }
      if (isPrint)
      {
        printData(resultsError);
      }

      // Serial.println();
      // }
    }
  }
  // delay(5);
  // }
}

void errorCallback(SF_VL53L5CX_ERROR_TYPE errorCode, uint32_t errorValue)
{
  Serial.print("Error: ");
  Serial.println(errorValue);
  delay(3000);
}

void i2cScanner()
{
  byte error, address;
  int nDevices = 0;

  delay(2000);

  Serial.println("Scanning for all I2C devices ...");
  for (address = 0x01; address < 0x7f; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address: 0x");
      Serial.println(address, HEX);
      nDevices++;
    }
    else if (error != 2)
    {
      Serial.print("Error ");
      Serial.print(error);
      Serial.print(" at address: 0x");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
  {
    Serial.println("No I2C devices found");
  }
}

byte getI2CAddress()
{
  byte error, address;
  int nDevices = 0;

  delay(1000);
  Serial.println("Get I2C devices address ...");
  for (address = 0x01; address < 0x7f; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address: 0x");
      Serial.println(address, HEX);
      return address;
      nDevices++;
    }
    else if (error != 2)
    {
      Serial.print("Error ");
      Serial.print(error);
      Serial.print(" at address: 0x");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
  {
    Serial.println("No I2C devices found");
  }
  return address == 0x7f ? 0 : address;
}

void sensorOn(int id) { digitalWrite(sensorResetPin[id], LOW); }

void sensorOff(int id) { digitalWrite(sensorResetPin[id], HIGH); }

void printData(int16_t* data)
{
  for (int i = 0; i < 64; i++)
  {
    if ((i % 8) == 0)
    {
      Serial.println();
    }
    Serial.print("\t");
    Serial.print(data[i]);
  }
  Serial.println();
}

void enableSensor(int id)
{
  if (id >= SENSOR_NUM)
  {
    id = SENSOR_NUM - 1;
  }
  Serial.print("Enable Sensor ");
  Serial.println(id);
  for (int i = 0; i < SENSOR_NUM; i++)
  {
    if (i == id)
    {
      sensorEnable[i] = true;
    }
    else
    {
      sensorEnable[i] = false;
    }
  }
}