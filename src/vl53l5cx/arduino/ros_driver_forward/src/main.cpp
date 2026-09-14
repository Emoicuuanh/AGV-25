#include <SparkFun_VL53L5CX_Library.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8.h>
#include <vl53l5cx/Safety_Esp.h>
#include <vl53l5cx/Vl53l5cxRanges.h>
// Phải có dòng comment này hoặc để trống 1 dòng nếu không VSCode tự sắp xếp
// lên trước <ros.h> thì sẽ không biên dịch được
#include <agv_msgs/DiffDriverMotorSpeed.h>

// Function declace
void i2cScanner();
void vl53l5cxSensorConfig(SparkFun_VL53L5CX& myImager_, byte sensorAddr,
                          byte oldAddr);
bool vl53l5cxSensorGetData(SparkFun_VL53L5CX&);
void sensorOn(int id);
void sensorOff(int id);
void enableSensor(int id);
void printData(int16_t* data);
byte getI2CAddress();
void errorCallback(SF_VL53L5CX_ERROR_TYPE errorCode, uint32_t errorValue);

// Global Variable
const int SENSOR_NUM = 2;
int imageResolution = 64;
int imageWidth = 8;
bool FORWARD = false;

int sensorAddress[SENSOR_NUM] = {0x15, 0x20};
int sensorResetPin[SENSOR_NUM] = {15, 4};
bool sensorEnable[SENSOR_NUM] = {true, true};
int sensorErrCnt[SENSOR_NUM] = {0, 0};
uint32_t ssErrStartTime[SENSOR_NUM] = {0, 0};

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
const long interval = 500;
int ledState = LOW;

ros::NodeHandle nh;
vl53l5cx::Vl53l5cxRanges lidar_msg;

ros::Publisher range_lidar_1("vl53l5cx_r1", &lidar_msg);
ros::Publisher range_lidar_2("vl53l5cx_r2", &lidar_msg);
ros::Publisher range_lidar_pub[SENSOR_NUM] = {range_lidar_1, range_lidar_2};

void setup()
{
  // Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  Serial.println("-----------------------Startup-----------------------");
  Wire.begin();
  Wire.setClock(200000);
  // First indicator after reset
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
  delay(200);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
  Serial.println("Get ID first time");
  i2cScanner();

  nh.initNode();

  // Setup enable pin for each sensor
  for (int id = 0; id < SENSOR_NUM; id++)
  {
    nh.advertise(range_lidar_pub[id]);  // advertise publish topic
    pinMode(sensorResetPin[id],
            OUTPUT);  // setmode output for reset pin of sensor
    digitalWrite(sensorResetPin[id], HIGH);  // Hold sensor all in reset
  }

  // Assign memory
  lidar_msg.range_length = 64;
  lidar_msg.range = (float*)malloc(sizeof(float) * 64);

  // Setup ID for each sensor
  for (int id = 0; id < SENSOR_NUM; id++)
  {
    // if (sensorEnable[id])
    {  // Enable only 1 sensor, disable other
      for (int i = 0; i < SENSOR_NUM; i++)
      {
        if (i == id)
          sensorOn(i);  // Bật sensor thứ i
        else
          sensorOff(i);  // Tắt sensor thứ i
      }
      delay(100);
      // Lấy địa chỉ của sensor
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
  }

  // Enable all sensors
  delay(100);
  Serial.println("Start 2 sensor. ");
  // Release all sensor
  for (int id = 0; id < SENSOR_NUM; id++)
  {
    sensorOn(id);
    delay(100);
    // digitalWrite(sensorResetPin[id], LOW);
    // Serial.print("Start sensor ");
    // Serial.println(id);
    bool result = myImager[id].startRanging();
    Serial.print("Start sensor ");
    Serial.print(id);
    Serial.print(" result: ");
    Serial.println(result);
  }

  Serial.println("Config 1 sensor done. ");
  Serial.println("Setup function done. ");
  // digitalWrite(LED, HIGH);
}

void loop()
{
  static uint32_t t;
  t = millis();
  // Serial.println("Imager Excute loop");
  for (int idx = 0; idx < SENSOR_NUM; idx++)
  {
    if (sensorEnable[idx])
    {
      // Test sensor
      // monitorSensorData(idx);
      // Get and publish data
      bool result = vl53l5cxSensorGetData(myImager[idx]);
      if (result)
      {
        lidar_msg.stamp = nh.now();
        range_lidar_pub[idx].publish(&lidar_msg);
      }
      else
      {
        digitalWrite(LED, LOW);
        Serial.print("Sensor error: ");
        Serial.println(idx);
        // loginfoNumber("Sensor error: ", idx, "");
        if (sensorErrCnt[idx] == 0)
        {
          ssErrStartTime[idx] = t;
          Serial.print("Begin error: ");
          Serial.println(idx);
        }
        sensorErrCnt[idx]++;

        if (t >= ssErrStartTime[idx] && t - ssErrStartTime[idx] < 1000)
        {
          // Error 4 times per second => real error
          if (sensorErrCnt[idx] > 3)
          {
            sensorErrCnt[idx] = 0;
            Serial.print("Real error: ");
            Serial.println(idx);
            // loginfoNumber("Real error: ", idx, "");
          }
          else
          {
            continue;
          }
        }
        else
        {
          ssErrStartTime[idx] = t;
          sensorErrCnt[idx] = 0;
          Serial.print("Reset cnt: ");
          Serial.println(idx);
          // loginfoNumber("Reset cnt: ", idx, "");
          // Serial.print("Time disconnect: ");
          // Serial.println(t - ssErrStartTime[idx]);
          // Serial.print("Error cnt: ");
          // Serial.println(sensorErrCnt[idx]);
          continue;
        }
        // // Setup enable pin for each sensor
        // for (int id = 0; id < SENSOR_NUM; id++)
        // {
        //   // nh.advertise(range_lidar_pub[id]);  // advertise publish topic
        //   pinMode(sensorResetPin[id],
        //           OUTPUT);  // setmode output for reset pin of sensor
        //   digitalWrite(sensorResetPin[id], HIGH);  // Hold sensor all in reset
        // }

        // // Assign memory
        // lidar_msg.range_length = 64;
        // lidar_msg.range = (float*)malloc(sizeof(float) * 64);

        // Setup ID for each sensor
        for (int id = 0; id < SENSOR_NUM; id++)
        {
          if (idx != id)
          {
            continue;
          }
          Serial.print("Restart sensor: ");
          Serial.println(idx);
          // loginfoNumber("Restart sensor: ", idx, "");
          // if (sensorEnable[id])
          {  // Enable only 1 sensor, disable other
            for (int i = 0; i < SENSOR_NUM; i++)
            {
              if (i == id)
                sensorOn(i);  // Bật sensor đang setup
              else
                sensorOff(i);  // Tắt các sensor còn lại
            }
            delay(100);
            // Lấy địa chỉ của sensor
            byte sensorAddr = 0;
            do
            {
              // loginfoNumber("Get I2C address ss: ", id, "");
              sensorAddr = getI2CAddress();
              if (sensorAddr != 0)
                break;
              delay(100);  // Loop until found at least sensor
            } while (sensorAddr == 0);

            // if (sensorAddr == 0)
            // {
            //   Serial.print("Sensor ");
            //   Serial.print(id);
            //   Serial.println(" not found. Please check again!");
            //   loginfoNumber("Sensor not found: ", idx, "");
            //   while (1)
            //     ;
            // }
            // else
            // {
            vl53l5cxSensorConfig(myImager[id], sensorAddress[id], sensorAddr);
            // }
          }
        }

        // Enable all sensors
        // delay(100);
        Serial.println("Start all sensors. ");
        // Release all sensor
        for (int id = 0; id < SENSOR_NUM; id++)
        {
          sensorOn(id);
          // delay(100);
          // digitalWrite(sensorResetPin[id], LOW);
          // Serial.print("Start sensor ");
          // Serial.println(id);
          bool result = myImager[id].startRanging();
          Serial.print("Start sensor ");
          Serial.print(id);
          Serial.print(" result: ");
          Serial.println(result);
          // loginfoNumber("Start sensor: ", id, "");
        }

        Serial.println("Config all sensors done. ");
      }
    }
    // else
    // {
    //   memset(lidar_msg.range, 0, sizeof(float) * 64);
    //   range_lidar_pub[id].publish(&lidar_msg);
    // }
  }

  // Blink led
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
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

  nh.spinOnce();
}

void vl53l5cxSensorConfig(SparkFun_VL53L5CX& myImager_, byte sensorAddr,
                          byte oldAddr)
{
  Serial.println("Config sensor ");
  Serial.print("Old address: ");
  Serial.println(oldAddr, HEX);
  Serial.print("New address: ");
  Serial.println(sensorAddr, HEX);

  // creat sensor
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
        Serial.print("Change address of sensor ");
        Serial.print(oldAddr, HEX);
        Serial.print(" to ");
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
  Serial.println("Sensor config done. ");

  Serial.println("Set error callback");
  myImager_.setErrorCallback(errorCallback);
  Serial.println("Finish set error callback");
}

bool vl53l5cxSensorGetData(SparkFun_VL53L5CX& myImager_)
{
  if (myImager_.isDataReady() == true)
  {
    // Serial.println("Data is ready");
    if (myImager_.getRangingData(&measurementData))
    {
      for (int id = 0; id < 64; id++)
      {
        if ((id % 8) == 0)
        {
          Serial.println();
        }
        if ((measurementData.nb_target_detected[id] > 0) &&
            (measurementData.distance_mm[id] > 0) &&
            ((measurementData.target_status[id] == 5) ||
             (measurementData.target_status[id] == 9)))
        {
          lidar_msg.range[id] = measurementData.distance_mm[id];
        }
        else
        {
          lidar_msg.range[id] = 0;
        }
      }
      delay(5);
      return true;
    }
    else
    {
      delay(5);
      return false;
    }
  }
  else
  {
    delay(5);
    return false;
  }
  // }
}

void monitorSensorData(int id)
{
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
    }
  }
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

  Serial.println("Scanning for I2C devices ...");
  for (address = 0x01; address < 0x7f; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      Serial.println(address, HEX);
      nDevices++;
    }
    else if (error != 2)
    {
      Serial.print("Error ");
      Serial.print(error);
      Serial.print(" at address 0x");
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

  Serial.println("Scanning for I2C devices ..................................");
  for (address = 0x01; address < 0x7f; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      Serial.println(address, HEX);
      return address;
      nDevices++;
    }
    else if (error != 2)
    {
      Serial.print("Error ");
      Serial.print(error);
      Serial.print(" at address 0x");
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