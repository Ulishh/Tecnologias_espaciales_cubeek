#include <SPI.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <LoRa.h>
#include <Adafruit_BMP085.h>
#include <TinyGPS++.h>
#include <Adafruit_INA219.h>
#include <RTClib.h>

//LORA
const int maxPacketSize = 256; // Maximum expected packet size, adjust as needed

//Banderas para envio de sensores dependiendo de la opcion seleccionada desde el GS
bool op1, op2, op3, op4, op5;

// RTC
RTC_DS1307 rtc;
bool rtc_enabled = false;
String year, month, day, hour, minute, second;

// GPS variables:
SoftwareSerial GPS_SoftSerial(3, 4);
TinyGPSPlus gps;
double lat_val, lng_val;
bool gps_enabled = false;

// MPU6050 variables:
const int MPU = 0x69; // MPU6050 I2C address for AD0 High
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
bool imu_enabled = false;

// BMP085 variables:
Adafruit_BMP085 bmp;
float altitude, pressure, tempr;
bool bmp_enabled = false;

Adafruit_INA219 ina219;
const int temt6000 = A1;

void setup()
{
  Serial.begin(9600); 
  while (!Serial);
  Serial.println("CUBEEK Initializing...");

  // communication init:
  LoRa.setPins(8, 9, 2);
  if (!LoRa.begin(433E6))
  {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // housekeeping sensor init:
  ina219.begin();
  pinMode(temt6000,INPUT);

  // payload init:
  imu_init();
  gps_init();
  bmp_init();
  rtc_init();
}

void loop()
{
  if(rtc_enabled) readRTC();

  // Escucha de lora en espera de un comando
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
      Serial.print("Received Data From Satellite:\n");
      char receivedData[maxPacketSize];
      int bytesRead = LoRa.readBytes(receivedData, maxPacketSize);
  
      receivedData[bytesRead] = '\0';
      
      // Condiciones para activar la bandera de la opcion seleccionada desde el GS y para leer el sensor que se requiere
      //GPS
      if (strcmp(receivedData, "a") == 0){
        if(gps_enabled) readgps();
        op1 = true;
        op2 = false;
        op3 = false;
        op4 = false;
        op5 = false;
      } 
      //IMU
      else if (strcmp(receivedData, "b") == 0){
        if(imu_enabled) readIMU();
        op1 = false;
        op2 = true;
        op3 = false;
        op4 = false;
        op5 = false;
      } 
      //Sensor de presion y altitud
      else if (strcmp(receivedData, "c") == 0){
        if(bmp_enabled) readbmp();
        op1 = false;
        op2 = false;
        op3 = true;
        op4 = false;
        op5 = false;
      } 
      // Sensor de luz
      else if (strcmp(receivedData, "d") == 0){
        op1 = false;
        op2 = false;
        op3 = false;
        op4 = true;
        op5 = false;
      } 
      //Sensor de temperatura del BMP
      else if (strcmp(receivedData, "e") == 0){
        if(bmp_enabled) readbmp();
        op1 = false;
        op2 = false;
        op3 = false;
        op4 = false;
        op5 = true;
      } 
      // Condicion para detener el envio de datos
      else if (strcmp(receivedData, "s") == 0){
        op1 = false;
        op2 = false;
        op3 = false;
        op4 = false;
        op5 = false;

      }
//      Serial.println(receivedData);
      sendSensorData();
  }
  delay(100);      // Adjust the delay according to your data sending rate
}


/* Initialization Code Start!*/
void gps_init()
{
  gps_enabled = true;
  GPS_SoftSerial.begin(9600);
}

void bmp_init()
{
  bmp_enabled = true;
  if (!bmp.begin())
  {
    Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
    while (1);
  }
}

void imu_init()
{
  imu_enabled = true;
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  calculate_IMU_error();
}

void rtc_init()
{
  rtc_enabled = true;
  if (!rtc.begin()) {
      Serial.println("Couldn't find RTC");
      while (1);
      }
   
   if (!rtc.isrunning()) {
     Serial.println("RTC is NOT running, let's set the time!");
     // The following line sets the RTC to the date & time this sketch was compiled
     rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
     // This line sets the RTC with an explicit date & time, for example to set
     // January 21, 2024 at 3:00am you would call:
     // rtc.adjust(DateTime(2024, 1, 21, 3, 0, 0));
    }
}

/* Initialization Code End!*/

/* IMU Reading Functions Start!*/
void calculate_IMU_error()
{
  int c = 0;

  while (c < 200)
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccErrorX += atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI;
    AccErrorY += atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI;
    c++;
  }
  AccErrorX /= 200;
  AccErrorY /= 200;

  c = 0;

  while (c < 200)
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroErrorX += GyroX / 131.0;
    GyroErrorY += GyroY / 131.0;
    GyroErrorZ += GyroZ / 131.0;
    c++;
  }
  GyroErrorX /= 200;
  GyroErrorY /= 200;
  GyroErrorZ /= 200;
}

void readIMU() 
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + AccErrorY;

  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000;

  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  GyroX += GyroErrorX;
  GyroY += GyroErrorY;
  GyroZ += GyroErrorZ;

  gyroAngleX += GyroX * elapsedTime;
  gyroAngleY += GyroY * elapsedTime;
  yaw += GyroZ * elapsedTime;

  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
}
/* IMU Reading Functions End!*/

/* GPS Reading Functions Start!*/
void readgps()
{
  smartDelay(1000);
  bool loc_valid;
  loc_valid = gps.location.isValid();
  if(loc_valid)
  {
    lat_val = gps.location.lat();
    lng_val = gps.location.lng();
  }
  else{
  lat_val = -1.000000;
  lng_val = -1.000000;
  }
}

void readRTC()
{
//  smartDelay(1000);
  DateTime now = rtc.now();
  second = String(now.second());
  minute = String(now.minute());
  hour = String(now.hour());
  day = String(now.day());
  month = String(now.month());
  year = String(now.year());
}

void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (GPS_SoftSerial.available()) 
      gps.encode(GPS_SoftSerial.read());
  } while (millis() - start < ms);
}
/* GPS Reading Functions End!*/

/* BMP085 Reading Functions Start!*/
void readbmp()
{
  altitude = bmp.readAltitude();
  pressure = bmp.readPressure() / 100.0F;
  tempr = bmp.readTemperature();
}
/* BMP085 Reading Functions End!*/


void sendSensorData()
{
  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float power_mW = ina219.getPower_mW();
  float loadvoltage = busvoltage + (shuntvoltage / 1000);
  int brightness = analogRead(temt6000);

  //Paquetes de cada sensor para enviar por lora 
  String packet = "";
  String reloj = "";
  String imu = "";
  String GPS = "";
  String presion = "";
  String temp = "";
  String luz = "";
  String data = "";

  // Reloj
  if(rtc_enabled)
  {
    reloj += hour;
    reloj += ":";
    reloj += minute;
    reloj += ":";
    reloj += second;
    reloj += ".\n";
  }
  //IMU
  if(imu_enabled && op2)
  {
    imu += reloj;
    imu += "Orientation (YPR): ";
    imu += yaw;
    imu += ", ";
    imu += pitch;
    imu += ", ";
    imu += roll;
    imu += ".\n";
    Serial.println(imu);
    LoRa.beginPacket();
    LoRa.print(imu);
    LoRa.endPacket();
  }
  // GPS
  if(gps_enabled && op1)
  {
    GPS += reloj;
    GPS += "(Lat, Long): ";
    GPS += String(lat_val,6);
    GPS += ", ";
    GPS += String(lng_val,6);
    GPS += ".\n";
    Serial.println(GPS);
    LoRa.beginPacket();
    LoRa.print(GPS);
    LoRa.endPacket();
  }
  //Sensor de presion y altitud
  if(bmp_enabled && op3)
  {
    presion += reloj;
    presion += "With altitude ";
    presion += altitude;
    presion += " m in , ";
    presion += pressure;
    presion += " hPa pressure &, ";
    Serial.println(presion);
    LoRa.beginPacket();
    LoRa.print(presion);
    LoRa.endPacket();
  }
  // Sensor de temperatura
  if(bmp_enabled && op5){
    temp += reloj;
    temp += tempr;
    temp += "°C.\n";
    Serial.println(temp);
    LoRa.beginPacket();
    LoRa.print(temp);
    LoRa.endPacket();
  }
  // Sensor de luz
  if(op4){
    luz += reloj;
    luz += " Brightness: ";
    luz += brightness;
    luz += " lm\n";
    Serial.println(luz);
    LoRa.beginPacket();
    LoRa.print(luz);
    LoRa.endPacket();
  }
}

//  data = "Bus & shunt voltage: ";
//  data += busvoltage;
//  data += " V &";
//  data += shuntvoltage;
//  data += " mV and";
//  data += " Current: ";
//  data += current_mA;
//  data += " mA";
//  data += " Brightness: ";
//  data += brightness;
//  data += " lm\n";
  
//  if(imu_enabled || gps_enabled || bmp_enabled || rtc_enabled) Serial.println(imu);
////  Serial.println(data);
//  LoRa.beginPacket();
//  if(imu_enabled || gps_enabled || bmp_enabled || rtc_enabled) LoRa.print(imu);
////  LoRa.print(data);
//  LoRa.endPacket();
