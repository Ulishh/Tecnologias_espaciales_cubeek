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
bool seleccion_sensor = false;
bool seleccion_tiempo = false;
int tiempo_sensado = 0;
int tiempo_inicial = 0;
int tiempo_actual = 0;

// RTC
RTC_DS1307 rtc;
bool rtc_enabled = false;
String year, month, day, hour, minute, second;
int int_tiempo;

// GPS variables:
SoftwareSerial GPS_SoftSerial(3, 4);
TinyGPSPlus gps;
float lat_val, lng_val;
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

//Adafruit_INA219 ina219;
const int temt6000 = A1;

// Variables para funcion de smart delay
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

const long intervalGPS = 2000; // 2 segundos entre lecturas de GPS
const long intervalIMU = 1000;  // 0.5 segundos entre lecturas de IMU
const long intervalBMP = 2000; // 1 segundo entre lecturas de BMP
const long intervalLUZ = 1000; // 0.5 segundos entre lecturas de BMP

// Prototipos de las funciones
void bmp_init();
void imu_init();
void gps_init();
void rtc_init();
void readbmp();
void readRTC();
void readIMU();
void sendSensorData();

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

 
  // payload init:
  imu_init();
  gps_init();
  bmp_init();
  rtc_init();
}

void loop()
{
  
  if(rtc_enabled) readRTC();
  if (!seleccion_sensor && !seleccion_tiempo){
  
    // Escucha de lora en espera de un comando
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        char receivedData[maxPacketSize];
        int bytesRead = LoRa.readBytes(receivedData, maxPacketSize);
    
        receivedData[bytesRead] = '\0';
        
        // Condiciones para activar la bandera de la opcion seleccionada desde el GS y para leer el sensor que se requiere
        //GPS
        if (strcmp(receivedData, "a") == 0){
          seleccion_sensor = true;
          Serial.println("Opcion a)");
          op1 = true;
        } 
        //IMU
        if (strcmp(receivedData, "b") == 0){
          seleccion_sensor = true;
          Serial.println("Opcion b)");
          op2 = true;
        } 
        //Sensor de presion y altitud
        if (strcmp(receivedData, "c") == 0){
          seleccion_sensor = true;
          Serial.println("Opcion c)");
          op3 = true;
        } 
//        // Sensor de luz
        if (strcmp(receivedData, "d") == 0){
          seleccion_sensor = true;
          Serial.println("Opcion d)");
          op4 = true;
        } 
        
        //Sensor de temperatura del BMP
        if (strcmp(receivedData, "e") == 0){
          seleccion_sensor = true;
          Serial.println("Opcion e)");
          op5 = true;
        } 
        
    }
  }
  else if (seleccion_sensor && !seleccion_tiempo){
    // Escucha de lora en espera de un comando
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        char receivedData[maxPacketSize];
        int bytesRead = LoRa.readBytes(receivedData, maxPacketSize);
    
        receivedData[bytesRead] = '\0';
        
        // Condiciones para asignar el tiempo de sensado
        //10 segundos de datos
        if (strcmp(receivedData, "a") == 0){
          Serial.println("10 seg");
          tiempo_sensado = 10;
          seleccion_tiempo = true;
        }
        else if (strcmp(receivedData, "b") == 0){
          Serial.println("30 seg");
          tiempo_sensado = 30;
          seleccion_tiempo = true;
        }
        else if (strcmp(receivedData, "c") == 0){
          Serial.println("1 minuto");
          tiempo_sensado = 60;
          seleccion_tiempo = true;
        }
    }
  }

  if (seleccion_sensor && seleccion_tiempo){
    if(rtc_enabled) readRTC();
    tiempo_actual = int_tiempo;
    tiempo_inicial = tiempo_actual;
    while(tiempo_actual-tiempo_inicial < tiempo_sensado){
      if(rtc_enabled) readRTC();
      tiempo_actual = int_tiempo;
      sendSensorData();      
    }
    seleccion_tiempo = false;
    seleccion_sensor = false;
    op1 = false;
    op2 = false;
    op3 = false;
    op4 = false;
    op5 = false;
  }
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
//  smartDelay(2000);
  bool loc_valid;
  loc_valid = gps.location.isValid();
  if(loc_valid)
  {
    lat_val = gps.location.lat();
    lng_val = gps.location.lng();
  }
  else{
  lat_val = -1.00;
  lng_val = -1.00;
  }
}

/* BMP085 Reading Functions Start!*/
void readbmp()
{
  altitude = bmp.readAltitude();
  pressure = bmp.readPressure() / 100.0F;
  tempr = bmp.readTemperature();
}
/* BMP085 Reading Functions End!*/


void readRTC()
{
//  smartDelay(1000);
  DateTime now = rtc.now();
  second = String(now.second());
  minute = String(now.minute());
  hour = String(now.hour());
//  day = String(now.day());
//  month = String(now.month());
//  year = String(now.year());
  int_tiempo = hour.toInt()*3600 + minute.toInt()*60 + second.toInt();
}




void sendSensorData()
{
  currentMillis = millis();

  //Paquetes de cada sensor para enviar por lora 
//  String reloj = "";
  String imu = "";
  String GPS = "";
  String presion = "";
  String temp = "";
  String luz = "";

  //IMU
  if(imu_enabled && op2)
  {
    if (currentMillis - previousMillis >= intervalIMU) {
      previousMillis = currentMillis;
      if (imu_enabled) {
        readIMU();  // Función para leer IMU
      }
      imu += int_tiempo;
      imu += "\n(YPR): ";
      imu += yaw;
      imu += ", ";
      imu += pitch;
      imu += ", ";
      imu += roll;
      imu += "\n";
      LoRa.beginPacket();
      LoRa.print(imu);
      LoRa.endPacket();
    }
    
  }
  // GPS
  if(gps_enabled && op1)
  {
    if (currentMillis - previousMillis >= intervalGPS) {
      previousMillis = currentMillis;
      if (gps_enabled) {
        readgps();
      }
      GPS += int_tiempo;
      GPS += "\n(Lat, Long): ";
      GPS += String(lat_val,5);
      GPS += ", ";
      GPS += String(lng_val,5);
      GPS += "\n";
      LoRa.beginPacket();
      LoRa.print(GPS);
      LoRa.endPacket();
    }
  }
  
  //Sensor de presion y altitud
  if(bmp_enabled && op3){
    if (currentMillis - previousMillis >= intervalBMP) {
      previousMillis = currentMillis;
      if (bmp_enabled) {
        readbmp();  // Función para leer BMP
      }
      presion += int_tiempo;
      presion += "\n";
      presion += pressure;
      presion += " hPa";
      LoRa.beginPacket();
      LoRa.print(presion);
      LoRa.endPacket();
      }
    
  }
  // Sensor de temperatura
  if(bmp_enabled && op5){
    if (currentMillis - previousMillis >= intervalBMP) {
      previousMillis = currentMillis;
      if (bmp_enabled) {
        readbmp();  // Función para leer BMP
      }
      temp += int_tiempo;
      temp += " \n";
      temp += tempr;
      temp += " C\n";
      LoRa.beginPacket();
      LoRa.print(temp);
      LoRa.endPacket();
      }
  }
  // Sensor de luz
  if(op4){
    if (currentMillis - previousMillis >= intervalLUZ) {
      previousMillis = currentMillis;
      int brightness = analogRead(temt6000);
      luz += int_tiempo;
      luz += "\nBrightness: ";
      luz += brightness;
      luz += " lm\n";
      LoRa.beginPacket();
      LoRa.print(luz);
      LoRa.endPacket();    
    }
  }
}
