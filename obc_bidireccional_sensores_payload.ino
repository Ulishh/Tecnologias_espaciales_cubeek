#include <SPI.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <LoRa.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Adafruit_BMP085.h>
#include <TinyGPS++.h>
#include <Adafruit_INA219.h>
#include <RTClib.h>
#include <math.h>

//LORA
const int maxPacketSize = 256; // Maximum expected packet size, adjust as needed

//Banderas para envio de sensores dependiendo de la opcion seleccionada desde el GS
bool op1, op2, op3, op4, op5, op6, op7;
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

/*********** IMU ***********/
// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito
//MPU6050 sensor;
MPU6050 sensor(0x69);
// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;
long tiempo_prev;
float dt;
float ang_x, ang_y, ang_z;
float ang_x_prev, ang_y_prev, ang_z_prev;

// BMP085 variables:
Adafruit_BMP085 bmp;
float altitude, pressure, tempr;
bool bmp_enabled = false;

// INA 219
Adafruit_INA219 ina219;
const int temt6000 = A1;

// Variables para funcion de smart delay
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

// Intervalos de tiempo para la lectura de los sensores
const long intervalSEN = 200; // 5 segundos entre lecturas de GPS
const long intervalIMU = 200; // 5 segundos entre lecturas de GPS
const long intervalTEMP = 500; // 1 segundo entre lecturas de BMP
const long intervalLUZ = 200; // 0.5 segundos entre lecturas de BMP
const long intervalINA = 2000; // 0.5 segundos entre lecturas de BMP

// Inicializacion de las funciones
void bmp_init();
void gps_init();
void rtc_init();
void readbmp();
void readRTC();
void readIMU();
void sendSensorData();
void smartDelay();


void setup()
{
//  Serial.begin(9600); 
//  while (!Serial);
//  Serial.println("CUBEEK Initializing...");
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor
//  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
//  else Serial.println("Error al iniciar el sensor");
  // Valores de una calibracion previa del IMU
  sensor.setXAccelOffset(-4937);
  sensor.setYAccelOffset(-2338);
  sensor.setZAccelOffset(1167);
  sensor.setXGyroOffset(325);
  sensor.setYGyroOffset(49);
  sensor.setZGyroOffset(-5);

  // communication init:
  LoRa.setPins(8, 9, 2);
  if (!LoRa.begin(433E6))
  {
//    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // housekeeping sensor init:
  ina219.begin();

  // payload init:
  gps_init();
  bmp_init();
  rtc_init();
}

void loop()
{
//  if(rtc_enabled) readRTC();
// Condicional en la cual se selecciona el sensor para su lectura y envio
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
          op1 = true;
        } 
        //IMU
        if (strcmp(receivedData, "b") == 0){
          seleccion_sensor = true;
          op2 = true;
        } 
        //Sensor de presion
        if (strcmp(receivedData, "c") == 0){
          seleccion_sensor = true;
          op3 = true;
        } 
        // Sensor de luz
        if (strcmp(receivedData, "d") == 0){
          seleccion_sensor = true;
          op4 = true;
        } 
        //Sensor de temperatura del BMP
        if (strcmp(receivedData, "e") == 0){
          seleccion_sensor = true;
          op5 = true;
        } 
        //Sensor de voltaje
        if (strcmp(receivedData, "f") == 0){
          seleccion_sensor = true;
          op6 = true;
        } 
        //Sensor de corriente
        if (strcmp(receivedData, "g") == 0){
          seleccion_sensor = true;
          op7 = true;
        } 
        
    }
  }
  // Condicional para el seleccionar el tiempo de sensado
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
          tiempo_sensado = 20;
          seleccion_tiempo = true;
        }
        else if (strcmp(receivedData, "b") == 0){
          tiempo_sensado = 40;
          seleccion_tiempo = true;
        }
        else if (strcmp(receivedData, "c") == 0){
          tiempo_sensado = 60;
          seleccion_tiempo = true;
        }
    }
  }
// Condicional para enviar los datos de los sensores por el tiempo de sensado seleccionado
  if (seleccion_sensor && seleccion_tiempo){
    tiempo_inicial = millis()/1000;
    while(tiempo_actual-tiempo_inicial < (tiempo_sensado)){
      tiempo_actual = millis()/1000;
      sendSensorData();      
    }
    // Se vuelven todas las banderas en false
    seleccion_tiempo = false;
    seleccion_sensor = false;
    op1 = false;
    op2 = false;
    op3 = false;
    op4 = false;
    op5 = false;
    op6 = false;
    op7 = false;
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
//    Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
    while (1);
  }
}



void rtc_init()
{
  rtc_enabled = true;
  if (!rtc.begin()) {
//      Serial.println("Couldn't find RTC");
      while (1);
      }
   
   if (!rtc.isrunning()) {
//     Serial.println("RTC is NOT running, let's set the time!");
     // The following line sets the RTC to the date & time this sketch was compiled
     rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
     // This line sets the RTC with an explicit date & time, for example to set
     // January 21, 2024 at 3:00am you would call:
     // rtc.adjust(DateTime(2024, 1, 21, 3, 0, 0));
    }
}

/* Initialization Code End!*/


void readIMU() 
{
  // Leer las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);
  
  float accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/PI);
  float accel_ang_y=atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/PI);
//Calcular angulo de rotación con giroscopio y filtro complemento  
  dt = (millis()-tiempo_prev)/1000.0;
  tiempo_prev=millis();
  ang_x = 0.98*(ang_x_prev+(gx/131)*dt) + 0.02*accel_ang_x;
  ang_y = 0.98*(ang_y_prev+(gy/131)*dt) + 0.02*accel_ang_y;
  ang_z = ang_z_prev+(gz/131)* dt;
  ang_x_prev=ang_x;
  ang_y_prev=ang_y;
  ang_z_prev=ang_z;
}
/* IMU Reading Functions End!*/

/* GPS Reading Functions Start!*/
void readgps()
{
  smartDelay(2000);
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


// Funcion para envio de datos de los sensores
void sendSensorData()
{
  readRTC(); // Se lee el rtc para enviar el tiempo en cada paquete
  currentMillis = millis(); // Se obtiene el tiempo transcurrido para enviar datos cada cierto intervalo de tiempo

  //Paquetes de cada sensor para enviar por lora 
  String imu = "";
  String GPS = "";
  String presion = "";
  String temp = "";
  String luz = "";
  String busvol = "";
  String curr = "";
  
  // GPS
  if(gps_enabled && op1)
  {
    if (currentMillis - previousMillis >= intervalSEN) {
      previousMillis = currentMillis;
      readgps();
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

  //IMU
  if(op2)
  {
    if (currentMillis - previousMillis >= intervalIMU) {
      previousMillis = currentMillis;
      readIMU();  // Función para leer IMU
      imu += int_tiempo;
      imu += "\n(YPR): ";
      imu += ang_z;
      imu += ", ";
      imu += ang_y;
      imu += ", ";
      imu += ang_x;
      imu += "\n";
      LoRa.beginPacket();
      LoRa.print(imu);
      LoRa.endPacket();
    }
  }
  
  //Sensor de presion 
  if(bmp_enabled && op3){
    if (currentMillis - previousMillis >= intervalSEN) {
      previousMillis = currentMillis;
      readbmp();  // Función para leer BMP
      presion += int_tiempo;
      presion += "\n";
      presion += pressure;
      presion += " hPa\n";
      LoRa.beginPacket();
      LoRa.print(presion);
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

  // Sensor de temperatura
  if(bmp_enabled && op5){
    if (currentMillis - previousMillis >= intervalTEMP) {
      previousMillis = currentMillis;
      readbmp();  // Función para leer BMP
      temp += int_tiempo;
      temp += " \n";
      temp += tempr;
      temp += " C\n";
      LoRa.beginPacket();
      LoRa.print(temp);
      LoRa.endPacket();
      }
  }

  // Sensor INA (Bus voltage)
  if (op6){
    if (currentMillis - previousMillis >= intervalINA) {
      previousMillis = currentMillis;
      float busvoltage = ina219.getBusVoltage_V();
      busvol += int_tiempo;
      busvol += "\nBus voltage: ";
      busvol += busvoltage;
      busvol += " V\n";
      LoRa.beginPacket();
      LoRa.print(busvol);
      LoRa.endPacket();
    }
  }

  // Sensor INA (Current)
  if (op7){
    if (currentMillis - previousMillis >= intervalINA) {
      previousMillis = currentMillis;
      float current_mA = ina219.getCurrent_mA();
      curr += int_tiempo;
      curr += "\nCurrent: ";
      curr += current_mA;
      curr += " mA\n";
      LoRa.beginPacket();
      LoRa.print(curr);
      LoRa.endPacket();
    }
  }
}
