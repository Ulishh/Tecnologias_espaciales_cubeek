//Librerias lora
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
//Librerias pantalla oled
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//Resolucion oled
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

// Definimos la dirección I2C de la pantalla OLED (puede variar según el modelo)
#define OLED_ADDRESS 0x3C

// Crear un objeto para la pantalla OLED usando I2C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const int maxPacketSize = 256; // Maximum expected packet size, adjust as needed

// Variables bool para definir estados
bool menu = false;
bool recepcion = false;

void setup() {
  //Iniciamos el puerto serial
  Serial.begin(9600);
  while (!Serial);

  // Inicializar la pantalla OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("No se pudo encontrar una pantalla OLED"));
    //while(1);  // Detener ejecución si no se encuentra la pantalla
  }

  //Configuraciones para lora
  LoRa.setPins(8, 9, 2);
  //Serial.println("LoRa Receiver");
  
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // Limpiar la pantalla oled al inicio
  display.clearDisplay();
  display.display();

}

void loop() {
  // Verifica si hay datos disponibles en el buffer serial
  if (Serial.available() > 0) {
    // Lee el dato recibido
    char receivedChar = Serial.read();
    
    // Verifica el caracter recibido y lo compara con las opciones disponibles 
    if (receivedChar == 'q') {
      menu = true;
      recepcion = false;
    }
    if (receivedChar == 'a'){
        Serial.println("GPS...");
        LoRa.beginPacket();
        LoRa.print('a');
        LoRa.endPacket();
        recepcion = true;
      }
      if (receivedChar == 'b'){
        Serial.println("IMU...");
        LoRa.beginPacket();
        LoRa.print('b');
        LoRa.endPacket();
        recepcion = true;
      }
      if (receivedChar == 'c'){
        Serial.println("Presion...");
        LoRa.beginPacket();
        LoRa.print('c');
        LoRa.endPacket();
        recepcion = true;
      }
      if (receivedChar == 'd'){
        Serial.println("Luz...");
        LoRa.beginPacket();
        LoRa.print('d');
        LoRa.endPacket();
        recepcion = true;
      }
      if (receivedChar == 'e'){
        Serial.println("Temperatura...");
        LoRa.beginPacket();
        LoRa.print('e');
        LoRa.endPacket();
        recepcion = true;
      }
      if (receivedChar == 's'){
        Serial.println("Detener");
        LoRa.beginPacket();
        LoRa.print('s');
        LoRa.endPacket();
        recepcion = false;
      }
  }
  if (menu){
    Serial.println("Sensores payload. Seleccione una opcion:");
    Serial.println("a) GPS.");
    Serial.println("b) IMU.");
    Serial.println("c) Presion.");
    Serial.println("d) Luz.");
    Serial.println("e) Temperatura.");
    Serial.println("s) Detener.");
    menu = false;
  }

  // Recibe los paquetes que mande el OBC y los despliega en el serial y la pantalla oled
  if (!menu && recepcion){
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      Serial.println("Received Data From Satellite:\n");
      char receivedData[maxPacketSize];
      int bytesRead = LoRa.readBytes(receivedData, maxPacketSize);
      receivedData[bytesRead] = '\0';
      Serial.println(receivedData);
      
      //Muestra en la pantalla oled el mensaje recibido por lora
      display.clearDisplay();         // Limpiar la pantalla
      display.setTextSize(1);         // Establecer el tamaño del texto
      display.setTextColor(WHITE);    // Establecer color del texto
      display.setCursor(0, 0);      // Posicionar el cursor en el centro
      display.print(receivedData);               // Imprimir el número
      display.display();              // Mostrar en la pantalla
      Serial.println();
    }
  }
  
}   
