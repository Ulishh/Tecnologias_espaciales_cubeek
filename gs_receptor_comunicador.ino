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
bool programacion = false;
bool menu = false;
bool recepcion = false;
bool tiempo = false;
bool seleccion_tiempo = false;
bool seleccion_sensor = false;

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
  // Modo de programacion
  if (programacion){
    // Condicion para mostrar el menu de los sensores una vez
    if (!seleccion_sensor && !seleccion_tiempo){
      seleccion_sensor = true;
    }
  // Un vez mostrado el menu de los sensores se revisa el buffer del serial  
  // para verificar si se selecciono alguna opcion
    if (seleccion_sensor){
      // Funcion para revisar el buffer del serial
      if (Serial.available() > 0) {
        // Se lee el dato recibido
        char receivedChar = Serial.read();
        // Comparacion del caracter recibido en el buffer del serial
        if (receivedChar == 'a'){
          LoRa.beginPacket();
          LoRa.print('a');
          LoRa.endPacket();
          seleccion_sensor = false;
          seleccion_tiempo = true;
        }
        if (receivedChar == 'b'){
          LoRa.beginPacket();
          LoRa.print('b');
          LoRa.endPacket();
          seleccion_sensor = false;
          seleccion_tiempo = true;
        }
        if (receivedChar == 'c'){
          LoRa.beginPacket();
          LoRa.print('c');
          LoRa.endPacket();
          seleccion_sensor = false;
          seleccion_tiempo = true;
        }
        if (receivedChar == 'd'){
          LoRa.beginPacket();
          LoRa.print('d');
          LoRa.endPacket();
          seleccion_sensor = false;
          seleccion_tiempo = true;
        }
        if (receivedChar == 'e'){
          LoRa.beginPacket();
          LoRa.print('e');
          LoRa.endPacket();
          seleccion_sensor = false;
          seleccion_tiempo = true;
        }
        if (receivedChar == 'f'){
          LoRa.beginPacket();
          LoRa.print('f');
          LoRa.endPacket();
          seleccion_sensor = false;
          seleccion_tiempo = true;
        }
        if (receivedChar == 'g'){
          LoRa.beginPacket();
          LoRa.print('g');
          LoRa.endPacket();
          seleccion_sensor = false;
          seleccion_tiempo = true;
        }
        if (receivedChar == 's'){
          seleccion_sensor = false;
          seleccion_tiempo = false;
          programacion = false;
          // Borra el display
          display.clearDisplay(); 
          display.display();
        }
      }
    } else if (seleccion_tiempo){
      if (Serial.available() > 0) {
        // Lee el dato recibido
        char receivedChar = Serial.read();
        // Comparacion del caracter recibido en el buffer del serial
        if (receivedChar == 'a'){
            LoRa.beginPacket();
            LoRa.print('a');
            LoRa.endPacket();
            seleccion_tiempo = false;
            programacion = false;
          }
          if (receivedChar == 'b'){
            LoRa.beginPacket();
            LoRa.print('b');
            LoRa.endPacket();
            seleccion_tiempo = false;
            programacion = false;
          }
          if (receivedChar == 'c'){
            LoRa.beginPacket();
            LoRa.print('c');
            LoRa.endPacket();
            seleccion_tiempo = false;
            programacion = false;
          }
          if (receivedChar == 's'){
            seleccion_tiempo = false;
            programacion = false;
            // Borra el display
            display.clearDisplay(); 
            display.display();
          }
      }
    }
  } 
  else if (!programacion){
    //Revision si se activo el modo programacion
    if (Serial.available() > 0) {
        // Lee el dato recibido
        char receivedChar = Serial.read();
        
        // Verifica el caracter recibido y lo compara con las opciones disponibles 
        if (receivedChar == 'q') {
          programacion = true;
        }
    }
  // Recibe los paquetes que mande el OBC y los despliega en el serial y la pantalla oled
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
//      Serial.println("Received Data From Satellite:\n");
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
