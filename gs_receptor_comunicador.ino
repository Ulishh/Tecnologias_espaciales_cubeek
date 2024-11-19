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

  // Mostrar ground station al inicio
  // Solo usar con fin demostrativo, para obtener los datos al lanzar el satelite en ocasiones se congela
  // debido a la poca memoria de procesamiento de la arduino nano, por eso al hacer pruebas comentamos estas lineas
  display.clearDisplay();         // Limpiar la pantalla
  display.setTextSize(2);         // Establecer el tamaño del texto
  display.setTextColor(WHITE);    // Establecer color del texto
  display.setCursor(0, 0);      // Posicionar el cursor en el centro
  display.println("Ground");               // Imprimir el texto
  display.print("Station");
  display.display();              // Mostrar en la pantalla
  delay(5000);
}


void loop() {
  // Modo de programacion
  if (programacion){
    // Condicion para habilitar la seleccion del sensor
    if (!seleccion_sensor && !seleccion_tiempo){
      seleccion_sensor = true;
    }
  // Condicional para mandar por lora la seleccion del sensor
    if (seleccion_sensor){
      // Funcion para revisar el buffer del serial
      if (Serial.available() > 0) {
        // Se lee el dato recibido
        char receivedChar = Serial.read();
        // Comparacion del caracter recibido en el buffer del serial
        // GPS
        if (receivedChar == 'a'){
          LoRa.beginPacket();
          LoRa.print('a');
          LoRa.endPacket();
          seleccion_sensor = false;
          seleccion_tiempo = true;
        }
        // IMU
        if (receivedChar == 'b'){
          LoRa.beginPacket();
          LoRa.print('b');
          LoRa.endPacket();
          seleccion_sensor = false;
          seleccion_tiempo = true;
        }
        // Presion
        if (receivedChar == 'c'){
          LoRa.beginPacket();
          LoRa.print('c');
          LoRa.endPacket();
          seleccion_sensor = false;
          seleccion_tiempo = true;
        }
        // Luz
        if (receivedChar == 'd'){
          LoRa.beginPacket();
          LoRa.print('d');
          LoRa.endPacket();
          seleccion_sensor = false;
          seleccion_tiempo = true;
        }
        // Temperatura
        if (receivedChar == 'e'){
          LoRa.beginPacket();
          LoRa.print('e');
          LoRa.endPacket();
          seleccion_sensor = false;
          seleccion_tiempo = true;
        }
        // Voltaje
        if (receivedChar == 'f'){
          LoRa.beginPacket();
          LoRa.print('f');
          LoRa.endPacket();
          seleccion_sensor = false;
          seleccion_tiempo = true;
        }
        // Corriente
        if (receivedChar == 'g'){
          LoRa.beginPacket();
          LoRa.print('g');
          LoRa.endPacket();
          seleccion_sensor = false;
          seleccion_tiempo = true;
        }
        // Cancelar
        if (receivedChar == 's'){
          seleccion_sensor = false;
          seleccion_tiempo = false;
          programacion = false;
          // Borra el display
          display.clearDisplay(); 
          display.display();
        }
      }
    // Condicional para mandar el tiempo seleccionado por lora 
    } else if (seleccion_tiempo){
      // Se revisa el buffer del serial
      if (Serial.available() > 0) {
        // Lee el dato recibido
        char receivedChar = Serial.read();
        // Comparacion del caracter recibido en el buffer del serial
        // 20 segundos
        if (receivedChar == 'a'){
            LoRa.beginPacket();
            LoRa.print('a');
            LoRa.endPacket();
            seleccion_tiempo = false;
            programacion = false;
          }
        // 40 segundos
          if (receivedChar == 'b'){
            LoRa.beginPacket();
            LoRa.print('b');
            LoRa.endPacket();
            seleccion_tiempo = false;
            programacion = false;
          }
        // 1 minuto
          if (receivedChar == 'c'){
            LoRa.beginPacket();
            LoRa.print('c');
            LoRa.endPacket();
            seleccion_tiempo = false;
            programacion = false;
          }
        // Cancelar
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
  // Condicional si el modo de programacion no esta activo
  // Siempre esta en escucha en espera de comandos
  else if (!programacion){
    //Se revisa el buffer del serial
    if (Serial.available() > 0) {
        // Lee el dato recibido
        char receivedChar = Serial.read();
        
        // Verifica el caracter recibido y lo compara con las opciones disponibles 
        //Revision si se activo el modo programacion
        if (receivedChar == 'q') {
          programacion = true;
        }
    }
  // Recibe los paquetes que mande el OBC por lora y los despliega en el serial y la pantalla oled
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      char receivedData[maxPacketSize];
      int bytesRead = LoRa.readBytes(receivedData, maxPacketSize);
      receivedData[bytesRead] = '\0';
      // Se imprimen los datos recibidos en el serial
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
