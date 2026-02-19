#include "config.h"
#include <Wire.h>

/************************ Example Starts Here *******************************/
void procesarUART(String data);
void handleAngX(AdafruitIO_Data *data);
void handleAngY(AdafruitIO_Data *data);
void handleDisparo(AdafruitIO_Data *data);
void handleAutorizacion(AdafruitIO_Data *data);

// variables globales
int16_t angulo_x = 0; 
int16_t angulo_y = 0; 
uint16_t dist = 0; 
int disp = 0; 
int c = 0; 
int mot = 0; 

#define IO_LOOP_DELAY 4000
unsigned long lastUpdate = 0;

// configuración de feed de salida. 
AdafruitIO_Feed *anguloxFeed = io.feed("angulo x");
AdafruitIO_Feed *anguloyFeed = io.feed("angulo y");
AdafruitIO_Feed *autoFeed = io.feed("autorización");
AdafruitIO_Feed *disparoFeed = io.feed("disparo");
AdafruitIO_Feed *distanciaFeed = io.feed("distancia del Objetivo");
AdafruitIO_Feed *motorFeed = io.feed("motor");



// configuración de feed de entrada. 
AdafruitIO_Feed *angulox1Feed = io.feed("angulo x1");
AdafruitIO_Feed *anguloy1Feed = io.feed("angulo y1");
//
AdafruitIO_Feed *disparoRXFeed = io.feed("disparorx");

String bufferUART = "";

void setup() {

  // start the serial connection
  Serial.begin(115200);

  // wait for serial monitor to open
  while(! Serial);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  Serial.print("Connecting to Adafruit IO");
  io.connect();

  angulox1Feed->onMessage(handleMessageangulox1);
  anguloy1Feed->onMessage(handleMessageanguloy1);
  disparoRXFeed->onMessage(handleMessagedisparorx);
  autoRXFeed->onMessage(handleMessageautorizacionrx);

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());
  //counter->get();
  //get's
  angulox1Feed->get();
  anguloy1Feed->get();
  disparoRXFeed->get();
  //autoFeed->get();


}

void loop() {
  io.run();
  while (Serial2.available()) {
   char cIn = Serial2.read();

    if (cIn == '<') {
      bufferUART = "";
    }
    else if (cIn == '>') {
      procesarUART(bufferUART);
    }
    else {
      bufferUART += cIn;
    }
  }


  if (millis() > lastUpdate + IO_LOOP_DELAY) {
    
    //angulo_x = random(-180,180);
    //dist = random(2,20); 
    Serial.print("sending -> ");
    Serial.println(angulo_x);
    Serial.println(angulo_y); 
    Serial.println(dist);
    Serial.println(disp);
    Serial.println(c); 

    Serial.println("Actualizando feeds en Adafruit...");

    //counter->save(count);
    anguloxFeed->save(angulo_x); 
    anguloyFeed->save(angulo_y);
    distanciaFeed->save(dist);
    //disparoFeed->save(disp);
    autoFeed->save(c);
    motorFeed->save(mot); 

    lastUpdate = millis();
  }

}

void procesarUART(String data) {

  int ax = angulo_x;
  int ay = angulo_y;
  int distancia = dist;
  //int disparoInt = disp;
  int autoInt = c;
  int mot1 = mot;

  sscanf(data.c_str(), "%d,%d,%d,%d,%d",
         &ax, &ay, &distancia, &autoInt, &mot1);

  angulo_x = ax;
  angulo_y = ay;
  dist = distancia;
  //disp = disparoInt;
  c = autoInt;
  mot = mot1; 

  Serial.println("Datos recibidos del Nano:");
  Serial.println(data);
 
}

void handleMessageangulox1(AdafruitIO_Data *data) {

  //Serial.print("received <- ");
  //Serial.println(data->value());
  int nuevoAngX = data->toInt();

  Serial.print("Nuevo angulo X desde IO: ");
  Serial.println(nuevoAngX);

  // Mandar al Nano
  Serial2.print("<X,");
  Serial2.print(nuevoAngX);
  Serial2.println(">");

}

void handleMessageanguloy1(AdafruitIO_Data *data) {

  //Serial.print("received <- ");
  //Serial.println(data->value());
  int nuevoAngY = data->toInt();
  Serial2.print("<Y,");
  Serial2.print(nuevoAngY);
  Serial2.println(">");

}

void handleMessageautorizacionrx(AdafruitIO_Data *data) {
  //Serial.print("received <- ");
  //Serial.println(data->value());
  int valor = data->toInt();   // convierte a entero (0 o 1)

  Serial.print("Autorización recibida <- ");
  Serial.println(valor);

  // Enviar al Nano
  Serial2.print("<A,");
  Serial2.print(valor);
  Serial2.println(">");
}