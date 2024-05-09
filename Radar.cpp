#include "esp32-hal-gpio.h"
#include "Radar.h"

Radar::Radar(int motA, int motB, int motC, int motD, int trig, int echo, BluetoothSerial &BT) {
  this->motorRadar = new PaP(motA, motB, motC, motD, 0.5);
  this->trigPin = trig;
  this->echoPin = echo;
  this->BTSerialObject = &BT;
}

Radar::~Radar() {
  delete motorRadar;
}

void Radar::begin() {
  this->motorRadar->begin();
  pinMode(this->trigPin, OUTPUT);
  pinMode(this->echoPin, INPUT);
}

String Radar::radSender(float angulo, float distancia) {
  String baseString = "";
  for(int i = 0; i < this->objetosAlrededor.getSize(); i++){
    object inList = this->objetosAlrededor.get(i);
    if(inList.angle == angulo){
      this->objetosAlrededor.remove(i);
      if(distancia < 1 || distancia > 180){
          baseString = "RD=";
      }
      else{
        baseString = "RU=";
        this->objetosAlrededor.add(object(angulo, distancia));
      }
      return (baseString + String(angulo) + "," + String(distancia));
    }
  }
  if(distancia > 1 && distancia < 180) {
    baseString = "RA=";
    this->objetosAlrededor.add(object(angulo, distancia));
    return (baseString + String(angulo) + "," + String(distancia));
  }
  return "";
}

void Radar::run() {
  for(int i = 0; i < 4076; i++){
    this->motorRadar->darPaso();
    digitalWrite(trigPin, LOW);  //
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    int duration = pulseIn(echoPin, HIGH);
    int distance = duration * 0.034 / 2;
    int angulo = i % 360;
    if(this->radSender(angulo, distance) != ""){
      this->BTSerialObject->println(this->radSender(angulo, distance));
    }
  }
  this->motorRadar->cambiarDireccion();
}