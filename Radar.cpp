#include "esp32-hal-gpio.h"
#include "Radar.h"

void Radar::run() {
  for(int i = 0; i < 371; i++){
    for(int j = 0; j < 12; j ++){
      this->motorRadar->darPaso();
    }
    digitalWrite(trigPin, LOW);  //
    delayMicroseconds(1);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(trigPin, LOW);
    int duration = pulseIn(echoPin, HIGH);
    int distance = duration * 0.034 / 2;
    int angulo = i % 360;
    
  }
  this->motorRadar->cambiarDireccion();
}