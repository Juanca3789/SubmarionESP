#include "esp32-hal.h"
#include "Motor.h"

Motor::Motor(bool register1, bool register2, int pwmPin, int pwmChannel){
  this->register1 = register1;
  this->register2 = register2;
  this->pwmPin = pwmPin;
  this->pwmChannel = pwmChannel;
}

void Motor::begin(){
  ledcSetup(this->pwmChannel, this->frequency, this->resolution);
  ledcAttachPin(this->pwmPin, this->pwmChannel);
}

void Motor::mover(int _velocidad){
  this->velocidad = _velocidad;
  ledcWrite(this->pwmChannel, this->velocidad);
}

void Motor::setFrequency(int frequency){
  this->frequency = frequency;
}

void Motor::setResolution(int resolution){
  this->resolution = resolution;
}

void Motor::setDireccion(int _direccion, bool &_register1, bool &_register2){
  this->direccion = _direccion;
  if(_direccion == 0){
    _register1 = 0;
    _register2 = 0;
    this->register1 = _register1;
    this->register2 = _register2;
  }
  else if(_direccion == 1){
    _register1 = 1;
    _register2 = 0;
    this->register1 = _register1;
    this->register2 = _register2;
  }
  else if(_direccion == 2){
    _register1 = 0;
    _register2 = 1;
    this->register1 = _register1;
    this->register2 = _register2;
  }
}

int Motor::getVelocidad() {
  return this->velocidad;
}