#include "esp32-hal.h"
#include "esp32-hal-gpio.h"
#include "PaP.h"

PaP::PaP(int pin1, int pin2, int pin3, int pin4, double fases) {
  this->pinA = pin1;
  this->pinB = pin2;
  this->pinC = pin3;
  this->pinD = pin4;
  this->cambiarFases(fases);
}

void PaP::cambiarDireccion() {
  this->direccion *= -1;
}

void PaP::cambiarFases(double fases) {
  if(fases >= 0.999999 && fases <= 1.000001) {
    this->velocidad = 2000;
    this->cantidadPasos = 4;
    this->tablaPasos = new int[this->cantidadPasos];
    this->tablaPasos[0] = B1000;
    this->tablaPasos[1] = B0100;
    this->tablaPasos[2] = B0010;
    this->tablaPasos[3] = B0001;
  }
  else if(fases >= 1.999999 && fases <= 2.000001) {
    this->velocidad = 2000;
    this->cantidadPasos = 4;
    this->tablaPasos = new int[this->cantidadPasos];
    this->tablaPasos[0] = B1001;
    this->tablaPasos[1] = B1100;
    this->tablaPasos[2] = B0110;
    this->tablaPasos[3] = B0011;
  }
  else{
    this->velocidad = 1000;
    this->cantidadPasos = 8;
    this->tablaPasos = new int[this->cantidadPasos];
    this->tablaPasos[0] = B1000;
    this->tablaPasos[1] = B1100;
    this->tablaPasos[2] = B0100;
    this->tablaPasos[3] = B0110;
    this->tablaPasos[4] = B0010;
    this->tablaPasos[5] = B0011;
    this->tablaPasos[6] = B0001;
    this->tablaPasos[7] = B1001;
  }
}

void PaP::begin() {
  pinMode(this->pinA, OUTPUT);
  pinMode(this->pinB, OUTPUT);
  pinMode(this->pinC, OUTPUT);
  pinMode(this->pinD, OUTPUT);
}

void PaP::darPaso() {
  this->pasosActuales += direccion;
  if(this->pasosActuales >= this->cantidadPasos) this->pasosActuales = 0;
  else if (this->pasosActuales < 0) this->pasosActuales = this->cantidadPasos - 1;
  int paso = this->pasosActuales;
  digitalWrite(this->pinA, bitRead(tablaPasos[paso], 0));
  digitalWrite(this->pinB, bitRead(tablaPasos[paso], 1));
  digitalWrite(this->pinC, bitRead(tablaPasos[paso], 2));
  digitalWrite(this->pinD, bitRead(tablaPasos[paso], 3));
  delayMicroseconds(this->velocidad);
}

void PaP::setVelocidad(int velocidadP) {
  if(velocidad >= 800){
    this->velocidad = velocidad;
  }
  else {
    this->velocidad = 800;
  }
}

PaP::~PaP() {
  if(this->tablaPasos != nullptr){
    delete []this->tablaPasos;
  }
}