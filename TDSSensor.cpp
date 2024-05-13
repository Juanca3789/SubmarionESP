#include "esp32-hal-adc.h"
#include "esp32-hal-gpio.h"
#include <cmath>
#include "TDSSensor.h"

TDSSensor::TDSSensor(int pin, float aref, int resolution) {
  this->pin = pin;
  this->aref = aref;
  this->resolution = pow(2, resolution);
}

TDSSensor::~TDSSensor(){}

void TDSSensor::setTemperature(float temperature) {
  this->temperature = temperature;
}

void TDSSensor::begin() {
  pinMode(this->pin, INPUT);
}

float TDSSensor::takeMedition() {
  float analogR = analogRead(this->pin);
  float voltage =  (analogR / float(this->resolution)) * this->aref;
  float ecValue = ((133.42 * pow(voltage, 3)) + (255.86 * pow(voltage, 2)) + (857.39 * voltage));
  float tempAdj = ecValue / (1.0 + (0.02 * (this->temperature - 25.0)));
  float tds = tempAdj * tdsFactor;
  return tds;
}