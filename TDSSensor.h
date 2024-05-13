#ifndef TDSSENSOR_H
  #define TDSSENSOR_H
#endif

#include <Arduino.h>
#define tdsFactor 0.5

class TDSSensor {
  private:
    int pin;
    float aref;
    int resolution;
    float temperature = 25.0;
  public:
    TDSSensor(int pin, float aref, int resolution);
    ~TDSSensor();
    float takeMedition();
    void setTemperature(float temperature);
    void begin();
};