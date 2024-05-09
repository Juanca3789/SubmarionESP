#ifndef MOTOR_H
  #define MOTOR_H
#endif
#include <Arduino.h>

class Motor{
  private:
    bool register1, register2;
    int direccion = 0, velocidad = 0;
    int pwmPin, pwmChannel;
    int frequency = 5000;
    int resolution = 12;
  public:
    Motor(bool register1, bool register2, int pwmPin, int pwmChannel);
    void begin();
    void mover(int _velocidad);
    void setFrequency(int frequency);
    void setResolution(int resolution);
    void setDireccion(int direccion, bool &_register1, bool &_register2);
    int getVelocidad();
};