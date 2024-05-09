#ifndef PAP_H
  #define PAP_H
#endif
#include <Arduino.h>

class PaP{
  private:
    int pinA;
    int pinB;
    int pinC;
    int pinD;
    int velocidad;
    int *tablaPasos = nullptr;
    int direccion = 1;
    double fases;
    int cantidadPasos;
    int pasosActuales = 0;
  public:
    PaP(int pin1, int pin2, int pin3, int pin4, double fases);
    ~PaP();
    void begin();
    void cambiarDireccion();
    void cambiarFases(double fases);
    void setVelocidad(int velocidadP);
    void darPaso();
};