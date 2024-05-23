#ifndef RADAR_H
  #define RADAR_H
#endif

#include <Arduino.h>
#include "PaP.h"
#include <List.hpp>
#include "BluetoothSerial.h"
class Radar{
  private:
    class object {
      public:
        float angle;
        float distance;
        object() {};
        object(float _angle, float _distance);
    };
    PaP *motorRadar;
    int trigPin;
    int echoPin;
        BluetoothSerial *BTSerialObject;
  public:
    Radar(int motA, int motB, int motC, int motD, int trig, int echo, BluetoothSerial &BT);
    void begin();
    void run();
    String radSender(float angulo, float distancia);
    ~Radar();
};