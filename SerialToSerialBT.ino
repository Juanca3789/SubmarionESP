#include "BluetoothSerial.h"
#include "Motor.h"
#include "PaP.h"
#include "TDSSensor.h"
#include <List.hpp>
#include <Ticker.h>
#define pwmpin1 15
#define pwmpin2 17
#define pwmPin3 5
#define pwmPin4 18
#define SerPin 2
#define RclkPin 4
#define SrclkPin 16
#define PaP1 13
#define PaP2 12
#define PaP3 14
#define PaP4 27
#define TrigPin 26
#define EchoPin 25
#define TDSPin 36
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
  #error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

class object {
  public:
    float angle;
    float distance;
    object() {};
    object(float _angle, float _distance){
      this->angle = _angle;
      this->distance = _distance;
    }
};

class Cola{
  private:
    String colastring[1000];
    int frente = 0;
    int final = -1;
  public:
    Cola() {};
    bool estaLlena();
    bool estaVacia();
    void add(String elemento);
    String remove();
};

bool Cola::estaVacia() {
  return frente > final;
}

bool Cola::estaLlena() {
  return final == 999;
}

void Cola::add(String elemento) {
  if(!estaLlena())
    colastring[++final] = elemento;
}

String Cola::remove() {
  if(!estaVacia()){
    String elemento  = colastring[0];
    for(int i = 1; i < final; i++){
      colastring[i - 1] = colastring[i];
    }
    final--;
    return elemento;
  }
  else{
    return "";
  }
}

TaskHandle_t GirarMotorPaP;
BluetoothSerial SerialBT;
int posicionRadar = 0;
bool registers[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int mot1Channel = 0;
int mot2Channel = 1;
int mot3Channel = 2;
int mot4Channel = 3;
Motor motor1(registers[0], registers[1], pwmpin1, mot1Channel);
Motor motor2(registers[2], registers[3], pwmpin2, mot2Channel);
Motor motor3(registers[4], registers[5], pwmPin3, mot3Channel);
Motor motor4(registers[6], registers[7], pwmPin4, mot4Channel);
PaP motorPaP(PaP1, PaP2, PaP3, PaP4, 0.5);
List<object> objetosAlrededor;
Cola colaEnvio = Cola();
Ticker enviarDatos, leerTDS;
TDSSensor sensorTds(TDSPin, 3.3, 12);

void writeRegisters();
void loopPaP(void *parametros);
String radSender(float angulo, float distancia);
void funcionEnvio();
void funcionTDS();

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32-BT-Slave"); //Bluetooth device name
  pinMode(SerPin, OUTPUT);
  pinMode(RclkPin, OUTPUT);
  pinMode(SrclkPin, OUTPUT);
  writeRegisters();
  motor1.begin();
  motor2.begin();
  motor3.begin();
  motor4.begin();
  motorPaP.begin();
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  xTaskCreatePinnedToCore(loopPaP, "Movimiento Motor", 1000, NULL, tskIDLE_PRIORITY, &GirarMotorPaP, 0);
  enviarDatos.attach(0.2, funcionEnvio);
  leerTDS.attach(3, funcionTDS);
}

void loop() {
  String env = "";
  if(env != ""){
    for(char a : env){
      SerialBT.write(a);
    }
    SerialBT.write(10);
  }
  //Codigo Recepción
  if(SerialBT.available()){
      char rec = SerialBT.read();
      if(rec == 'U'){
        for(
          int i = motor1.getVelocidad(), j = motor2.getVelocidad(), k = motor3.getVelocidad(), l = motor4.getVelocidad();
          i >= 0 || j >= 0 || k >= 0 || l >= 0;
          i--, j--, k--, l--
        ) {
          if (i >= 0) motor1.mover(i);
          if (j >= 0) motor2.mover(j);
          if (k >= 0) motor3.mover(k);
          if (l >= 0) motor4.mover(l);
          delayMicroseconds(8);
        }
        motor1.setDireccion(1, registers[0], registers[1]);
        motor2.setDireccion(1, registers[2], registers[3]);
        motor3.setDireccion(1, registers[4], registers[5]);
        motor4.setDireccion(1, registers[6], registers[7]);
        writeRegisters();
        motor1.mover(4095);
        motor2.mover(4095);
        motor3.mover(4095);
        motor4.mover(4095);
      }
      else if(rec == 'D') {
        for(
          int i = motor1.getVelocidad(), j = motor2.getVelocidad(), k = motor3.getVelocidad(), l = motor4.getVelocidad();
          i >= 0 || j >= 0 || k >= 0 || l >= 0;
          i--, j--, k--, l--
        ) {
          if (i >= 0) motor1.mover(i);
          if (j >= 0) motor2.mover(j);
          if (k >= 0) motor3.mover(k);
          if (l >= 0) motor4.mover(l);
          delayMicroseconds(10);
        }
        motor1.setDireccion(2, registers[0], registers[1]);
        motor2.setDireccion(2, registers[2], registers[3]);
        motor3.setDireccion(2, registers[4], registers[5]);
        motor4.setDireccion(2, registers[6], registers[7]);
        writeRegisters();
        motor1.mover(4095);
        motor2.mover(4095);
        motor3.mover(4095);
        motor4.mover(4095);
      }
      else if(rec == 'F') {
        for(
          int i = motor1.getVelocidad(), j = motor2.getVelocidad(), k = motor3.getVelocidad(), l = motor4.getVelocidad();
          i >= 0 || j >= 0 || k >= 0 || l >= 0;
          i--, j--, k--, l--
        ) {
          if (i >= 0) motor1.mover(i);
          if (j >= 0) motor2.mover(j);
          if (k >= 0) motor3.mover(k);
          if (l >= 0) motor4.mover(l);
          delayMicroseconds(15);
        }      
      }
  }
}

void loopPaP(void *parametros){
  while(true) {
    if(SerialBT.hasClient()){
      for(posicionRadar = 0; posicionRadar < 371; posicionRadar++){
        for(int j = 0; j < 12; j++){
          motorPaP.darPaso();
        }
      }
      motorPaP.cambiarDireccion();
    }
  }
  vTaskDelay(10);
}

//Functions
void writeRegisters(){
  digitalWrite(RclkPin, LOW);
  for(int i = 7; i >=  0; i--){
    digitalWrite(SrclkPin, LOW); 
    digitalWrite(SerPin, registers[i]);
    digitalWrite(SrclkPin, HIGH);
  }
  digitalWrite(RclkPin, HIGH);
}

void funcionEnvio(){
  if(SerialBT.hasClient()){
    if(!colaEnvio.estaVacia()){
      SerialBT.println(colaEnvio.remove());
    }
  }
}

void funcionTDS() {
  float val = 0;
  for(int i = 0; i < 50; i++){
    val += sensorTds.takeMedition();
  }
  val = val / 50;
  if(!colaEnvio.estaLlena()){
    colaEnvio.add("TDS=" + String(val));
  }
}

String radSender(float angulo, float distancia) {
  String baseString = "";
  for(int i = 0; i < objetosAlrededor.getSize(); i++){
    object inList = objetosAlrededor.get(i);
    if(inList.angle == angulo){
      objetosAlrededor.remove(i);
      if(distancia < 1 || distancia > 180){
          baseString = "RD=";
      }
      else{
        baseString = "RU=";
        objetosAlrededor.add(object(angulo, distancia));
      }
      return (baseString + String(angulo) + "," + String(distancia));
    }
  }
  if(distancia > 1 && distancia < 180) {
    baseString = "RA=";
    objetosAlrededor.add(object(angulo, distancia));
    return (baseString + String(angulo) + "," + String(distancia));
  }
  return "";
}

