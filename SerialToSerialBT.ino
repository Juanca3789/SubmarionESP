#include "BluetoothSerial.h"
#include "Motor.h"
#include "PaP.h"
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
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
  #error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

TaskHandle_t GirarMotorPaP;
BluetoothSerial SerialBT;
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
unsigned long int previousMillis = 0;

void writeRegisters();
void loopPaP(void *parametros);

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
  xTaskCreatePinnedToCore(loopPaP, "Movimiento Motor", 1000, NULL, tskIDLE_PRIORITY, &GirarMotorPaP, 0);
}

void loop() {
  String env = "";
  /*
  unsigned long int actualMillis = millis();
  if(actualMillis - previousMillis > 1000) {
    previousMillis = actualMillis;
    Serial.println("En nucleo " + String(xPortGetCoreID()));
  }
  */
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
    /*
    Serial.println("En nucleo " + String(xPortGetCoreID()));
    delay(500);
    */
    for(int i = 0; i < 4076; i++){
      motorPaP.darPaso();
    }
    motorPaP.cambiarDireccion();
  }
  vTaskDelay(10);
}


//Functions
void writeRegisters(){/* function writeRegisters */ 
//// Write register after being set 
digitalWrite(RclkPin, LOW);
 for(int i = 7; i >=  0; i--){
  digitalWrite(SrclkPin, LOW); 
  digitalWrite(SerPin, registers[i]);
  digitalWrite(SrclkPin, HIGH);
}
  digitalWrite(RclkPin, HIGH);
}

