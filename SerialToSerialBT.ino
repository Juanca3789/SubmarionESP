#include "BluetoothSerial.h"
#include "Motor.h"
#include "PaP.h"
#include "TDSSensor.h"
#include <List.hpp>
#include <Ticker.h>
#include <Wire.h>
#define MPU_6050 0x68
#define pwmpin1 15
#define pwmpin2 17
#define pwmPin3 5
#define pwmPin4 18
#define pwmpin5 19
#define pwmpin6 23
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
bool registers[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int mot1Channel = 0;
int mot2Channel = 1;
int mot3Channel = 2;
int mot4Channel = 3;
int mot5Channel = 4;
int mot6Channel = 5;
Motor motor1(registers[0], registers[1], pwmpin1, mot1Channel);
Motor motor2(registers[2], registers[3], pwmpin2, mot2Channel);
Motor motor3(registers[4], registers[5], pwmPin3, mot3Channel);
Motor motor4(registers[6], registers[7], pwmPin4, mot4Channel);
Motor motor5(registers[8], registers[9], pwmpin5, mot5Channel);
Motor motor6(registers[10], registers[11], pwmpin6, mot6Channel);
PaP motorPaP(PaP1, PaP2, PaP3, PaP4, 0.5);
List<object> objetosAlrededor;
Cola colaEnvio = Cola();
Ticker enviarDatos, leerTDS, leerUltrasonico;
TDSSensor sensorTds(TDSPin, 3.3, 12);
float gyro_Z, gyro_X, gyro_Y, temperature, gyro_X_cal, gyro_Y_cal, gyro_Z_cal;
int gx, gy, gz, cal_int;
float acc_total_vector, ax, ay, az;
bool set_gyro_angles, accCalibOK  = false;
float acc_X_cal, acc_Y_cal, acc_Z_cal, angulo_pitch_acc, angulo_roll_acc, angulo_pitch, angulo_roll;
unsigned long int pastMicros = 0, tiempo_ejecucion = 0;
int velocidad;

void writeRegisters();
void loopPaP(void *parametros);
String radSender(float angulo, float distancia);
void funcionEnvio();
void funcionTDS();
void funcionUltrasonico();
/*
void MPU6050Init();
void MPU6050Leer();
void MPU6050Procesar();
*/


void setup() {
  Wire.begin();
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
  motor5.begin();
  motor6.begin();
  motorPaP.begin();
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  xTaskCreatePinnedToCore(loopPaP, "Movimiento Motor", 1000, NULL, tskIDLE_PRIORITY, &GirarMotorPaP, 0);
  enviarDatos.attach(0.2, funcionEnvio);
  leerTDS.attach(3, funcionTDS);
  leerUltrasonico.attach(1.5, funcionUltrasonico);
  /*
  MPU6050Init();
  for (cal_int = 0; cal_int < 3000 ; cal_int ++) {
    MPU6050Leer();   // Leer sensor MPU6050
    gyro_X_cal += gx;
    gyro_Y_cal += gy;
    gyro_Z_cal += gz;
    acc_X_cal  += ax;
    acc_Y_cal  += ay;
    acc_Z_cal  += az;
    delayMicroseconds(50);
  }
  gyro_X_cal = gyro_X_cal / 3000;
  gyro_Y_cal = gyro_Y_cal / 3000;
  gyro_Z_cal = gyro_Z_cal / 3000;
  acc_X_cal  = acc_X_cal  / 3000;
  acc_Y_cal  = acc_Y_cal  / 3000;
  acc_Z_cal  = acc_Z_cal  / 3000;
  accCalibOK = true;
  */
}

void loop() {
  //Codigo Recepción
  if(SerialBT.available()){
      char rec = SerialBT.read();
      Serial.print(rec);
      if(rec == 'V'){
        String val = "";
        for(int i = 0; i < 4; i++){
          char a = SerialBT.read();
          val += a;
        }
        Serial.print(val + "->");
        velocidad = val.toInt();
        Serial.print(velocidad);
      }
      else if(rec == 'U'){
        motor1.setDireccion(1, registers[0], registers[1]);
        motor2.setDireccion(1, registers[2], registers[3]);
        motor3.setDireccion(1, registers[4], registers[5]);
        motor4.setDireccion(1, registers[6], registers[7]);
        writeRegisters();
        motor1.mover(velocidad);
        motor2.mover(velocidad);
        motor3.mover(velocidad);
        motor4.mover(velocidad);
      }
      else if(rec == 'D') {
        motor1.setDireccion(2, registers[0], registers[1]);
        motor2.setDireccion(2, registers[2], registers[3]);
        motor3.setDireccion(2, registers[4], registers[5]);
        motor4.setDireccion(2, registers[6], registers[7]);
        writeRegisters();
        motor1.mover(velocidad);
        motor2.mover(velocidad);
        motor3.mover(velocidad);
        motor4.mover(velocidad);
      }
      else if (rec == '-') {
        motor1.setDireccion(1, registers[0], registers[1]);
        motor2.setDireccion(2, registers[2], registers[3]);
        motor3.setDireccion(1, registers[4], registers[5]);
        motor4.setDireccion(2, registers[6], registers[7]);
        writeRegisters();
        motor1.mover(velocidad);
        motor2.mover((velocidad * 50) / 100);
        motor3.mover(velocidad);
        motor4.mover((velocidad * 50) / 100);
      }
      else if (rec == '+') {
        motor1.setDireccion(2, registers[0], registers[1]);
        motor2.setDireccion(1, registers[2], registers[3]);
        motor3.setDireccion(2, registers[4], registers[5]);
        motor4.setDireccion(1, registers[6], registers[7]);
        writeRegisters();
        motor1.mover((velocidad * 50) / 100);
        motor2.mover(velocidad);
        motor3.mover((velocidad * 50) / 100);
        motor4.mover(velocidad);
      }
      else if (rec == 'L') {
        motor1.setDireccion(2, registers[0], registers[1]);
        motor2.setDireccion(1, registers[2], registers[3]);
        motor3.setDireccion(2, registers[4], registers[5]);
        motor4.setDireccion(1, registers[6], registers[7]);
        writeRegisters();
        motor1.mover(velocidad);
        motor2.mover((velocidad * 50) / 100);
        motor3.mover((velocidad * 50) / 100);
        motor4.mover(velocidad);
      }
      else if (rec == 'R') {
        motor1.setDireccion(2, registers[0], registers[1]);
        motor2.setDireccion(1, registers[2], registers[3]);
        motor3.setDireccion(2, registers[4], registers[5]);
        motor4.setDireccion(1, registers[6], registers[7]);
        writeRegisters();
        motor1.mover((velocidad * 50) / 100);
        motor2.mover(velocidad);
        motor3.mover(velocidad);
        motor4.mover((velocidad * 50) / 100);
      }
      else if (rec == 'A') {
        motor5.setDireccion(1, registers[8], registers[9]);
        motor6.setDireccion(1, registers[10], registers[11]);
        writeRegisters();
        motor5.mover(velocidad);
        motor6.mover(velocidad);
      }
      else if (rec == 'S') {
        motor5.setDireccion(2, registers[8], registers[9]);
        motor6.setDireccion(2, registers[10], registers[11]);
        writeRegisters();
        motor5.mover(velocidad);
        motor6.mover(velocidad);
      }
      else if(rec == 'F'){
        for(
          int i = motor1.getVelocidad(), j = motor2.getVelocidad(), k = motor3.getVelocidad(), l = motor4.getVelocidad(), m = motor5.getVelocidad(), o = motor6.getVelocidad();
          i >= 0 || j >= 0 || k >= 0 || l >= 0 || m >= 0 || o >= 0;
          i--, j--, k--, l--, m--, o--
        ) {
          if (i >= 0) motor1.mover(i);
          if (j >= 0) motor2.mover(j);
          if (k >= 0) motor3.mover(k);
          if (l >= 0) motor4.mover(l);
          if (m >= 0) motor5.mover(m);
          if (o >= 0) motor6.mover(o);
          delayMicroseconds(8);
        }
      }
      /*
      if(actualMicros - pastMicros > 1000000){
        pastMicros = actualMicros;
        tiempo_ejecucion = (actualMicros - pastMicros) / 1000;
        MPU6050Leer();
        MPU6050Procesar();
      }
      */
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
      for(posicionRadar = 370;posicionRadar >= 0; posicionRadar--){
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
  for(int i = 15; i >=  0; i--){
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

void funcionUltrasonico() {
    digitalWrite(TrigPin, LOW);  //
    delayMicroseconds(2);
    digitalWrite(TrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(TrigPin, LOW);
    int duration = pulseIn(EchoPin, HIGH);
    int distance = duration * 0.034 / 2;
    int angulo = posicionRadar % 360;
    String toEnv = radSender(angulo, distance);
    if(toEnv != ""){
      colaEnvio.add(toEnv);
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

/*
void MPU6050Init() {
  Wire.beginTransmission(MPU_6050);
  Wire.write(0x6B);                          // PWR_MGMT_1 registro 6B hex
  Wire.write(0x00);                          // 00000000 para activar
  Wire.endTransmission();
  Wire.beginTransmission(MPU_6050);
  Wire.write(0x1B);                          // GYRO_CONFIG registro 1B hex
  Wire.write(0x08);                          // 00001000: 500dps
  Wire.endTransmission();
  Wire.beginTransmission(MPU_6050);
  Wire.write(0x1C);                          // ACCEL_CONFIG registro 1C hex
  Wire.write(0x10);                          // 00010000: +/- 8g
  Wire.endTransmission();

  Wire.beginTransmission(MPU_6050);
  Wire.write(0x1B);
  Wire.endTransmission();
  Wire.requestFrom(MPU_6050, 1);
  while (Wire.available() < 1);

  // Activar y configurar filtro pasa bajos LPF que incorpora el sensor
  Wire.beginTransmission(MPU_6050);
  Wire.write(0x1A);
  Wire.write(0x04);
  Wire.endTransmission();
}

void MPU6050Leer() {
  Wire.beginTransmission(MPU_6050);       // Empezamos comunicación
  Wire.write(0x3B);                             // Pedir el registro 0x3B (AcX)
  Wire.endTransmission();
  Wire.requestFrom(MPU_6050, 14);         // Solicitar un total de 14 registros
  while (Wire.available() < 14);                // Esperamos hasta recibir los 14 bytes

  ax = Wire.read() << 8 | Wire.read();          // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  ay = Wire.read() << 8 | Wire.read();          // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az = Wire.read() << 8 | Wire.read();          // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gx = Wire.read() << 8 | Wire.read();          // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy = Wire.read() << 8 | Wire.read();          // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz = Wire.read() << 8 | Wire.read();          // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void MPU6050Procesar() {

  // Restar valores de calibración del acelerómetro
  ax -= acc_X_cal;
  ay -= acc_Y_cal;
  az -= acc_Z_cal;
  az  = az + 4096;

  // Restar valores de calibración del giroscopio y calcular
  // velocidad angular en º/s. Leer 65.5 en raw equivale a 1º/s
  gyro_X = (gx - gyro_X_cal) / 65.5;
  gyro_Y = (gy - gyro_Y_cal) / 65.5;
  gyro_Z = (gz - gyro_Z_cal) / 65.5;

  // Calcular ángulo de inclinación con datos del giroscopio
  // 0.000000266 = tiempo_ejecucion / 1000 / 65.5 * PI / 180
  angulo_pitch += gyro_X * tiempo_ejecucion / 1000;
  angulo_roll  += gyro_Y * tiempo_ejecucion / 1000;
  angulo_pitch += angulo_roll  * sin((gz - gyro_Z_cal) * tiempo_ejecucion  * 0.000000266);
  angulo_roll  -= angulo_pitch * sin((gz - gyro_Z_cal) * tiempo_ejecucion  * 0.000000266);

  // Calcular vector de aceleración
  // 57.2958 = Conversion de radianes a grados 180/PI
  acc_total_vector  = sqrt(pow(ay, 2) + pow(ax, 2) + pow(az, 2));
  angulo_pitch_acc  = asin((float)ay / acc_total_vector) * 57.2958;
  angulo_roll_acc   = asin((float)ax / acc_total_vector) * -57.2958;

  // Filtro complementario
  if (set_gyro_angles) {
    angulo_pitch = angulo_pitch * 0.99 + angulo_pitch_acc * 0.01;
    angulo_roll  = angulo_roll  * 0.99 + angulo_roll_acc  * 0.01;
  }
  else {
    angulo_pitch = angulo_pitch_acc;
    angulo_roll  = angulo_roll_acc;
    set_gyro_angles = true;
  }
}
*/