
#include <PID_v1.h> //biblioteka obslugująca PID
#include <NewPing.h>  // Biblioteka obslugująca czujnik Ultradźwiękowy
#include <Servo.h>  // biblioteka do obsługi serwa
#include "I2Cdev.h" // biblioteka do obslugi I2C
#include "MPU6050_6Axis_MotionApps20.h" // biblioteka do obslugi MPU6050
#include "Wire.h" // Biblioteka do obslugi wejść i wy
#define runEvery(t) for (static typeof(t) _lasttime;(typeof(t))((typeof(t))millis() - _lasttime) > (t);_lasttime += (t)) //definicja funkcji która wykonuje sie co określony czas t
#define TRIGGER_PIN  9  //ustawienie pinu TRIGGER
#define ECHO_PIN     10  //ustawienie pinu ECHO
#define MAX_DISTANCE 200 //maksymalny dystans
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13
Servo myservo;                                        // definicja serwa
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);   //definicja czujnika Ultradźwiękowego
int uS;                  
int distance;            //dystans
int pos = 90;            //pozycja startowa serwa
int kierunek_serwa = 0;  
int robot_kierunek = 0;  
int poprzedni_robot_kierunek=0;  //last direction of the robot
int dystans_przod;    
int dystans_lewo;
int dystans_prawo;
int servoDelay = 20;
long previousMillis = 0;
const int interval = 650; 
int ir;
int przyrost=50;
unsigned long currentMillis;
bool blinkState = false;
// Obsługa modułu MPU6050
bool dmpReady = false;  
uint8_t mpuIntStatus;   // status byte z MPU
uint8_t devStatus;      
uint16_t packetSize;   
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; // FIFO buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw ;
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     
void dmpDataReady() {
    mpuInterrupt = true;
}
//sterowanie silnikami
unsigned long czas;
const int EN1 = 3;       
const int kierunek1 = 8; 
const int EN2 = 5;       
const int kierunek2 = 4; 
const int kierunek3 = 7;
const int kierunek4 = 12;
int pr_lewo;
int pr_prawo;
int spl, spr;
int output_minus;
boolean obrot_lewo;
boolean obrot_prawo;
//zmienne PID
double Setpoint, Input, Output, lastSetpoint;
double Kp=6, Ki=1/10, Kd=0.04;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); //deklaracja PID
int recive_byte=0;


void setup() {
    //obsluga MPU6050
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; 
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200); // ustawienie prędkośći portu szeregowego
    while (!Serial); 

    mpu.initialize(); // inicjalizacja MPU
    devStatus = mpu.dmpInitialize();

    //Offset zyroskopu po kalibracji
    mpu.setXGyroOffset(79);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(14);
    mpu.setZAccelOffset(1463); 

    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    pinMode(LED_PIN, OUTPUT);
  
    //definicja wyjść
    pinMode(EN1, OUTPUT); 
    pinMode(kierunek1, OUTPUT);
    pinMode(EN2, OUTPUT);
    pinMode(kierunek2, OUTPUT);
    pinMode(kierunek3, OUTPUT);
    pinMode(kierunek4, OUTPUT);
    analogWrite(EN1, 0);
    analogWrite(EN2, 0);
    myservo.attach(6); 
    myservo.write(pos); //centrowanie serwa
         //Ustawienie zmiennych   
    pr_lewo=80;
    pr_prawo=80;
    robot_kierunek=0;
    poprzedni_robot_kierunek=0;
    Input = ypr[0]*180/M_PI;
    Setpoint = 0;
    myPID.SetMode(AUTOMATIC);
    ir=1;
    
    delay(3000); //opoznienie potrzebne do inicjalizacji systemu
}
void loop (){

  czas= millis();
  gyro();
/* wypisywanie parametrów używane do sprawd
Serial.print(czas);
Serial.print("\t");
Serial.print(Setpoint);
Serial.print("\t");
Serial.print(ypr[0]*180/M_PI);
Serial.print("\t");
Serial.print(Input);
Serial.print("\t");
Serial.print(Output);
Serial.print("\t");
Serial.print(robotDirection);
Serial.print(przebieg);
Serial.print("\t");
Serial.print(InputR);
Serial.print("\t");
Serial.println(OutputR);
*/
   if(Serial.available()==0 ){
    obrotserwo(); 
    pobierz_odleglosc();  
    sterowanie();
      }
  
 
   if(Serial.available()>0 ){
      recive_byte=Serial.read();
      }
switch(recive_byte){
  case 1: obrotserwo(); pobierz_odleglosc();  sterowanie(); break;
  case 3: Setpoint=0; jedz_prosto(); break;
  case 2: stop(); break;
  case 4: Setpoint=-45; jedz_prosto(); break;
  case 5: Setpoint=45; jedz_prosto(); break;
  case 6: Setpoint=-90; jedz_prosto(); break;
  case 7: Setpoint=90; jedz_prosto(); break;
  case 8: Setpoint=-135; jedz_prosto(); break;
  case 9: Setpoint=180; jedz_prosto(); break;
  case 10: Setpoint=135; jedz_prosto(); break;
  default: stop(); break;

}
}
void jedz_prosto(){
  if (Setpoint >= 180){
    Setpoint= Setpoint-360;
    }
  if (Setpoint <=-180){
    Setpoint= Setpoint+360;
    }
  if(Setpoint+przyrost > 180){
    lastSetpoint=Setpoint;
    }
  if(Setpoint+przyrost < -180){
    lastSetpoint=Setpoint;
    }
    
  gyro();
  
  if (ypr[0]*180/M_PI < Setpoint){
    myPID.SetControllerDirection(DIRECT);
    }
  else {
    myPID.SetControllerDirection(REVERSE);
    }
  Input =ypr[0]*180/M_PI;
  myPID.Compute();
  
  if (ypr[0]*180/M_PI<Setpoint && Setpoint<0 && lastSetpoint+przyrost>180 ){
    spl= pr_lewo+Output;
    spr= pr_prawo-Output; 
    if(ypr[0]*180/M_PI<Setpoint+5 &&ypr[0]*180/M_PI>Setpoint-5) lastSetpoint=Setpoint;
    }
  else if (ypr[0]*180/M_PI>=Setpoint && Setpoint<0 && lastSetpoint+przyrost>180){
    spl= pr_lewo+Output;
    spr= pr_prawo-Output; 
    if(ypr[0]*180/M_PI<Setpoint+5 &&ypr[0]*180/M_PI>Setpoint-5) lastSetpoint=Setpoint;
    }
  else if (ypr[0]*180/M_PI>=Setpoint && Setpoint>0 && lastSetpoint+przyrost<-180 ){
    spl= pr_lewo-Output;
    spr= pr_prawo+Output; 
    if(ypr[0]*180/M_PI<Setpoint+5 &&ypr[0]*180/M_PI>Setpoint-5) lastSetpoint=Setpoint;
    }
  else if (ypr[0]*180/M_PI<Setpoint && Setpoint>0 && lastSetpoint+przyrost<-180){
    spl= pr_lewo-Output;
    spr= pr_prawo+Output; 
    if(ypr[0]*180/M_PI<Setpoint+5 &&ypr[0]*180/M_PI>Setpoint-5) lastSetpoint=Setpoint;
    }
  else if (ypr[0]*180/M_PI < Setpoint){
      spl= pr_lewo+Output;
      spr= pr_prawo-Output;
      }
  else if (ypr[0]*180/M_PI >= Setpoint ){
      spl=pr_lewo-Output;
      spr=pr_prawo+Output;
      }
  if( spr <0 ) spr=0;
  if( spl <0) spl=0;
  if( spr >255 ) spr=255;
  if( spl >255) spl=255;
  //ustawienie kierunków obrotów silnika
  digitalWrite(8, LOW); 
  digitalWrite(7, HIGH);
  digitalWrite(12, LOW); 
  digitalWrite(4, HIGH);
  //zapis prędkośći 
  analogWrite(3, spl);
  analogWrite(5, spr);
}
void stop(){           
  digitalWrite(8, HIGH); 
  digitalWrite(7, LOW);
  analogWrite(3, 0);
  digitalWrite(12, HIGH); 
  digitalWrite(4, LOW);
  analogWrite(5, 0);
}
void backward(){          
  digitalWrite(8, HIGH); //Silnik nr 1 - obroty w lewo
  digitalWrite(7, LOW);
  digitalWrite(12, HIGH); //Silnik nr 1 - obroty w lewo
  digitalWrite(4, LOW);
  for(int i=0;i < pr_lewo; i++){
  analogWrite(3, i);
  analogWrite(5, i); 
  }
}


void gyro() {
  if (!dmpReady) return;
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    } 
  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    #endif
    }
}

void pobierz_odleglosc(){
  runEvery(40){
    uS = sonar.ping();
    distance = uS / US_ROUNDTRIP_CM;
    if (uS == NO_ECHO){
      distance = MAX_DISTANCE;     
      }
    }
}

void obrotserwo(){
  runEvery(servoDelay){
    if(pos < 165 && kierunek_serwa == 0) {                                 
      pos = pos + 5;                        
      }
    if(pos > 15 && kierunek_serwa == 1){                               
      pos = pos - 5;
      }   
    }  
  if (pos == 165 ){
    kierunek_serwa = 1;  
    }
  if (pos == 15 ){
    kierunek_serwa = 0;  
    }  
  myservo.write(pos);  
}

void sterowanie(){
  ir = digitalRead(14);
  currentMillis = millis();  
  if (pos >= 0 && pos <= 45){
    dystans_prawo = distance;  
    }
  if (pos >= 135 && pos <= 180){
    dystans_lewo = distance; 
    }  
  if (pos > 70 && pos < 110){
    dystans_przod = distance; 
    }
  if (ir == 0){
    robot_kierunek = 1;
    }
  else{
    if (dystans_przod >= 50 ){
      robot_kierunek = 0;
      }
    else {
      if (dystans_lewo > dystans_prawo || dystans_prawo <= 25){
        robot_kierunek = 2; 
        }
      if (dystans_lewo < dystans_prawo || dystans_lewo <= 25){
        robot_kierunek = 3;
        }
      if (dystans_lewo <= 15 && dystans_przod <= 15 || dystans_prawo <= 15 && dystans_przod <= 15){
        robot_kierunek = 1; 
        }
      }
    }
  if(robot_kierunek == 0 && robot_kierunek == poprzedni_robot_kierunek){
    jedz_prosto();
    poprzedni_robot_kierunek = robot_kierunek;
    }
  if(robot_kierunek == 0 && robot_kierunek != poprzedni_robot_kierunek && currentMillis - previousMillis > interval ){ 
    jedz_prosto();
    poprzedni_robot_kierunek = robot_kierunek;
    previousMillis = currentMillis;
    }
  if(robot_kierunek == 1 && robot_kierunek == poprzedni_robot_kierunek){
    backward();
    poprzedni_robot_kierunek = robot_kierunek;
    }
  if(robot_kierunek == 1 && robot_kierunek != poprzedni_robot_kierunek && currentMillis - previousMillis > interval ){ 
    backward();
    poprzedni_robot_kierunek = robot_kierunek;
    previousMillis = currentMillis;
    }
  if(robot_kierunek == 2 && robot_kierunek == poprzedni_robot_kierunek){
    poprzedni_robot_kierunek = robot_kierunek;
    }
  if(robot_kierunek == 2 && robot_kierunek != poprzedni_robot_kierunek && currentMillis - previousMillis > interval ){ 
    Setpoint=ypr[0]*180/M_PI-50;
    jedz_prosto();
    poprzedni_robot_kierunek = robot_kierunek;
    previousMillis = currentMillis;
    }
  if(robot_kierunek == 3 && robot_kierunek == poprzedni_robot_kierunek){
    poprzedni_robot_kierunek = robot_kierunek;
    }
  if(robot_kierunek == 3 && robot_kierunek != poprzedni_robot_kierunek && currentMillis - previousMillis > interval ){ 
    Setpoint=ypr[0]*180/M_PI+50;
    jedz_prosto();
    poprzedni_robot_kierunek = robot_kierunek;
    previousMillis = currentMillis;
    }
}





















