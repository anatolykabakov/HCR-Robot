//Использование выводов
/*
===PWM==============================================================
Моторы
4 ---> правый мотор PWM1 (+ земля)
5 ---> левый мотор PWM2


===DIGITAL==========================================================
Моторы
52 ---> Direction-пин правого мотора DIR1
53 ---> Direction-пин левого мотора DIR2



===ANALOG==========================================================

нет
*/
#include <Kalman.h>
#include <Metro.h>
#include <PID_v1.h>
#include <Math.h>
#include <TimerOne.h> // http://www.arduino.cc/playground/Code/Timer1
#include <Wire.h>
#include <VL53L0X.h>
#include <LiquidCrystal.h>
// #include <Check_times.cpp>

#include <TroykaIMU.h>
#include <Wire.h>

// MegaADK DIGITAL PINS USABLE FOR INTERRUPTS 2, 3, 18, 19, 20, 21
//                                                 I2C pins 20, 21

//Encoder variables

const byte encoderRpinA = 2;                              //A pin -> the interrupt pin (2)
const byte encoderRpinB = 17;                              //B pin -> the digital pin (16)
const byte encoderLpinA = 3;                              //A pin -> the interrupt pin (3)
const byte encoderLpinB = 16;                              //B pin -> the digital pin (17)


byte encoderRPinALast;
byte encoderLPinALast;
double wheelSpeedR = 0;  // Скорость правого колеса с энкодера
double wheelSpeedL = 0;  // Скорость левого колеса с энкодера
unsigned long wheelImpR = 0; // число импульсов с энкодера правого колеса 
unsigned long wheelImpL = 0; // число импульсов с энкодера левого колеса 

//PID variables
double Motor_2[3]={0,3,0.005};                //PID parameters [P,I,D]
double Setpoint1,Input1,Output1;                   //PID input&output values for Motor1
double Setpoint2,Input2,Output2;                   //PID input&output values for Motor2

PID myPID1(&Input1,&Output1,&Setpoint1,Motor_2[0],Motor_2[1],Motor_2[2],DIRECT);  
PID myPID2(&Input2,&Output2,&Setpoint2,Motor_2[0],Motor_2[1],Motor_2[2],DIRECT);

// Timer variables
const long Timer1Interval=100000;                                // 100 ms = 10 times per sec - Timer interrupt interval

//Motor control variables
const int MotorRdir = 52;    //Right motor Direction Control pin
const int MotorLdir = 53;    //Left motor Direction Control pin
const int MotorRpwm = 4;     //Right motor PWM Speed Control pin
const int MotorLpwm = 5;     //Left motor PWM Speed Control pin
double SetSpeedR = 0;   //Wish Speed of right motor
double SetSpeedL = 0;   //Wish Speed of left motor
bool DirectionR = 0;     //Right Motor Direction
bool DirectionL = 0;     //Left Motor Direction

unsigned long Time_delay;
//-------------------------События------------------------------------------------------
bool settingSpeed = false; // команда установки скорости

//------------------------------------------------
#include <string.h>
char buffer[8];
double left, right;
double readV, readW;
double wheelLeftS = 0;
double wheelRightS = 0;
double wheelLeftV = 0;
double wheelRightV = 0;
double omegaRight = 0;
double omegaLeft = 0;
double R = 0.0682;
double L = 0.135;
double V = 0;
double omega = 0;
double Vl = 0;
double Vr = 0;
double SetV = 0;
double SetW = 0;
double maxSpeed = 0.6848; // максимальная линейная скорость при скважности 100%, в м/с

double yaw = 0;
double x = 0;
double y = 0;

void setup() {
   Init();
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Главный цикл ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
void loop() { 
  // --------------- Чтение порта --------------------
  ReadPort();
  // --------------- Смена уставки скорости ----------
  Motor();
  // -------------------------------------------------
  delay(10);
 }
 //loop ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Motor(){
  CheckSettingsSpeed();
  PIDMovement(SetSpeedR,SetSpeedL);  // передается линейная скорость колес, в м/сек  
}
void SetSpeed(double left, double right){ 
  SetSpeedR = (((2*left)+(right*L))/(2*R))*R;   //M/S - линейная скорость колеса
  SetSpeedL = (((2*left)-(right*L))/(2*R))*R;
}
void CheckSettingsSpeed(){
  if (settingSpeed){
      settingSpeed = false;
      SetSpeed(left, right);
    }
}
// --------------- Чтение порта --------------------
void ReadPort(){
  while (Serial.available() > 0) {  // чтение строки из последовательного порта
      //---------------------------
      String line = Serial.readStringUntil('\n');// считываем скорости для левого и правого колеса [40 50]
      line.toCharArray(buffer,10);//переводим в char
      left        = atof(strtok(buffer," "));//разделяем на скорости левого и правого колеса
      right       = atof(strtok(NULL,  " "));
      if (line.length() > 0){settingSpeed = true;}
      if(left == 90){reset_xy();}
      if(left == 80){Print();}
      
      //---------------------------
    }
}
void reset_xy(){
  x = 0;
  y = 0;
  V = 0;
  yaw = 0;
}
void Init(){
  Wire.begin();
  Serial.begin(9600);//Initialize the Serial port
  while (!Serial) ; // while the Serial stream is not open, do nothing
  MotorsInit();
  EncoderInit();//Initialize encoder
  PIDInit();  
}

void PIDInit(){
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(-255,255);
  myPID2.SetOutputLimits(-255,255);
}
void MotorsInit() { //Initialize motors variables 
  DirectionR = LOW;
  DirectionL = LOW;
  SetSpeedR = 0;
  SetSpeedL = 0;
  
  pinMode(MotorRdir, OUTPUT);
  pinMode(MotorLdir, OUTPUT);
  pinMode(MotorRpwm, OUTPUT);
  pinMode(MotorLpwm, OUTPUT);
  
  digitalWrite (MotorRdir, DirectionR);
  digitalWrite (MotorLdir, DirectionL);
  analogWrite (MotorRpwm, SetSpeedR);
  analogWrite (MotorLpwm, SetSpeedL);
}
void EncoderInit() { //Initialize encoder interruption 

  pinMode(encoderRpinA,INPUT);  // Right weel
  pinMode(encoderRpinB,INPUT);  
  pinMode(encoderLpinA,INPUT);  // Left weel
  pinMode(encoderLpinB,INPUT);  
 
  // Привязка прерывания по импульсу энкодера
  attachInterrupt(digitalPinToInterrupt(encoderRpinA), WheelPulseR, RISING ); // вызов процедуры по прерыванию. Параметры: номер прерывания (не ножки), имя процедуры, состояние сигнала
  attachInterrupt(digitalPinToInterrupt(encoderLpinA), WheelPulseL, RISING );  // ЗАМЕНА, была ссылка на DecodeSpeedL
  
  // Настройка таймера
  Timer1.initialize(Timer1Interval);
  Timer1.attachInterrupt(Timer_finish);
 
}
void WheelPulseR(){   // Счетчик спиц правого колеса 
  wheelImpR ++;
}
void WheelPulseL(){   // Счетчик спиц левого колеса 
  wheelImpL ++;
}
void Timer_finish()  {

  wheelSpeedR = double(wheelImpR * 1000000 / Timer1Interval); // число импульсов за сек
  wheelSpeedL = double(wheelImpL * 1000000 / Timer1Interval); // число импульсов за сек

  // пройденный колесом путь
  wheelRightS = ((wheelSpeedR / 663) * 2 * 3.14 *  R) ; // метры L = 2*PI*R*n/N 
  wheelLeftS  = ((wheelSpeedL / 663) * 2 * 3.14 *  R); //* 
  
  // линейная скорость колеса
  wheelRightV = wheelRightS/ 1; // mетры за сек
  wheelLeftV  = wheelLeftS / 1;

  // угловая скорость колеса
  omegaRight = wheelRightV/R;   // rad за сек
  omegaLeft  = wheelLeftV/R;
  
  // фактическая линейная скорость центра робота
  V     = (R/2)*(omegaRight + omegaLeft);//m/s
  // фактическая угловая скорость поворота робота
  omega = (R/L)*(omegaRight - omegaLeft);

  yaw+=(omega*0.1);
  x += V*cos(yaw)*0.1;
  y += V*sin(yaw)*0.1;

  // проверка
  Vr = (((2*V)+(omega*L))/(2*R))*R;//M/S
  Vl = (((2*V)-(omega*L))/(2*R))*R;
  
  
  wheelImpR = 0;
  wheelImpL = 0;
  
  //Print();

}
void Print() {    // ==================== Вывод данных в порт

    Serial.print (V); Serial.print ("; ");
    Serial.print (yaw*3.14/180); Serial.print ("; ");
    //Serial.print (omega); Serial.print ("; ");
    Serial.print (x); Serial.print ("; ");
    Serial.print (y); Serial.print ("; ");
    Serial.print(millis());
    Serial.println();
 }
void Movement(int a,int b){//move
  analogWrite (MotorRpwm,a);      //motor1 move forward at speed a
  digitalWrite(MotorRdir,DirectionR);  
  analogWrite (MotorLpwm,b);      //motor2 move forward at speed b
  digitalWrite(MotorLdir,DirectionL);  
}

//PID modules
void PIDMovement(double a,double b){
  // a, b - m/sec
  
  Setpoint1= int(a * 255 /maxSpeed); // уставка скорости
  Setpoint2= int(b * 255 /maxSpeed);
 
  Input1= wheelRightV * 255 / maxSpeed;           // обратная связь ПИД-регулятора, м/сек
  Input2= wheelLeftV * 255 / maxSpeed;

  myPID1.Compute();
  myPID2.Compute();

  Movement (int (Output1), int(Output2));
}

  
      

