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
double Motor_2[3]={0.1,3,0};                //PID parameters [P,I,D]
double Setpoint1,Input1,Output1;                   //PID input&output values for Motor1
double Setpoint2,Input2,Output2;                   //PID input&output values for Motor2

PID myPID1(&Input1,&Output1,&Setpoint1,Motor_2[0],Motor_2[1],Motor_2[2],DIRECT);  
PID myPID2(&Input2,&Output2,&Setpoint2,Motor_2[0],Motor_2[1],Motor_2[2],DIRECT);

// Timer variables
const long Timer1Interval=100000;                                // 100 ms = 10 times per sec - Timer interrupt interval
double dT = double(Timer1Interval)/1000000;           // период счета 

//Motor control variables
const int MotorRdir = 52;    //Right motor Direction Control pin
const int MotorLdir = 53;    //Left motor Direction Control pin
const int MotorRpwm = 4;     //Right motor PWM Speed Control pin
const int MotorLpwm = 5;     //Left motor PWM Speed Control pin
double SetSpeedR = 0;   //Wish Speed of right motor
double SetSpeedL = 0;   //Wish Speed of left motor
bool DirectionR = 0;     //Right Motor Direction
bool DirectionL = 0;     //Left Motor Direction

//-------------------------События------------------------------------------------------
bool settingSpeed = false; // команда установки скорости

//------------------------------------------------
#include <string.h>
char buffer[8];
double LinearVelocity, AngularVelocity;
double readV, readW;
double wheelLeftS = 0;
double wheelRightS = 0;
double wheelLeftV = 0;
double wheelRightV = 0;
double omegaRight = 0;
double omegaLeft = 0;
double R = 0.0682;
double L = 0.275;
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

bool printflag = false;
bool is_connected = false;

void setup() {
   Init();
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Главный цикл ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
void loop() { 
  // --------------- Чтение порта --------------------
  get_messages_from_Serial1();
  // --------------- Смена уставки скорости ----------
  Motor();
  // -------------------------------------------------
 }
 //loop ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Motor(){
  CheckSettingsSpeed();
  PIDMovement(SetSpeedR,SetSpeedL);  // передается линейная скорость колес, в м/сек  
}
void SetSpeed(double LinearVelocity, double AngularVelocity){ 
  SetSpeedR = (((2*LinearVelocity)+(AngularVelocity*L))/(2*R))*R;   //M/S - линейная скорость колеса
  SetSpeedL = (((2*LinearVelocity)-(AngularVelocity*L))/(2*R))*R;
}
void CheckSettingsSpeed(){
  if (settingSpeed){
      settingSpeed = false;
      SetSpeed(LinearVelocity, AngularVelocity);
    }
}
// --------------- Чтение порта --------------------

void reset_var(){
  x = 0;
  y = 0;
  yaw = 0;
  V = 0;
  SetSpeedR = 0;   //M/S - линейная скорость колеса
  SetSpeedL = 0;
}
void Init(){
  Wire.begin();
  Serial1.begin(57600);//Initialize the Serial1 port
  while (!Serial1) ; // while the Serial1 stream is not open, do nothing
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

  wheelSpeedR = double(wheelImpR / dT); // число импульсов за сек
  wheelSpeedL = double(wheelImpL / dT); // число импульсов за сек

  // пройденный колесом путь, м
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

  yaw+=(omega * dT);    // направление в рад
  x += V*cos(yaw) * dT; // в метрах
  y += V*sin(yaw) * dT;

  // проверка
  Vr = (((2*V)+(omega*L))/(2*R))*R; //M/S
  Vl = (((2*V)-(omega*L))/(2*R))*R;
  
  wheelImpR = 0;
  wheelImpL = 0;


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

    if (a < 0) {a = abs(a); DirectionR = true;}
    else {DirectionR = false;}

    if (b < 0) {b = abs(b); DirectionL = true;}
    else {DirectionL = false;}

  Setpoint1= (a * 255 /maxSpeed); // уставка скорости
  Setpoint2= (b * 255 /maxSpeed);
 
  Input1= wheelRightV * 255 / maxSpeed;           // обратная связь ПИД-регулятора, м/сек
  Input2= wheelLeftV * 255 / maxSpeed;

  myPID1.Compute();
  myPID2.Compute();

  Movement (int (Output1), int(Output2));
}

void get_messages_from_Serial1()
{
  if(Serial1.available() > 0)
  {
    // The first byte received is the instruction
    int order_received = Serial1.read();

    if(order_received == 's')
    {
      // If the cards haven't say hello, check the connection
      if(!is_connected)
      {
        is_connected = true;
        Serial1.print("r");
      }
    }
    else
    {
      switch(order_received)
      {

        case 'v'://если v, то считываем уставку по скорости
        {

          String line = Serial1.readStringUntil('\n');// считываем скорости для левого и правого колеса [40 50]
          line.toCharArray(buffer,10);//переводим в char
          LinearVelocity        = atof(strtok(buffer," "));//разделяем на скорости левого и правого колеса
          AngularVelocity       = atof(strtok(NULL,  " "));
          settingSpeed = true;

          break;
        }
        case 'd'://если d, то печатаем текущие значения
        {
          if(V>=0){Serial1.print ("+");Serial1.print (V); Serial1.print (";");}
          else {Serial1.print (V); Serial1.print (";");}
          if(yaw>=0){Serial1.print ("+");Serial1.print (yaw); Serial1.print (";");}
          else {Serial1.print (yaw); Serial1.print (";");}
          if(x>=0){Serial1.print ("+");Serial1.print (x); Serial1.print (";");}
          else {Serial1.print (x); Serial1.print (";");}
          if(y>=0){Serial1.print ("+");Serial1.print (y); Serial1.print (";");}
          else {Serial1.print (y); Serial1.print (";");}
          
          break;
        }
        // Unknown order
        default:

          return;
      }
    }
    Serial1.print("c");
 
  }
}
      


