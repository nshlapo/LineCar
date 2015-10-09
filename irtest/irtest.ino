#include <PID_v1.h>
#include <Wire.h>  //include Motor Liraries
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMS1 = Adafruit_MotorShield();  //create motor shield object
Adafruit_MotorShield AFMS2 = Adafruit_MotorShield();  //create motor shield object
Adafruit_DCMotor *MotorR = AFMS1.getMotor(1); //set up motor in port 1
Adafruit_DCMotor *MotorL = AFMS2.getMotor(2); //set up motor 2

//Define Variables we'll be connecting to
double Setpoint1, Input1, Output1, Setpoint2, Input2, Output2;
int kp = 1;
int ki = 0;
int kd = 1;

//Specify the links and initial tuning parameters
PID PID1(&Input1, &Output1, &Setpoint1, kp, ki, kd, DIRECT);
PID PID2(&Input2, &Output2, &Setpoint2, kp, ki, kd, DIRECT);

int setpoint_value = 0; //find true value
int sensorPinR = A0; //the input pin for our IR sensor
int sensorPinL = A1;
long sensorValueR = 0; //initializing sensor value
long sensorValueL =0; //initializing sensor value
int speedR = 50;
int speedL = 50;
int default_speed = 50;
int calibration_variable = 1;
int sensor_switch_point = 600;
int turning_speed = 60;  //fast wheel during turn
int low_speed = 0;      //slow wheel during turn


void setup() {
  Serial.begin(115200);  //initalize serial monitor
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
//  Input1 = analogRead(sensorPinR);
//  Input2 = analogRead(sensorPinL);
//  Setpoint1 = setpoint_value;
//  Setpoint2 = setpoint_value;
//  //turn the PID on
//  PID1.SetMode(AUTOMATIC);
//  PID2.SetMode(AUTOMATIC);
  AFMS1.begin();
  AFMS2.begin();
  MotorR->setSpeed(default_speed);
  MotorL->setSpeed(default_speed);
  MotorR->run(FORWARD);
  MotorL->run(FORWARD);
}


void loop() {
    //initialize the variables we're linked to
  sensorValueR = analogRead(sensorPinR);  //set sensorValue to current value of analog pin
  sensorValueL = analogRead(sensorPinL);  //set sensorValue to current value of analog pin
  //Serial.print(sensorValueR);
  //Serial.print(", ");
  //Serial.println(sensorValueL);  //print sensor value
  //Serial.print("Difference");
  //Serial.println(sensorValueR-sensorValueL);
  
    if (sensorValueR-sensorValueL >= 350)  // if its is on the ground the value is low, on the tape it is high
    {
      MotorR->setSpeed(turning_speed);
      MotorL->setSpeed(low_speed);
      //Serial.println("Left Turn");
      digitalWrite(12, HIGH);   // turn the LED on (HIGH is the voltage level)
      digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)

    }
    else if (sensorValueL-sensorValueR >= 250) 
    {
      MotorL->setSpeed(turning_speed);
      MotorR->setSpeed(low_speed);
      digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
      digitalWrite(12, LOW);   // turn the LED on (HIGH is the voltage level)
      //Serial.println("Right Turn");
    }  
    else
    {
      MotorR->setSpeed(default_speed);
      MotorL->setSpeed(default_speed);
      //Serial.println("No Turn");
      //digitalWrite(12, LOW);   // turn the LED on (HIGH is the voltage level)
      //digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)

    }
  
//    if (sensorValueR >=  sensor_switch_point)
//    {
//      Serial.print("Turn right motor, Speed: ");
//      speedR -= (600-sensorValueR)*calibration_variable;
//      MotorR->setSpeed(speedR);
//      Serial.println(speedR);
//    }
//    else
//    {
//      MotorR->setSpeed(default_speed);
//      Serial.println("No turn right motor, Speed: 50");
//
//    }
//    
//    if (sensorValueL >= sensor_switch_point)
//    {
//      Serial.print("Turn left motor, Speed: ");
//      speedL -= (600-sensorValueL)*calibration_variable;
//      MotorL->setSpeed(speedL);
//      Serial.println(speedL)    ;
//    }
//    else
//    {
//      MotorL->setSpeed(default_speed);
//      Serial.println("No turn left motor, Speed: 50");
//
//    }    
//    Input1 = sensorValueR;
//    Input2 = sensorValueL;
//    PID1.Compute();
//    PID2.Compute();
//    
//   if(Output1 !=0)
//    {
//      speed1 -= Output1*calibration_variable;
//      MotorR->setSpeed(speed1);
////      Serial.println("1 - turning");
//      digitalWrite(12, LOW);   // turn the LED on (HIGH is the voltage level)
//    }
//    else
//  {
//    MotorR->setSpeed(default_speed);
//    digitalWrite(12, HIGH);   // turn the LED on (HIGH is the voltage level)
////      Serial.println("1 - not turning");
//
//  }
////  
//   if(Output2 !=0)
//    {
//      speed2 -= Output2*calibration_variable;
//      MotorL->setSpeed(speed2);
////      Serial.println("2 - turning");
//      digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)
//    }
//    else
//  {
//    MotorL->setSpeed(speed2);
////      Serial.println("2 - not turning");
//      digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
//  }
    
//    Serial.print(Output1);
//    Serial.print(", ");
//    Serial.println(Output2);
    delay(5);  //wait 1 second before printing next value

} 
