#include <Wire.h>  //include Motor Liraries
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMS1 = Adafruit_MotorShield();  //create motor shield object
Adafruit_MotorShield AFMS2 = Adafruit_MotorShield();  //create motor shield object
Adafruit_DCMotor *MotorR = AFMS1.getMotor(1); //set up motor in port 1
Adafruit_DCMotor *MotorL = AFMS2.getMotor(2); //set up motor 2

//Define Variables
int setpoint_value = 0; //find true value
int sensorPinR = A0; //the input pin for our IR sensor
int sensorPinL = A1;
long sensorValueR = 0; //initializing sensor value
long sensorValueL =0; //initializing sensor value
int speedR;
int speedL;
int defaultSpeed = 50;
double kP = 1;
int ref = 0;

void setup() {
    Serial.begin(115200);  //initalize serial monitor
    pinMode(12,OUTPUT);
    pinMode(13,OUTPUT);
    AFMS1.begin();
    AFMS2.begin();
    MotorR->setSpeed(defaultSpeed);
    MotorL->setSpeed(defaultSpeed);
    MotorR->run(FORWARD);
    MotorL->run(BACKWARD);
}


void loop() {
    sensorValueR = analogRead(sensorPinR);  //set sensorValue to current value of analog pin
    sensorValueL = analogRead(sensorPinL);  //set sensorValue to current value of analog pin
    sensorDiff = sensorValueR - sensorValueL;

    corrVal = (ref - sensorDiff)*kP;
    Serial.println(corrVal);
    speedR = defaultSpeed - corrVal;
    speedL = defaultSpeed + corrVal;

    MotorR->setSpeed(speedR);
    MotorL->setSpeed(speedL);
    delay(5);  //wait 1 second before printing next value

}
