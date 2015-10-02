#include <Wire.h>  //include Motor Liraries
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMS1 = Adafruit_MotorShield();  //create motor shield object
Adafruit_MotorShield AFMS2 = Adafruit_MotorShield();  //create motor shield object
Adafruit_DCMotor *Motor1 = AFMS1.getMotor(3); //set up motor in port 1
Adafruit_DCMotor *Motor2 = AFMS2.getMotor(4); //set up motor 2

int speed = 255;


void setup()
{
  AFMS1.begin();
  AFMS2.begin();
  Motor1->setSpeed(speed);
  Motor2->setSpeed(speed);
  Motor1->run(FORWARD);
  Motor2->run(FORWARD);
  Serial.begin(9600);
}

void loop()
{
  Serial.println('hey');
}
