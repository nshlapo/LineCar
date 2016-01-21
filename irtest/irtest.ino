//Adding Libraries for motor shield
#include <Wire.h>  //include Motor Liraries
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMS1 = Adafruit_MotorShield();  //create motor shield object
Adafruit_MotorShield AFMS2 = Adafruit_MotorShield();  //create motor shield object
Adafruit_DCMotor *MotorR = AFMS1.getMotor(1); //set up motor in port 1
Adafruit_DCMotor *MotorL = AFMS2.getMotor(2); //set up motor 2

//Define Variables
int setpoint_value = 0; //find true value
int sensorPinR = A0; //the input pin for right IR sensor
int sensorPinL = A1;  //input pin for left IR sesnor
long sensorValueR = 0; //initializing sensor value
long sensorValueL =0; //initializing sensor value
int speedR;  //initalizing speed variables
int speedL;
int defaultSpeed = 40;  //speed that the robot will converge to
double kP = .15; // proportinal value for PID control
double kI = 0;   // integral value for PID control, not actually used in our functioning model
double kD = .0001; //dampening value for PID control
int ref = 0;  //temp variable for calculating error
int sensorDiff = 0; //difference between the two sensor values

//correctional values for PID
int corrDiff = 0;
int control = 0;
int corrVal = 0;
int corrStore = 0;
int corri = 0;
int corrf = 0;

boolean moving = true;  //only keeps robot running if this is true

byte input = 40;  // serial user input variables
byte reading = 40;


void setup() {
  Serial.begin(250000);  //initalize serial monitor
  AFMS1.begin();   // begin motors
  AFMS2.begin();
  MotorR->setSpeed(defaultSpeed);  //set both motors too default speeds
  MotorL->setSpeed(defaultSpeed);
  MotorR->run(FORWARD);  //start running motors
  MotorL->run(FORWARD);
  delay(5000);  //so matlab can get its shit together

}


void loop() {
  reading = Serial.parseInt();  //make sure user input is one number (50) rather than 5 and then 0

  if (moving){  //if robot is moving
    defaultSpeed = input;  //set default speed to whatever the user input is

    //initialize the variables we're linked to
    sensorValueR = analogRead(sensorPinR);  //set sensorValue to current value of analog pin
    sensorValueL = analogRead(sensorPinL);  //set sensorValue to current value of analog pin
    sensorDiff = sensorValueR - sensorValueL;  //get difference between 2 sensors for PID
    control = ref - sensorDiff;  //for integral - not actually used in final model
    corri = control;
    corrDiff = corrf - corri;
    corrStore += (control);

    //make correctional value determined by proportinal difference in sensor value (kP)
    //then dampen this (kD) so that the robot doesn't go cray
    corrVal = (control)*kP + corrStore*kI + corrDiff*kD;

    speedR = defaultSpeed - corrVal;  //change speeds of motors based on control PID algorithm
    speedL = defaultSpeed + corrVal;

    if (speedR < 0)  //making sure motors don't loop and go from 0 to 244 when it gets a negative
    {
      speedR =0;
    }

    if (speedL < 0)
    {
      speedL =0;
    }

    MotorR->setSpeed(speedR  //reset motor speeds
    MotorL->setSpeed(speedL);
    corrf = corri;
    delay(5);  //wait 1 second before printing next value


    //print a bunch of values to send to matlab for plotting
    Serial.print(sensorValueR);
    Serial.print(", ");
    Serial.print(sensorValueL);
    Serial.print(", ");
    Serial.print(speedR);
    Serial.print(", ");
    Serial.print(speedL);
    Serial.print(", ");
    Serial.println(reading);

    if (sensorValueR <=300 && sensorValueL <= 250)  //if both sensors are on the tape, stop running
    {
      moving = false;   //once its not running, send a bunch of 0s to matlab so that it knows to stop
      Serial.print(0);
      Serial.print(", ");
      Serial.print(0);
      Serial.print(", ");
      Serial.print(0);
      Serial.print(", ");
      Serial.print(0);
      Serial.print(", ");
      Serial.println(0);
    }

    //user input things
        if (reading > 0)  // if there is a user input (reading) that is greater than 0
    {
      input = reading;  // change the input to the value of the reading
    }

  }

    delay(5);  //wait 1 second before printing next value

}
