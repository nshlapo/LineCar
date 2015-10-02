int sensorPin1 = A0; //the input pin for our IR sensor
int sensorPin2 = A1;
long sensorValue1 = 0; //initializing sensor value
long sensorValue2 =0; //initializing sensor value


void setup() {
  Serial.begin(115200);  //initalize serial monitor
}


void loop() {
  sensorValue1 = analogRead(sensorPin1);  //set sensorValue to current value of analog pin
  sensorValue2 = analogRead(sensorPin2);  //set sensorValue to current value of analog pin
  Serial.print(sensorValue2);
  Serial.print(", ");
  Serial.println(sensorValue1);  //print sensor value
  delay(1000);  //wait 1.5 second before printing next value
  
  if (sensorValue1 >=  600)
{
  Serial.println("Move left motor");
}
  if (sensorValue2 >= 600)
  {
    Serial.println("Move right motor");
  }
} 
