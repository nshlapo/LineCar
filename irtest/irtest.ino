int sensorPin = A0; //the input pin for our IR sensor
long sensorValue = 0; //initializing sensor value

void setup() {
  Serial.begin(115200);  //initalize serial monitor
}


void loop() {
  sensorValue = analogRead(sensorPin);  //set sensorValue to current value of analog pin
  Serial.println(sensorValue);  //print sensor value
  delay(1500);  //wait 1.5 second before printing next value
} 