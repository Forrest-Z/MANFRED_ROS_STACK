

int IRPin = A0;    // select the input pin for the IR

int sensorValue = 0;  // variable to store the value coming from the sensor

void setup() {

   Serial.begin(9600);
}

void loop() {
  // read the value from the sensor:
  sensorValue = analogRead(IRPin);    
  // turn the ledPin on
  Serial.println(sensorValue);
  delay(100);  
}
