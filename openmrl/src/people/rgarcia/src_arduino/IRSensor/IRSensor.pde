
int a = 2; // LSB multiplexer
int b = 3; //
int c = 4; // MSB multiplexer

int readPinY = A0; 

//  int IRPin = A0;    // select the input pin for the IR

int sensorValue = 0;  // variable to store the value coming from the sensor

void setup() {
   pinMode(a, OUTPUT);
   pinMode(b, OUTPUT);
   pinMode(c, OUTPUT);
   Serial.begin(9600);
   
   digitalWrite(a, LOW);
   digitalWrite(b, HIGH);
   digitalWrite(c, HIGH);
   
   
}

void loop() {
  // read the value from the sensor:
  sensorValue = analogRead(readPinY);    
  // turn the ledPin on
  Serial.println(sensorValue);
  delay(100);  
}
