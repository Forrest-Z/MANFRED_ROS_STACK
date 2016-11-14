char incomingByte;

int a = 2; // LSB multiplexer
int b = 3; //
int c = 4; // MSB multiplexer
int readPin = A0; // Select the output channel

long meanMeasure = 0;
int sensorValue = 0;  // variable to store the value coming from the sensor

void setup() {
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
  //Serial.println("Hi there! I'm Arduino");
   pinMode(a, OUTPUT);
   pinMode(b, OUTPUT);
   pinMode(c, OUTPUT);
   Serial.begin(9600);
   
   digitalWrite(a, LOW);
   digitalWrite(b, HIGH);
   digitalWrite(c, HIGH);
}

void loop() {
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    switch (incomingByte){
    case 'i': // Send robot information to the computer
      {
        Serial.println("MiniSkybot");
        break;
      }    
    case 'm': // Send measure of the selected sensor to the computer
      {
        meanMeasure = 0;
        for (int i = 0; i < 10; i++){
          sensorValue = analogRead(readPin);
          meanMeasure += sensorValue;
        }
      meanMeasure/= 10;  
      Serial.println(meanMeasure);
      }
    }
  }
}

