char incomingByte;

int a = 2; // LSB multiplexer
int b = 3; //
int c = 4; // MSB multiplexer
int readPin = A0; // Select the output channel

long meanMeasure = 0;
int sensorValue = 0;  // variable to store the value coming from the sensor

const float ADC_REF = 5000; // Value of the ADC reference voltage (in millivolts) (Usually 5000mV for Arduino boards)

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
        Serial.println("MiniSkybot: IR Sensor Calibration Program");
        break;
      }    
    case 'm': // Send measure of the selected sensor to the computer
      {
        digitalWrite(13,HIGH);
        meanMeasure = 0;
        for (int i = 0; i < 10; i++){
          sensorValue = analogRead(readPin);
          meanMeasure += sensorValue;
          delay(50);
        }
        digitalWrite(13,LOW);
      meanMeasure/= 10;  
      int measured_mV = meanMeasure * ADC_REF / 1023.0;
      Serial.println(measured_mV);
      }
    }
  }
}

