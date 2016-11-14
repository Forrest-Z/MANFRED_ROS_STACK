char incomingByte;

int a = 2; // LSB multiplexer
int b = 3; //
int c = 4; // MSB multiplexer
int readPin; // Select the output channel

float sensorValue = 0;  // variable to store the value coming from the sensor

void sendIRdata(int number, boolean A, boolean B, boolean C, int AN, float alpha, float beta, float x, float y, float theta, int minimum, int maximum, int fov){
            
  digitalWrite(a, A);
  digitalWrite(b, B);
  digitalWrite(c, C);
            
  sensorValue = analogRead(AN);
  sensorValue = sensorValue*5000/1024; 
  sensorValue = alpha*pow(sensorValue,-beta);
  
  Serial.print("IR_L");
  Serial.print(number);
  Serial.print("\t");   
  Serial.print(sensorValue);
  Serial.print("\t");
  Serial.print(x);
  Serial.print("\t");
  Serial.print(y);
  Serial.print("\t");
  Serial.print(theta);
  Serial.print("\t");
  Serial.print(minimum);
  Serial.print("\t");
  Serial.print(maximum);
  Serial.print("\t");
  Serial.println(fov);  
}

void setup() {
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
  //Serial.println("Hi there! I'm Arduino");
   pinMode(a, OUTPUT);
   pinMode(b, OUTPUT);
   pinMode(c, OUTPUT);
   Serial.begin(9600);
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
         sendIRdata(1, 0, 1, 1, A0, 350.8, 1.036,75, 0, 0, 100, 800, 8);
         sendIRdata(0, 0, 1, 1, A1, 350.8, 1.036,75, 0, 0, 100, 800, 8);
         sendIRdata(4, 1, 0, 1, A0, 350.8, 1.036,75, 0, 0, 100, 800, 8);
         
         Serial.println();
      }
    }
  }
}


