

// this constant won't change:
const int  EchoPin = 12;    // the pin that the pushbutton is attached to
const int TrigPin = 13;       // the pin that the LED is attached to

// Variables will change:

     // previous state of the button

void setup() {
  // initialize the echo pin as a input:
  pinMode(EchoPin, INPUT);
  // initialize the trig pin as an output:
  pinMode(TrigPin, OUTPUT);
  // initialize serial communication:
  Serial.begin(9600);
}


void loop() {
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);
  while(digitalRead(EchoPin)==LOW){
      
    }
  long start;
  long endTime;
  start = micros();
  while(digitalRead(EchoPin)==HIGH){
      
    }
  endTime = micros();
  Serial.println((endTime-start)/58);
  delay(100);
}









