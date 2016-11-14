#include <Servo.h> 
#include <math.h>
Servo scannerServo;
int pos = 0;
  
float IRdistance(){
  float ARef = 2.5; // V
  float distance; // Output of the function (Distance of the target in m)

  // This lookup table contains the voltage in tenths of mV for each distance (from 0 to 89 cm)
  static int Volts2cmTable[] = {0,0,0,0,0,0,0,0,0,0,214,196,187,173,162,155,141,132,125,123,112,109,110,110,108,100,88,88,87,87,78,78,75,75,72,72,64,64,62,62,57,57,54,54,51,51,51,51,50,50,47,47,44,44,44,44,41,41,39,39,39,39,38,38,38,38,38,38,38,38,36,36,36,36,36,33,33,33,33,33,29,29,29,29,29,27,27,27,27,27};

  int sensorV = analogRead(0)*ARef*100/1024; // Sensor output in 0.01V's
  
  for (int d = 10; d < 90; d++)
    if (sensorV > Volts2cmTable[d]){
      distance = d/100.0;
      break;
    }
  
  return(distance);  
}

float deg2rad(float deg){
    return(deg * 3.14 / 180.0 - (M_PI/2) ); 
}

void setup(){
  Serial.begin(9600);
  analogReference(EXTERNAL);
  scannerServo.attach(9);
  scannerServo.write(0);
  delay(1000);
  digitalWrite(13,HIGH);
  //Serial.println("Start");
  //delay(1000);
}


void loop() 
{ 
  for(pos = 0; pos < 180; pos += 1)  
  {                                  
    scannerServo.write(pos);
    Serial.print("i");
    Serial.print("\t");
    Serial.print(deg2rad(pos));
    Serial.print("\t");
    Serial.println(IRdistance());
    delay(50);
  }
  Serial.println("End");
  digitalWrite(13,LOW);
  while(1){
    /*Do nothing*/;
  }
} 
