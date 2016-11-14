int measure = 0;         // Variable que almacena la lectura del sensor IR.
long meanMeasure = 0;    // Variable que almacena la media de las lecturas del sensor IR.    
boolean button = false;  // Variable que almacena el estado del pulsador.

int infraredPin = 0;     // Pin analogico conectado a la salida del sensor IR. 
int buttonPin = 2;       // Pin digital conectado al pull up del pulsador.

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
}

void loop() {
  meanMeasure = 0;
  while (!digitalRead(buttonPin)){}
  delayMicroseconds(52900);
 
  for (int i = 0; i < 100; i++){
    measure = analogRead(infraredPin);
    Serial.println(measure);
    meanMeasure += measure;
  }
  
  meanMeasure /= 100;
  Serial.println(meanMeasure);
  
  while (digitalRead(buttonPin)){}
 
}
