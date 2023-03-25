#define ENCA 4  // pin2 of the Arduino
#define ENCB 0 // Pin3 of the Arduino
int ENCA_DATA;
int ENCB_DATA;

void setup() {
  Serial.begin(9600); // Activates Serial communication
  pinMode(ENCA,INPUT); // sets pin2 as the input
  pinMode(ENCB,INPUT); // sets pin3 as the input
}

void loop() {
  ENCA_DATA = digitalRead(ENCA); 
// We simply read Pin2 of the Arduino and store the result in variable ENCA_DATA
  ENCB_DATA = digitalRead(ENCB); 
// We simply read Pin3 of the Arduino and store the result in variable b
  Serial.print(ENCA_DATA*5); 
  Serial.print(" ");
  Serial.print(ENCB_DATA*5);
  Serial.println();
}
