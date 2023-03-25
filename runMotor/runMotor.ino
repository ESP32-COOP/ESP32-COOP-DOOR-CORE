   /*
   L9110S-Stepper-DC-motor-Driver-Module
  made on 28 oct 2020
  by Amir Mohammad Shojaee @ Electropeak
  Home

*/

#define A1 25  // Motor A pins
#define A2 26
#define ENCA 2  // pin2 of the Arduino
#define ENCB 15 // Pin3 of the Arduino
int ENCA_DATA;
int ENCB_DATA;



int incomingByte = 0; // for incoming serial data

void setup() {

  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(ENCA,INPUT); // sets pin2 as the input
  pinMode(ENCB,INPUT); // sets pin3 as the input


  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);

  
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps

}
int  input = 0;
void loop() {
  ENCA_DATA = digitalRead(ENCA); 
  // We simply read Pin2 of the Arduino and store the result in variable ENCA_DATA
  ENCB_DATA = digitalRead(ENCB); 
  // We simply read Pin3 of the Arduino and store the result in variable b
  Serial.print(ENCA_DATA*5); 
  Serial.print(" ");
  Serial.print(ENCB_DATA*5);
  Serial.println();

  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    input = incomingByte - 48; //convert ASCII code of numbers to 1,2,3

  switch (input) { 
    case 1:         // if input=1 ....... motors turn forward
      forward();
      break;
    case 2:         // if input=2 ....... motors turn backward
      backward();
      break;
    case 3:         // if input=1 ....... motors turn stop
      Stop();
      break;
  }
  delay(200);
  input=0;
}
}
void forward() {          //function of forward 
  //Serial.println("forward");
  digitalWrite(A1, HIGH);
  digitalWrite(A2, LOW);
}

void backward() {         //function of backward
  //Serial.println("backward");
  digitalWrite(A1, LOW);
  digitalWrite(A2, HIGH);
}

void Stop() {              //function of stop
  //Serial.println("stop");
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
}
