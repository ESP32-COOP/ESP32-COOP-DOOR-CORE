
#define A1 25  // Motor A pins
#define A2 26
#define ENCA 2  // pin2 of the Arduino
#define ENCB 15 // Pin3 of the Arduino
int ENCA_DATA;
int ENCB_DATA;

volatile  int posi;

//PID
long prevT = 0;
float eprev = 0;
float eintegral = 0;
int target;
int fd = 0;


int incomingByte = 0; // for incoming serial data

void setup() {

  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(ENCA,INPUT); // sets pin2 as the input
  pinMode(ENCB,INPUT); // sets pin3 as the input


  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);

  
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

}
int  input = 0;
void loop() {

  //PID 
    // set target position
  target = 219;//217 218
  //int target = 250*sin(prevT/1e6);

  // PID constants
  float kp = 1.5;//1.5
  float kd = 0.025; //0.025
  float ki = 0.02;//0.02

  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

   int pos = 0; 
  noInterrupts(); // disable interrupts temporarily while reading
  pos = posi;
  interrupts(); // turn interrupts back on
  
   // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  if (pwr !=255){
    dir=0;
  }

  // signal the motor
  //if (abs(e)>10&& (fd ==0 || fd == dir)){
  /*
  if (pos<target){
    fd = dir;
    dir =-1;
    */
  setMotor(dir,pwr,A1,A2);
  /*
  Serial.print(100);
  Serial.print(" ");
  }else{
    Serial.print(0);
  Serial.print(" ");
    setMotor(0,0,A1,A2);
  }
  */

  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.print(" ");
  Serial.print(pwr);
  Serial.print(" ");
  Serial.print(dir*100);
  
  Serial.println();

  /*
  
  ENCA_DATA = digitalRead(ENCA); 
  // We simply read Pin2 of the Arduino and store the result in variable ENCA_DATA
  ENCB_DATA = digitalRead(ENCB); 
  // We simply read Pin3 of the Arduino and store the result in variable b
  Serial.print(ENCA_DATA*5); 
  Serial.print(" ");
  Serial.print(ENCB_DATA*5);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
  */
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
  input=0;
}

}

void setMotor(int dir,int pwmVal, int in1, int in2){
  if(dir == 1){
    analogWrite(in1,pwmVal);
    //digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    analogWrite(in2,pwmVal);
    //digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void forward() {          //function of forward 
   target = 1200;
}

void backward() {         //function of backward
   target = -1200;
}

void Stop() {              //function of stop
   target = 0;
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b>0){
    posi++;
  }else{
    posi--;
  }
}
