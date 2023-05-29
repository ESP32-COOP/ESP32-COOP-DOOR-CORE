// This alternate version of the code does not require
// atomic.h. Instead, interrupts() and noInterrupts() 
// are used. Please use this code if your 
// platform does not support ATOMIC_BLOCK.

#define ENCA 2 // YELLOW
#define ENCB 15 // WHITE
#define PWM 27
#define IN2 26
#define IN1 25

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;
int oneTurn = 372;
int target = oneTurn;
int lastPosition = -1;
int countLastPosition = 0;

void setup() {
  Serial.begin(115200);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  Serial.println("target pos");

}

void loop() {

  // set target position
  //int target = 1200;
  

  // PID constants
  float kp = 8;//1//5
  float kd = 0.03;//0.025//0.12
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position
  int pos = 0; 
  noInterrupts(); // disable interrupts temporarily while reading
  pos = posi;
  interrupts(); // turn interrupts back on
  if (lastPosition == pos){
    countLastPosition++;
  }else{
    lastPosition = pos;
    countLastPosition = 0;
  }

  if (target == pos || countLastPosition > 100){
    if (target == oneTurn){
      target= 0;
    }else{
      target = oneTurn;
    }
    countLastPosition = 0;
    analogWrite(PWM,0);
    delay(1000);
  }
  
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
  if( pwr > 120  ){
    pwr = 120;
  }

  if( pwr < 50  ){
    pwr = 0;
  }
  

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.print(" ");
  Serial.print(pwr);
  Serial.print(" ");
  Serial.print(countLastPosition);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}