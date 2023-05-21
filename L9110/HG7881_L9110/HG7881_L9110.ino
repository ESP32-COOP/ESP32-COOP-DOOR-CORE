/********************************************************
 * PID Basic Example
 2
 ********************************************************/

#include <Motor_PID.h>
#define ENCA 2 // YELLOW from polulu
#define ENCB 15 // WHITE from polulu

#define IN2 26 //B1-A
#define IN1 25 //A1-A
int count =1;
long prevT;
int target = 372;//372
float kp = 8;
float kd = 1;
float ki = 0.01;
//motor motor1(ENCA, ENCB, IN1, IN2);
motor motor1 = motor(ENCA, ENCB, IN1, IN2, 0, 50, 100); // Set upper limit to 100
void setup()
{
  Serial.begin(115200);
   
   
    motor1.init(kp, ki, kd);
	
}
void loop()
{
   noInterrupts();
  Serial.println("step1");
  interrupts();


    if (count%2 == 0){
      motor1.set_target(0);
    }else{
      motor1.set_target(target);
    }


     noInterrupts();
  Serial.println("step2");
  interrupts();

  
    motor1.start();
    
  while (!motor1.target_reached()) {
    motor1.start();

     
  }
   noInterrupts();
  Serial.println("step3");

  interrupts();
  delay(1000); // Wait for 1 second
    //motor1.turn_off();

    count++;
    


}
