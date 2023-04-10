/*     Arduino Rotary Encoder Tutorial
 *      
 *  by Dejan Nedelkovski, www.HowToMechatronics.com
 *  
 */
 
 #define outputA 2
 #define outputB 15
 #define M1 26
 #define M2 25

 int counter = 0; 
 int aState;
 int aLastState;  

 void setup() { 
   pinMode (outputA,INPUT);
   pinMode (outputB,INPUT);
   
   Serial.begin (115200);
   // Reads the initial state of the outputA
   aLastState = digitalRead(outputA);   
 } 

 void loop() { 
   aState = digitalRead(outputA); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (aState != aLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(outputB) != aState) { 
       counter ++;
     } else {
       counter --;
     }
     Serial.print("Position: ");
     Serial.println(counter);
   } 
   aLastState = aState; // Updates the previous state of the outputA with the current state

   


   if (counter <= -100){
    
    digitalWrite(M2,HIGH);
    digitalWrite(M1,HIGH);
    Serial.println("motor 0");
   }else{
    analogWrite(M2,100);
    digitalWrite(M1,HIGH);
    Serial.println("motor 1");
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
