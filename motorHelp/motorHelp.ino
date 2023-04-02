// include the library for the encoder
#include <Encoder.h>

// define the pins for the encoder
#define ENC_A 2
#define ENC_B 15

// create an Encoder object
Encoder myEnc(ENC_A, ENC_B);

// define the pins for the motor control
#define MOTOR_1 26
#define MOTOR_2 25

// define the target count
#define TARGET_COUNT 370

void setup() {
  // set the motor control pins as outputs
  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_2, OUTPUT);
}

void loop() {
  // read the encoder count
  long count = myEnc.read();

  // check if the target count has been reached
  if (count < TARGET_COUNT) {
    // set the motor speed and direction
    digitalWrite(MOTOR_1, HIGH);
    digitalWrite(MOTOR_2, LOW);
  } else {
    // stop the motor
    digitalWrite(MOTOR_1, LOW);
    digitalWrite(MOTOR_2, LOW);
  }
}
