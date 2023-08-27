#define ENCA 2   // YELLOW
#define ENCB 15  // WHITE
#define PWM 27
#define IN2 26
#define IN1 25

volatile int posi = 0;  // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/

int oneTurn = 372;


void setup() {
  Serial.begin(115200);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Serial.println("target pos");
}

void loop() {
  runMotor(oneTurn);
  delay(1000);
  runMotor(0);
  delay(1000);
}

void runMotor(int target) {
  long prevT = 0;
  float eprev = 0;
  float eintegral = 0;
  int lastPosition = -1;
  int countLastPosition = 0;
  // PID constants
  float kp = 8;     // 1 // 5
  float kd = 0.03;  // 0.025 // 0.12
  float ki = 0.0;

  while (true) {
    // Time difference
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / (1.0e6);
    prevT = currT;

    // Read the position
    int pos = 0;
    noInterrupts();  // Disable interrupts temporarily while reading
    pos = posi;
    interrupts();  // Turn interrupts back on

    if (lastPosition == pos) {
      countLastPosition++;
    } else {
      lastPosition = pos;
      countLastPosition = 0;
    }

    if (target == pos || countLastPosition > 100) {

      countLastPosition = 0;
      analogWrite(PWM, 0);
      break;  // Exit the loop when target is reached
    }

    // Error
    int e = pos - target;

    // Derivative
    float dedt = (e - eprev) / (deltaT);

    // Integral
    eintegral = eintegral + e * deltaT;

    // Control signal
    float u = kp * e + kd * dedt + ki * eintegral;

    // Motor power
    float pwr = fabs(u);
    if (pwr > 120) {
      pwr = 120;
    }

    if (pwr < 50) {
      pwr = 0;
    }

    // Motor direction
    int dir = 1;
    if (u < 0) {
      dir = -1;
    }

    // Signal the motor
    setMotor(dir, pwr, PWM, IN1, IN2);

    // Store previous error
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
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    posi++;
  } else {
    posi--;
  }
}
