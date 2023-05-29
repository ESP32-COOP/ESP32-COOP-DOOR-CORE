#include <ArduinoBLE.h>
#include <ESP32Time.h>
#define ENCA 2 // YELLOW from polulu
#define ENCB 15 // WHITE from polulu
#define PWM 27
#define IN2 26 //B1-A
#define IN1 25 //A1-A

volatile int posi = 0;  // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/

ESP32Time rtc;
//ESP32Time rtc(3600);  // offset in seconds GMT+1

BLEService service("1ce76320-2d32-41af-b4c4-46836ea7a62a"); // Bluetooth® Low Energy LED Service
BLECharacteristic dateCharacteristic("ad804469-19ec-406a-b949-31ae17e43813", BLERead | BLENotify | BLEWrite, 8);
BLECharacteristic lightCharacteristic("947aad02-c25d-11ed-afa1-0242ac120002", BLERead | BLENotify | BLEWrite , 4);
BLECharacteristic doorCharacteristic("c3773399-b755-4e30-9160-bed203fae718", BLERead | BLENotify | BLEWrite , 2);
BLECharacteristic doorCloseCharacteristic("e011ba0e-84c5-4e83-8648-f3e2660c44b0", BLERead | BLENotify | BLEWrite , 4);
BLECharacteristic doorOpenCharacteristic("cc959fff-4f84-4d08-a720-9d9156a48ed5", BLERead | BLENotify | BLEWrite , 4);

//badge
uint8_t ble_value = 0x0;
int analogValue = 500;
int minValue = analogValue;
int maxValue = analogValue;

//settings door
int doorWantedStatus = 1;
float doorNbTurn = 1;
int count =1;
int target = 372;
int current_target =target;
int gowing_up;

//settings close
int doorCloseMode = 0;
int doorCloseLightThreshold = -1;
int doorCloseTimeH = -1;
int doorCloseTImeM = -1;

//settings open
int doorOpenMode = 0;
int doorOpenLightThreshold = -1;
int doorOpenTimeH = -1;
int doorOpenTImeM = -1;

//door controle
int oneTurn = 372;//372
int doorStaus = 1;
float kp = 8;
float kd = 1;
float ki = 0.01;


void setup() {
  Serial.begin(115200);
  rtc.setTime(1680108200);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  while (!Serial);

  // begin initialization
  if (!BLE.begin()) {
    
    noInterrupts();
      Serial.println("starting Bluetooth® Low Energy module failed!");
      interrupts();
    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("COOP-DOOR");
  BLE.setAdvertisedService(service);
  service.addCharacteristic(dateCharacteristic);
  service.addCharacteristic(lightCharacteristic);
  service.addCharacteristic(doorCharacteristic);
  service.addCharacteristic(doorCloseCharacteristic);
  service.addCharacteristic(doorOpenCharacteristic);
  BLE.addService(service);
  dateCharacteristic.writeValue(0);
  lightCharacteristic.writeValue(0);
  uint8_t initialValue[] = {10, 1};
  doorCharacteristic.writeValue(initialValue, sizeof(initialValue));


  // start advertising
  BLE.advertise();

  
  noInterrupts();
      Serial.println("BLE LED Peripheral");
      interrupts();



}

void loop() {
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    noInterrupts();
      Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
      interrupts();
    

    // while the central is still connected to peripheral:
    while (central.connected()) {
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      delay(500);

      manageDate();
      manageLight();
      manageSettingsDoor();
      manageSettingsClose();
      manageSettingsOpen();



    }

    // when the central disconnects, print it out:
    
    noInterrupts();
      Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
      interrupts();
  }





}

void manageSettingsOpen() {
  if (doorOpenCharacteristic.written() ) {
    
    noInterrupts();
      Serial.println("update Door Open settings");
    Serial.print(doorOpenCharacteristic.value()[0]);
    Serial.print(";");
    Serial.print(doorOpenCharacteristic.value()[1]);
    Serial.print(";");
    Serial.print(doorOpenCharacteristic.value()[2]);
    Serial.print(";");
    Serial.println(doorOpenCharacteristic.value()[3]);
      interrupts();
    doorOpenMode = doorOpenCharacteristic.value()[0];
    doorOpenLightThreshold = doorOpenCharacteristic.value()[1];
    doorOpenTimeH = doorOpenCharacteristic.value()[2];
    doorOpenTImeM = doorOpenCharacteristic.value()[3];

  } else {
    //todo
  }
}

void manageSettingsClose() {
  if (doorCloseCharacteristic.written() ) {
    
    noInterrupts();
      Serial.println("update Door Close settings");
    Serial.print(doorCloseCharacteristic.value()[0]);
    Serial.print(";");
    Serial.print(doorCloseCharacteristic.value()[1]);
    Serial.print(";");
    Serial.print(doorCloseCharacteristic.value()[2]);
    Serial.print(";");
    Serial.println(doorCloseCharacteristic.value()[3]);
      interrupts();
    doorCloseMode = doorCloseCharacteristic.value()[0];
    doorCloseLightThreshold = doorCloseCharacteristic.value()[1];
    doorCloseTimeH = doorCloseCharacteristic.value()[2];
    doorCloseTImeM = doorCloseCharacteristic.value()[3];

  } else {
    //todo
  }
}



void manageSettingsDoor() {
 
  if (doorCharacteristic.written() ) {
    
    noInterrupts();
      Serial.println("update Door");
      interrupts();

      doorWantedStatus = doorCharacteristic.value()[1];
      if (doorWantedStatus == 0){
        current_target = 0;
        gowing_up = false;
      }else{
        current_target = target;
          gowing_up = true;
      }

    
 

  
    runMotor(current_target);
    doorStaus = doorWantedStatus;
    count++;


  } else {
    noInterrupts();
      Serial.println("else continue Door");
      Serial.println(current_target);

      interrupts();

    

    doorStaus = doorWantedStatus;
    count++;

  }

  
}




void manageLight() {
  if (lightCharacteristic.written() && lightCharacteristic.value()[3] != 0x00) {
    analogValue = random(10, 1000);
    minValue = analogValue;
    maxValue = analogValue;
    Serial.println("reset light");


  } else {
    analogValue = random(10, 1000);
    minValue = min(analogValue, minValue);
    maxValue = max(analogValue, maxValue);
  }
  /*
    Serial.print(" value ");
    Serial.print(analogValue);
    Serial.print(" minValue ");
    Serial.print(minValue);
    Serial.print(" maxValue ");
    Serial.print(maxValue);
    Serial.println("");
  */
  float divider = 1000 / 255.0; //1000 max allowed value, 255 max byte value
  uint8_t currentValue = analogValue / divider;
  uint8_t scaledMinValue = minValue / divider;
  uint8_t scaledMaxValue = maxValue / divider;

  uint8_t ble_value_array[4] = {currentValue, scaledMinValue, scaledMaxValue, 0x00 };

  // Write the array to the characteristic
  lightCharacteristic.writeValue(ble_value_array, 4);


}





void manageDate() {
  if (dateCharacteristic.written()) {
    
    long xx = getLongFromBytes(dateCharacteristic.value());
    
    rtc.setTime(xx);
    noInterrupts();
    Serial.println("Date update");
    Serial.println(xx);
    Serial.println(rtc.getEpoch());
    interrupts();

  } else {
    //Serial.println(rtc.getEpoch());
    byte* tmpDate = getBytesFromLong(rtc.getEpoch());

    dateCharacteristic.writeValue(tmpDate, 8);
    delete[] tmpDate;
  }

}


byte* getBytesFromLong(long x  ) {
  byte* bytes = new byte[8];
  for (int i = 0; i < 8; i++) {
    bytes[i] = x & 0xff;
    x = (x - bytes[i]) / 256;
  }
  return bytes;

}

long getLongFromBytes(const byte* bytes) {
  long result = 0;
  for (int i = 7; i >= 0; i--) {
    result = (result * 256) + bytes[i];
  }
  return result;
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


