#include <ArduinoBLE.h>
#include <ESP32Time.h>
#define ENCA 2 // YELLOW from polulu
#define ENCB 15 // WHITE from polulu
#define PWM 27
#define IN2 26 //B1-A
#define IN1 25 //A1-A
#define LIGHT 34

volatile int posi = 0;  // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/

uint32_t Freq = 0;

ESP32Time rtc;
//ESP32Time rtc(7200);  // offset in seconds GMT+2

BLEService service("1ce76320-2d32-41af-b4c4-46836ea7a62a"); // Bluetooth® Low Energy LED Service
BLECharacteristic dateCharacteristic("ad804469-19ec-406a-b949-31ae17e43813", BLERead | BLENotify | BLEWrite, 9); // 8 UNIX + 1 UTC offset
BLECharacteristic lightCharacteristic("947aad02-c25d-11ed-afa1-0242ac120002", BLERead | BLENotify | BLEWrite , 4);
BLECharacteristic doorCharacteristic("c3773399-b755-4e30-9160-bed203fae718", BLERead | BLENotify | BLEWrite , 2);
BLECharacteristic doorCloseCharacteristic("e011ba0e-84c5-4e83-8648-f3e2660c44b0", BLERead | BLENotify | BLEWrite , 4);
BLECharacteristic doorOpenCharacteristic("cc959fff-4f84-4d08-a720-9d9156a48ed5", BLERead | BLENotify | BLEWrite , 4);

//badge
uint8_t ble_value = 0x0;
int analogValue = 500;
int minValue = analogValue;
int maxValue = analogValue;
int timeOffset = 0;

//settings door
int doorWantedStatus = 0;
float doorNbTurn = 1;
int count =1;
int target = 372;
int current_target =target;
int gowing_up;
bool modeAuto = false;
bool testDoor = false;

//settings close
int doorCloseMode = 0;
int doorCloseLightThreshold = -1;
int doorCloseTimeH = -1;
int doorCloseTimeM = -1;


//settings open
int doorOpenMode = 0;
int doorOpenLightThreshold = -1;
int doorOpenTimeH = -1;
int doorOpenTimeM = -1;

//door controle
int oneTurn = 372;//372
int doorStatus = 0;
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
    
      Serial.println("starting Bluetooth® Low Energy module failed!");
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
  uint8_t initialValue[] = {10, 0};
  doorCharacteristic.writeValue(initialValue, sizeof(initialValue));


  // start advertising
  BLE.advertise();

  
      Serial.println("BLE LED Peripheral");

      setCpuFrequencyMhz(160);
    Freq = getCpuFrequencyMhz();
    Serial.print("CPU Freq = ");
    Serial.println(Freq);



}

void loop() {

   

  
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
      Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    setCpuFrequencyMhz(240);
    Freq = getCpuFrequencyMhz();
    Serial.print("CPU Freq = ");
    Serial.println(Freq);

   
    

    // while the central is still connected to peripheral:
    while (central.connected()) {
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      delay(500);

      manageAutoDoor();
      manageMotor();

      manageDate();
      manageLight();
      manageSettingsDoor();
      manageSettingsClose();
      manageSettingsOpen();

      Serial.print(F("light: "));
      Serial.println(analogValue);



    }

    // when the central disconnects, print it out:
    
      Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());

    setCpuFrequencyMhz(160);
    Freq = getCpuFrequencyMhz();
    Serial.print("CPU Freq = ");
    Serial.println(Freq);
  }



  // no device connected

  manageAutoDoor();
  manageMotor();

  delay(1000);
}



void manageAutoDoor() {

  if (modeAuto) {
    Serial.println("mAuto");
    int TimeH = rtc.getHour(true);
    int TimeM = rtc.getMinute();

    bool isTimeFulfillClose = (doorCloseTimeH > 0 && ((TimeH > doorCloseTimeH + timeOffset ) || (TimeH == doorCloseTimeH + timeOffset && TimeM >= doorCloseTimeM)));

    bool isTimeFulfillOpen = (doorOpenTimeH > 0 && ((TimeH > doorOpenTimeH + timeOffset ) || (TimeH == doorOpenTimeH + timeOffset && TimeM >= doorOpenTimeM)));
    
    if (doorWantedStatus == doorStatus && doorStatus == 0) {  // door currently close
      bool openingDoor = false;
      
      bool isLightFulfill = (analogValue > doorOpenLightThreshold);
      //bool isTimeFulfill = (( (doorCloseTimeH  < 0 || doorCloseTimeH + timeOffset >  TimeH) && TimeH > doorOpenTimeH + timeOffset) || (TimeH == doorOpenTimeH +timeOffset && TimeM >= doorOpenTimeM));
      bool isTimeFulfill = isTimeFulfillOpen && !isTimeFulfillClose;
      bool isLightAndTimeFulfill = (isLightFulfill && isTimeFulfill);
      bool isLightOrTimeFulfill = (isLightFulfill || isTimeFulfill);
    
      
      switch (doorOpenMode) {
        case 1:
          openingDoor = isLightFulfill;
          break;
        case 2:
          openingDoor = isTimeFulfill;
          break;
        case 3:
          openingDoor = isLightAndTimeFulfill;
          break;
        case 4:
          openingDoor = isLightOrTimeFulfill;
          break;
        default:
          break;
      }
      
      Serial.print("op: ");
      Serial.print(openingDoor);

      Serial.print(" mod: ");
      Serial.print(doorOpenMode);

      Serial.print(" L?: ");
      Serial.print(isLightFulfill);
      
      Serial.print(" T?: ");
      Serial.print(isTimeFulfill);
      
      Serial.print(" L&T?: ");
      Serial.print(isLightAndTimeFulfill);

      Serial.print(" L|T: ");
      Serial.print(isLightOrTimeFulfill);

      Serial.print(" H: ");
      Serial.print(TimeH);

      Serial.print(" TH: ");
      Serial.print(doorOpenTimeH + timeOffset);

      Serial.println();

      

      if (openingDoor) {
        doorWantedStatus = 1;
        uint8_t doorUpdate[] = {doorCharacteristic.value()[0] , 11};
        doorCharacteristic.writeValue(doorUpdate, sizeof(doorUpdate));
      }
      
    } else if (doorWantedStatus == doorStatus && doorStatus == 1) {  // door currently open
      bool closingDoor = false;
      
      bool isLightFulfill = (analogValue < doorCloseLightThreshold);
      //bool isTimeFulfill = (doorCloseTimeH > 0 && ((TimeH > doorCloseTimeH + timeOffset ) || (TimeH == doorCloseTimeH + timeOffset && TimeM >= doorCloseTimeM)));
      bool isTimeFulfill = isTimeFulfillClose;
      bool isLightAndTimeFulfill = (isLightFulfill && isTimeFulfill);
      bool isLightOrTimeFulfill = (isLightFulfill || isTimeFulfill);
      
      
      switch (doorCloseMode) {
        case 1:
          closingDoor = isLightFulfill;
          break;
        case 2:
          closingDoor = isTimeFulfill;
          break;
        case 3:
          closingDoor = isLightAndTimeFulfill;
          break;
        case 4:
          closingDoor = isLightOrTimeFulfill;
          break;
        default:
          break;
      }

      Serial.print("cl: ");
      Serial.print(closingDoor);

      Serial.print(" mod: ");
      Serial.print(doorCloseMode);

      Serial.print(" L?: ");
      Serial.print(isLightFulfill);
      
      Serial.print(" T?: ");
      Serial.print(isTimeFulfill);
      
      Serial.print(" L&T?: ");
      Serial.print(isLightAndTimeFulfill);

      Serial.print(" L|T: ");
      Serial.print(isLightOrTimeFulfill);

      Serial.print(" H: ");
      Serial.print(TimeH );

      Serial.print(" TH: ");
      Serial.print(doorCloseTimeH + timeOffset);

      Serial.println();
      
      if (closingDoor) {
        doorWantedStatus = 0;
        uint8_t doorUpdate[] = {doorCharacteristic.value()[0] , 10};
        doorCharacteristic.writeValue(doorUpdate, sizeof(doorUpdate));
      }

      
    }
  }
}



void manageMotor() {

      Serial.print("else continue Door : ");
      Serial.print("current_target : ");
      Serial.print(current_target);
      Serial.print("doorWantedStatus : ");
      Serial.print(doorWantedStatus);
      Serial.print("doorStatus : ");
      Serial.println(doorStatus);


      if (doorWantedStatus == 0 && doorStatus != doorWantedStatus) {
      current_target = (int)0;
      
      } else if (doorWantedStatus == 1 && doorStatus != doorWantedStatus) {
        current_target = (int)oneTurn * doorNbTurn;
        
      }

      if (doorStatus != doorWantedStatus){
        runMotor(current_target);
        doorStatus = doorWantedStatus;
      }

    

    if (testDoor) {
      Serial.println("test go back");
      doorWantedStatus = !doorWantedStatus;
      testDoor = false;
    }
    count++;

  
  
}


void manageSettingsOpen() {
  if (doorOpenCharacteristic.written() ) {
    
      Serial.println("update Door Open settings");
    
    doorOpenMode = doorOpenCharacteristic.value()[0];
    doorOpenLightThreshold = doorOpenCharacteristic.value()[1]*4;
    doorOpenTimeH = doorOpenCharacteristic.value()[2];
    doorOpenTimeM = doorOpenCharacteristic.value()[3];
    Serial.print(doorOpenMode);
    Serial.print(";");
    Serial.print(doorOpenLightThreshold);
    Serial.print(";");
    Serial.print(doorOpenTimeH);
    Serial.print(";");
    Serial.println(doorOpenTimeM);

  } 
}

void manageSettingsClose() {
  if (doorCloseCharacteristic.written() ) {
    
      Serial.println("update Door Close settings");
    
    doorCloseMode = doorCloseCharacteristic.value()[0];
    doorCloseLightThreshold = doorCloseCharacteristic.value()[1]*4;
    doorCloseTimeH = doorCloseCharacteristic.value()[2];
    doorCloseTimeM = doorCloseCharacteristic.value()[3];

    Serial.print(doorCloseMode);
    Serial.print(";");
    Serial.print(doorCloseLightThreshold);
    Serial.print(";");
    Serial.print(doorCloseTimeH);
    Serial.print(";");
    Serial.println(doorCloseTimeM);

  } 
}



void manageSettingsDoor() {
 
  if (doorCharacteristic.written()  ) {
    
      Serial.println("update Door");


      doorNbTurn = doorCharacteristic.value()[0] / 10;
      modeAuto = false;
      if (doorCharacteristic.value()[1] == 2) {
        uint8_t doorUpdate[] = {doorCharacteristic.value()[0] , doorWantedStatus};
        doorCharacteristic.writeValue(doorUpdate, sizeof(doorUpdate));
        doorWantedStatus = !doorWantedStatus;
        testDoor = true;
        

      } else if (doorCharacteristic.value()[1] < 10) {
        doorWantedStatus = doorCharacteristic.value()[1];
      } else {
        doorWantedStatus = doorCharacteristic.value()[1] - 10;
        modeAuto = true;
      }
     

      

  } 

  
}




void manageLight() {
  analogValue = analogRead(LIGHT);
  Serial.print(F("light origin: "));
      Serial.println(analogValue);
  analogValue = analogValue /4;

  //4095 maxium
  if (analogValue >1000){
   analogValue = 1000;
    // maximum allowed by the BLE protocol, 1000 is bright above is very bright, not very usefull
  }
  if (lightCharacteristic.written() && lightCharacteristic.value()[3] != 0x00) {
    minValue = analogValue;
    maxValue = analogValue;
    Serial.println("reset light");

  } else {
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
  float divider = 1024 / 255.0; //1000 max allowed value, 255 max byte value
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

    timeOffset = dateCharacteristic.value()[8]-12;
    
    rtc.setTime(xx);
    Serial.println("Date update");
    Serial.println(xx);
    Serial.println(timeOffset);
    //Serial.println(rtc.getEpoch());

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
