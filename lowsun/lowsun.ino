#include <ArduinoBLE.h>
#include <ESP32Time.h>
#include <Motor_PID.h>
#define ENCA 2   // YELLOW from polulu
#define ENCB 15  // WHITE from polulu

#define IN2 26  //B1-A
#define IN1 25  //A1-A

ESP32Time rtc;
//ESP32Time rtc(3600);  // offset in seconds GMT+1

BLEService service("1ce76320-2d32-41af-b4c4-46836ea7a62a");  // Bluetooth® Low Energy LED Service
BLECharacteristic dateCharacteristic("ad804469-19ec-406a-b949-31ae17e43813", BLERead | BLENotify | BLEWrite, 8);
BLECharacteristic lightCharacteristic("947aad02-c25d-11ed-afa1-0242ac120002", BLERead | BLENotify | BLEWrite, 4);
BLECharacteristic doorCharacteristic("c3773399-b755-4e30-9160-bed203fae718", BLERead | BLENotify | BLEWrite, 2);
BLECharacteristic doorCloseCharacteristic("e011ba0e-84c5-4e83-8648-f3e2660c44b0", BLERead | BLENotify | BLEWrite, 4);
BLECharacteristic doorOpenCharacteristic("cc959fff-4f84-4d08-a720-9d9156a48ed5", BLERead | BLENotify | BLEWrite, 4);

//badge
uint8_t ble_value = 0x0;
int analogValue = 500;
int minValue = analogValue;
int maxValue = analogValue;

//settings door
int doorWantedStatus = 0;
float doorNbTurn = 1;
bool testDoor = false;
bool modeAuto = false;
int count = 1;
int gowing_up;

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
int oneTurn = 372;  //372
int current_target = oneTurn;
int doorStatus = 0;
float kp = 8;
float kd = 1;
float ki = 0.01;
motor motor1 = motor(ENCA, ENCB, IN1, IN2, 0, 50, 100);  // Set upper limit to 100
//motor motor1 = motor(ENCA, ENCB, IN1, IN2);  // Set upper limit to 100


void setup() {
  Serial.begin(115200);
  rtc.setTime(1680108200);
  while (!Serial)
    ;

  // begin initialization
  if (!BLE.begin()) {

    noInterrupts();
    Serial.println("starting Bluetooth® Low Energy module failed!");
    interrupts();
    while (1)
      ;
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
  uint8_t initialValue[] = { 10, 0 };
  doorCharacteristic.writeValue(initialValue, sizeof(initialValue));
  uint8_t initialValuedoorCloseCharacteristic[] = { 0, 0, 0, 0 };
  doorCloseCharacteristic.writeValue(initialValuedoorCloseCharacteristic, sizeof(initialValuedoorCloseCharacteristic));
  uint8_t initialValuedoorOpenCharacteristic[] = { 0, 0, 0, 0 };
  doorOpenCharacteristic.writeValue(initialValuedoorOpenCharacteristic, sizeof(initialValuedoorOpenCharacteristic));


  // start advertising
  BLE.advertise();


  noInterrupts();
  Serial.println("BLE LED Peripheral");
  interrupts();

  //door controler
  motor1.init(kp, ki, kd);
  //motor1.set_position(0);
  //motor1.set_target(10);
  motor1.turn_off();
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
      manageAutoDoor();
    }

    // when the central disconnects, print it out:

    noInterrupts();
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
    interrupts();
  }
}

void manageAutoDoor() {

  if (modeAuto) {
    noInterrupts();
    Serial.println("modeAuto");
    interrupts();
    if (doorWantedStatus == doorStatus && doorStatus == 0) {  // door currently close
      bool openingDoor = false;
      bool isLightFulfill = (analogValue > doorOpenLightThreshold);
      bool isTimeFulfill = (rtc.getHour() > doorOpenTimeH && rtc.getMinute() > doorOpenTimeM);
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
        case 4:
          openingDoor = isLightOrTimeFulfill;
          break;
        default:
          break;
      }

      noInterrupts();
      Serial.print("openingDoor: ");
      Serial.println(openingDoor);

      Serial.print("isLightFulfill: ");
      Serial.println(isLightFulfill);

      Serial.print("isTimeFulfill: ");
      Serial.println(isTimeFulfill);

      Serial.print("isLightAndTimeFulfill: ");
      Serial.println(isLightAndTimeFulfill);

      Serial.print("isLightOrTimeFulfill: ");
      Serial.println(isLightOrTimeFulfill);
      interrupts();

      if (openingDoor) {
        doorWantedStatus = 1;
        //uint8_t currentDoorNbTurn = doorCharacteristic.value()[0];
        uint8_t currentDoorStatus = 11;


        //uint8_t ble_value_array[2] = { currentDoorNbTurn, currentDoorStatus };

        // Write the array to the characteristic
        //doorCharacteristic.writeValue(ble_value_array, 2);
      }
    } else if (doorWantedStatus == doorStatus && doorStatus == 1) {  // door currently open
      bool closingDoor = false;
      bool isLightFulfill = (analogValue < doorCloseLightThreshold);
      bool isTimeFulfill = (rtc.getHour() > doorCloseTimeH && rtc.getMinute() > doorCloseTimeM);
      bool isLightAndTimeFulfill = (isLightFulfill && isTimeFulfill);
      bool isLightOrTimeFulfill = (isLightFulfill || isTimeFulfill);
      
      switch (doorOpenMode) {
        case 1:
          closingDoor = isLightFulfill;
          break;
        case 2:
          closingDoor = isTimeFulfill;
          break;
        case 3:
          closingDoor = isLightAndTimeFulfill;
        case 4:
          closingDoor = isLightOrTimeFulfill;
          break;
        default:
          break;
      }

      noInterrupts();
      Serial.print("closingDoor: ");
      Serial.println(closingDoor);

      Serial.print("isLightFulfill: ");
      Serial.println(isLightFulfill);

      Serial.print("isTimeFulfill: ");
      Serial.println(isTimeFulfill);

      Serial.print("isLightAndTimeFulfill: ");
      Serial.println(isLightAndTimeFulfill);

      Serial.print("isLightOrTimeFulfill: ");
      Serial.println(isLightOrTimeFulfill);
      interrupts();

      if (closingDoor) {
        doorWantedStatus = 0;
        //uint8_t currentDoorNbTurn = doorCharacteristic.value()[0];
        uint8_t currentDoorStatus = 10;


        //uint8_t ble_value_array[2] = { currentDoorNbTurn, currentDoorStatus };

        // Write the array to the characteristic
        //doorCharacteristic.writeValue(ble_value_array, 2);
      }
    }
  }
}


void manageSettingsOpen() {
  if (doorOpenCharacteristic.written()) {

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
    doorOpenTimeM = doorOpenCharacteristic.value()[3];

  } else {
    //todo
  }
}

void manageSettingsClose() {
  if (doorCloseCharacteristic.written()) {

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
    doorCloseTimeM = doorCloseCharacteristic.value()[3];

  } else {
    //todo
  }
}



void manageSettingsDoor() {
  int last_motor_position = 0;
  int same_for = 0;

  if (doorCharacteristic.written()) {
    noInterrupts();
    Serial.println("update Door Close settings");
    interrupts();

    doorNbTurn = doorCharacteristic.value()[0] / 10;
    modeAuto = false;
    if (doorCharacteristic.value()[1] == 2) {
      doorWantedStatus = !doorWantedStatus;
      testDoor = true;
    } else if (doorCharacteristic.value()[1] < 10) {
      doorWantedStatus = doorCharacteristic.value()[1];
    } else {
      doorWantedStatus = doorCharacteristic.value()[1] - 10;
      modeAuto = true;
    }

    if (doorWantedStatus == 0 && doorStatus != doorWantedStatus) {
      current_target = (int)0;
      gowing_up = true;
    } else if (doorWantedStatus == 1 && doorStatus != doorWantedStatus) {
      current_target = (int)oneTurn * doorNbTurn;
      gowing_up = false;
    }


    motor1.set_target(current_target);
    motor1.start();


    while (!motor1.target_reached()) {  // || same_for > 10) {
      motor1.start();
    }

    delay(200);  // Wait for 1 second
    motor1.turn_off();


    count++;


  } else {



    motor1.set_target(current_target);
    motor1.start();

    while (!motor1.target_reached()) {

      motor1.start();
    }

    //delay(1000); // Wait for 1 second
    motor1.turn_off();

    doorStatus = doorWantedStatus;
    if (testDoor) {
      doorWantedStatus = !doorWantedStatus;
      testDoor = false;
    }
    count++;
  }
}



/*



void manageSettingsDoor() {
  int last_motor_position = motor1.get_position();
  int same_nb_time = 0;
  if (doorCharacteristic.written() ) {
    
    noInterrupts();
      Serial.println("update Door");
      interrupts();

    
   if (count%2 == 0){
    current_target = 0;
    }else{
      current_target = target;
    }
    motor1.set_target(current_target);
    motor1.start();

    
    while (!motor1.target_reached()) {
      
      motor1.start();
    
      if (motor1.get_position() == last_motor_position){
        same_nb_time++;
      }

      if (same_nb_time >10){
        break;
      }
        
        last_motor_position = motor1.get_position();
      
      
    }
    
 
    delay(1000); // Wait for 1 second
    motor1.turn_off();
    //if(motor1.get_position()==current_target) motor1.turn_off();
    
    doorStatus = doorWantedStatus;
    count++;


  } else {
    noInterrupts();
      Serial.println("else continue Door");
      Serial.println(motor1.target_reached());
      interrupts();

    
   motor1.set_target(current_target);
    motor1.start();

   
      while (!motor1.target_reached()) {
      
      motor1.start();
    
      if (motor1.get_position() == last_motor_position){
        same_nb_time++;
      }

      if (same_nb_time >10){
        break;
      }
        
        last_motor_position = motor1.get_position();
      
      
    }
      
    
    
    delay(1000); // Wait for 1 second
    motor1.turn_off();
    //if(motor1.get_position()==current_target) motor1.turn_off();
    
    doorStatus = doorWantedStatus;
    count++;

  }

  
}

*/

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
  float divider = 1000 / 255.0;  //1000 max allowed value, 255 max byte value
  uint8_t currentValue = analogValue / divider;
  uint8_t scaledMinValue = minValue / divider;
  uint8_t scaledMaxValue = maxValue / divider;

  uint8_t ble_value_array[4] = { currentValue, scaledMinValue, scaledMaxValue, 0x00 };

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


    Serial.print(dateCharacteristic.value()[0]);
    Serial.print(";");
    Serial.print(dateCharacteristic.value()[1]);
    Serial.print(";");
    Serial.print(dateCharacteristic.value()[2]);
    Serial.print(";");
    Serial.println(dateCharacteristic.value()[3]);
    //Serial.println(rtc.getEpoch());
    interrupts();

  } else {
    //Serial.println(rtc.getEpoch());
    byte* tmpDate = getBytesFromLong(rtc.getEpoch());

    dateCharacteristic.writeValue(tmpDate, 8);
    delete[] tmpDate;
  }
}


byte* getBytesFromLong(long x) {
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

/*#include <ArduinoBLE.h>
#include <ESP32Time.h>
#include <Motor_PID.h>
#define ENCA 2 // YELLOW from polulu
#define ENCB 15 // WHITE from polulu

#define IN2 26 //B1-A
#define IN1 25 //A1-A

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
int target;

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
int doorStatus = 1;
float kp = 8;
float kd = 1;
float ki = 0.01;
motor motor1 = motor(ENCA, ENCB, IN1, IN2, 0, 50, 100); // Set upper limit to 100


void setup() {
  Serial.begin(115200);
  rtc.setTime(1680108200);
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

  //door controler
  motor1.init(kp, ki, kd);
  //motor1.set_position(0);
  //motor1.set_target(10);
  motor1.turn_off();

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
      //delay(500);

      manageSettingsDoor();




    }

    // when the central disconnects, print it out:
    
    noInterrupts();
      Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
      interrupts();
  }





}


void manageSettingsDoor() {
  if (doorCharacteristic.written() ) {
    
    noInterrupts();
      Serial.println("update Door");
    Serial.print(doorCharacteristic.value()[0]);
    Serial.print(";");
    Serial.println(doorCharacteristic.value()[1]);
    Serial.println("doorNbTurn");
      interrupts();
    doorNbTurn = doorCharacteristic.value()[0] / 10;
    doorWantedStatus = doorCharacteristic.value()[1];
    
    if (doorWantedStatus == 0 && doorStatus != doorWantedStatus ) {
      target = (int) oneTurn * doorNbTurn;
      noInterrupts();
      
      Serial.println(target); // * doorNbTurn);
      interrupts();
      
      motor1.set_target(target); // * doorNbTurn);
    } else if (doorWantedStatus == 1 && doorStatus != doorWantedStatus) {
      target = (int) oneTurn * 0;
      noInterrupts();
      
      Serial.println(target); // * doorNbTurn*-1 );
      interrupts();
      motor1.set_target(target); // * doorNbTurn * -1);

    }
    motor1.start();

    while (!motor1.target_reached()) {
      
      motor1.start();
      
    }
    
    delay(1000); // Wait for 1 second
    motor1.turn_off();
    
    doorStatus = doorWantedStatus;


  } else {
    //motor1.start();

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
*/
