#include <ArduinoBLE.h>
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

    
    motor1.set_target(current_target);
    motor1.start();

  
    while (!motor1.target_reached()) {
      
      motor1.start();
      
    }
    
    delay(200); // Wait for 1 second
    motor1.turn_off();
    
    doorStaus = doorWantedStatus;
    count++;


  } else {
    noInterrupts();
      Serial.println("else continue Door");
      Serial.println(current_target);
      Serial.println(motor1.target_reached());
      interrupts();

    
   motor1.set_target(current_target);
    motor1.start();

    while (!motor1.target_reached()) {
      
      motor1.start();
      
    }
    
    //delay(1000); // Wait for 1 second
    motor1.turn_off();
    
    doorStaus = doorWantedStatus;
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
    
    doorStaus = doorWantedStatus;
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
    
    doorStaus = doorWantedStatus;
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
int target = 372;
int current_target = target;
int count =1;

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
      
    }
    
    delay(1000); // Wait for 1 second
    motor1.turn_off();
    
    doorStaus = doorWantedStatus;
    count++;


  } else {
    noInterrupts();
      Serial.println("else continue Door");
      interrupts();

    
   motor1.set_target(current_target);
    motor1.start();

    while (!motor1.target_reached()) {
      
      motor1.start();
      
    }
    
    delay(1000); // Wait for 1 second
    motor1.turn_off();
    
    doorStaus = doorWantedStatus;
    count++;

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
int target = 372;
int count = 1;

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