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
int target;

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
    Serial.print(doorCharacteristic.value()[0]);
    Serial.print(";");
    Serial.println(doorCharacteristic.value()[1]);
    Serial.println("doorNbTurn");
      interrupts();
    doorNbTurn = doorCharacteristic.value()[0] / 10;
    doorWantedStatus = doorCharacteristic.value()[1];
    
    if (doorWantedStatus == 0 && doorStaus != doorWantedStatus ) {
      target = (int) oneTurn * doorNbTurn;
      noInterrupts();
      
      Serial.println(target); // * doorNbTurn);
      interrupts();
      
      motor1.set_target(target); // * doorNbTurn);
    } else if (doorWantedStatus == 1 && doorStaus != doorWantedStatus) {
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
    
    doorStaus = doorWantedStatus;


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
