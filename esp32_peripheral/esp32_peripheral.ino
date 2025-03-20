/*
  BLE_Peripheral.ino

  This program uses the ArduinoBLE library to set-up an Arduino Nano 33 BLE 
  as a peripheral device and specifies a service and a characteristic. Depending 
  of the value of the specified characteristic, an on-board LED gets on. 

  The circuit:
  - Arduino Nano 33 BLE. 

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>
      
enum {
  tilt_NONE  = -1,
  tilt_UP    = 0,
  tilt_DOWN  = 1,
  tilt_LEFT  = 2,
  tilt_RIGHT = 3
};

const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* deviceServiceCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";

int tilt=-1;

BLEService tiltService(deviceServiceUuid); 
BLEIntCharacteristic tiltCharacteristic(deviceServiceCharacteristicUuid, BLERead | BLEWrite);


void setup() {
  Serial.begin(115200);
  while (!Serial);  

  if (!BLE.begin()) {
    Serial.println("- Starting Bluetooth® Low Energy module failed!");
    while (1);
  }

  BLE.setLocalName("Arduino ESP 32 (Peripheral)");
  BLE.setAdvertisedService(tiltService);
  tiltService.addCharacteristic(tiltCharacteristic);
  BLE.addService(tiltService);
  tiltCharacteristic.writeValue(-1);
  BLE.advertise();

  Serial.println("Nano 33 BLE (Peripheral Device)");
  Serial.println(" ");
}

void loop() {
  BLEDevice central = BLE.central();
  Serial.println("- Discovering central device...");
  delay(500);

  if (central) {
    Serial.println("* Connected to central device!");
    Serial.print("* Device MAC address: ");
    Serial.println(central.address());
    Serial.println(" ");

    while (central.connected()) {
      if (tiltCharacteristic.written()) {
         tilt = tiltCharacteristic.value();
        //  writetilt(tilt);
        Serial.println(tilt);
       }
    }
    
    Serial.println("* Disconnected to central device!");
  }
}

// void writetilt(int tilt) {
//   Serial.println("- Characteristic <tilt_type> has changed!");
  
//    switch (tilt) {
//       case tilt_UP:
//         Serial.println("* Actual value: UP (red LED on)");
//         Serial.println(" ");
//         digitalWrite(LEDR, LOW);
//         digitalWrite(LEDG, HIGH);
//         digitalWrite(LEDB, HIGH);
//         digitalWrite(LED_BUILTIN, LOW);
//         break;
//       case tilt_DOWN:
//         Serial.println("* Actual value: DOWN (green LED on)");
//         Serial.println(" ");
//         digitalWrite(LEDR, HIGH);
//         digitalWrite(LEDG, LOW);
//         digitalWrite(LEDB, HIGH);
//         digitalWrite(LED_BUILTIN, LOW);
//         break;
//       case tilt_LEFT:
//         Serial.println("* Actual value: LEFT (blue LED on)");
//         Serial.println(" ");
//         digitalWrite(LEDR, HIGH);
//         digitalWrite(LEDG, HIGH);
//         digitalWrite(LEDB, LOW);
//         digitalWrite(LED_BUILTIN, LOW);
//         break;
//       case tilt_RIGHT:
//         Serial.println("* Actual value: RIGHT (built-in LED on)");
//         Serial.println(" ");
//         digitalWrite(LEDR, HIGH);
//         digitalWrite(LEDG, HIGH);
//         digitalWrite(LEDB, HIGH);
//         digitalWrite(LED_BUILTIN, HIGH);
//         break;
//       default:
//         digitalWrite(LEDR, HIGH);
//         digitalWrite(LEDG, HIGH);
//         digitalWrite(LEDB, HIGH);
//         digitalWrite(LED_BUILTIN, LOW);
//         break;
//     }      
// }