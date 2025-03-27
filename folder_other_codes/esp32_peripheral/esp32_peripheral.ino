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
BLEDevice central;
bool initialized=false;    
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



// #include "Lpf2Hub.h"
// Lpf2Hub myTrainHub;
// byte port = (byte)PoweredUpHubPort::A;


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
// BLE.setConnectionInterval(100); 
  Serial.println("Nano 33 BLE (Peripheral Device)");
  Serial.println(" ");
}

void loop() {
  central = BLE.central();
  Serial.println("- Discovering central device...");
  delay(500);

  // if (central && !initialized) {
  //   Serial.println("* Connected to central device!");
  //   Serial.print("* Device MAC address: ");
  //   Serial.println(central.address());
  //   Serial.println(" ");
  //   initialized=true;
  // }
  //   if (central.connected()) {
  //     if (tiltCharacteristic.written()) {
  //        tilt = tiltCharacteristic.value();
  //       //  writetilt(tilt);
  //       Serial.println(tilt);
  //      }
    
  //   delay(1000);
  //   tiltCharacteristic.writeValue((int32_t)1000);
  //   Serial.println("* Disconnected to central device!");
  // }

 if (central) {
    Serial.println("* Connected to central device!");
    Serial.print("* Device MAC address: ");
    Serial.println(central.address());
    Serial.println(" ");

    while (central.connected()) {
      if (tiltCharacteristic.written()) {
         tilt = tiltCharacteristic.value();
         Serial.println(tilt);
       }
       delay(1000);
    tiltCharacteristic.writeValue((int32_t)1000);
    }
    
    Serial.println("* Disconnected to central device!");
  }

//   if (!central && !myTrainHub.isConnected() && !myTrainHub.isConnecting()) {
//     myTrainHub.init(); 
//     Serial.println("Trying to connect"); 
//   }

//   // connect flow. Search for BLE services and try to/ connect if the uuid of the hub is found
//   if (myTrainHub.isConnecting()) {
//     myTrainHub.connectHub();
//     if (myTrainHub.isConnected()) {
//       Serial.println("Connected to HUB");
//       Serial.print("Hub address: ");
//       Serial.println(myTrainHub.getHubAddress().toString().c_str());
//       Serial.print("Hub name: ");
//       Serial.println(myTrainHub.getHubName().c_str());

//       // myTrainHub.activateHubPropertyUpdate(HubPropertyReference::ADVERTISING_NAME, hubPropertyChangeCallback);
//       // delay(50);
//       // myTrainHub.activateHubPropertyUpdate(HubPropertyReference::BATTERY_VOLTAGE, hubPropertyChangeCallback);
//       // delay(50);
//       // myTrainHub.activateHubPropertyUpdate(HubPropertyReference::BUTTON, hubPropertyChangeCallback);
//       // delay(50);
//       // myTrainHub.activateHubPropertyUpdate(HubPropertyReference::RSSI, hubPropertyChangeCallback);
//       // delay(50);
//       // // myTrainHub.activatePortDevice((byte)MoveHubPort::TILT, portValueChangeCallback);
//       // // delay(50);
//       // myTrainHub.activatePortDevice((byte)PoweredUpHubPort::A, portValueChangeCallback);
//       // delay(50);
//       // // myTrainHub.activatePortDevice((byte)PoweredUpHubPort::VOLTAGE, portValueChangeCallback);
//       // isInitialized = true;

//     } else {
//       Serial.println("Failed to connect to HUB");
//     }
// }
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