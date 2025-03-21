/**
 * A MoveHub basic example to connect a boost hub and try to get the
 * hub device infos like battery level, Rssi and firmware version. Additionally the read out
 * of the hub button with a callback is shown.
 * 
 * (c) Copyright 2020 - Cornelius Munz
 * Released under MIT License
 * 
 */

#include "Lpf2Hub.h"

// create a hub instance
Lpf2Hub myMoveHub11;

bool isHwVersionAvailable = false;
bool isFwVersionAvailable = false;
bool isBatteryTypeAvailable = false;

bool isInitialized = false;



void setup()
{
  Serial.begin(115200);
  while (!Serial);
  myMoveHub1.init(); 
  Serial.println("C1:45:29:1A:CD:26");// initalize the MoveHub instance
}
void handleNotification(uint8_t* data, size_t length) {
//     Serial.print("Received: ");
//     // for (int i = 0; i < length; i++) {
//     //     Serial.print(data[i], HEX);
//     //     Serial.print(" ");
//     // }
//     // Serial.println();
//     uint8_t value = data[0];
// Serial.println("Received direction: " + String(value));
if (length == 4) {
    int32_t value;
    memcpy(&value, data, sizeof(value));  // interpret 4 bytes as int32_t
    Serial.print("Received int32 value: ");
    Serial.println(value);
  }
}
// main loop
void loop()
{

  // connect flow. Search for BLE services and try to connect if the uuid of the hub is found
  if (myMoveHub1.isConnecting())
  {
    myMoveHub1.connectHub();
    if (myMoveHub1.isConnected() && !isInitialized)
    {
      Serial.println("Connected to HUB 2");
      delay(50); //needed because otherwise the message is to fast after the connection procedure and the message will get lost

      // myMoveHub1.activateHubPropertyUpdate(HubPropertyReference::RSSI, hubPropertyChangeCallback);
      // delay(50);
      // myMoveHub1.activatePortDevice((byte)MoveHubPort::TILT, portValueChangeCallback);
      // delay(50);
      // myMoveHub1.activatePortDevice((byte)MoveHubPort::CURRENT, portValueChangeCallback);
      // delay(50);
      // myMoveHub1.activatePortDevice((byte)MoveHubPort::VOLTAGE, portValueChangeCallback);
      myMoveHub1.subscribeToCustomCharacteristic("00001623-1212-efde-1623-785feabcd123", "00001624-1212-efde-1623-785feabcd123", handleNotification);
      isInitialized = true;
    }
    else
    {
      Serial.println("Failed to connect to HUB");
    }
  }


  // if connected, print out continously the hub property values

} // End of loop