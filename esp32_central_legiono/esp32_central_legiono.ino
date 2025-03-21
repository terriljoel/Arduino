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
Lpf2Hub myHub1;
Lpf2Hub myHub2;

bool isHwVersionAvailable = false;
bool isFwVersionAvailable = false;
bool isBatteryTypeAvailable = false;

// bool isInitialized = false;



void setup()
{
  Serial.begin(115200);
  while (!Serial);
  myHub1.init(); 
  Serial.println("C1:45:29:1A:CD:26");// initalize the MoveHub instance
}
void handleNotification(uint8_t* data, size_t length) {
if (length == 4) {
    int32_t value;
    memcpy(&value, data, sizeof(value));  // interpret 4 bytes as int32_t
    Serial.print("Received int32 value: ");
    Serial.println(value);
  }
  Serial.print("Received int32 value: ");
}
// main loop
void loop()
{

  // connect flow. Search for BLE services and try to connect if the uuid of the hub is found
  if (myHub1.isConnecting())
  {
    myHub1.connectHub();
    if (myHub1.isConnected())
    {
      Serial.println("Connected to HUB 1");
      delay(50); //needed because otherwise the message is to fast after the connection procedure and the message will get lost
      //  myHub2.init();
      myHub1.subscribeToCustomCharacteristic("00001623-1212-efde-1623-785feabcd123", "00001624-1212-efde-1623-785feabcd123", handleNotification);
      // isInitialized = true;
      delay(50);
       myHub2.init();
       delay(50);
    }
    else
    {
      Serial.println("Failed to connect to HUB 1");
    }
  }
    if (myHub2.isConnecting())
  {
    myHub2.connectHub();
    if (myHub2.isConnected())
    {
      Serial.println("Connected to HUB 2");
      delay(50); //needed because otherwise the message is to fast after the connection procedure and the message will get lost

      // myHub2.subscribeToCustomCharacteristic("00001623-1212-efde-1623-785feabcd123", "00001624-1212-efde-1623-785feabcd123", handleNotification);
      // isInitialized = true;
    }
    else
    {
      Serial.println("Failed to connect to HUB 2");
    }
  }


  // if connected, print out continously the hub property values

} // End of loop