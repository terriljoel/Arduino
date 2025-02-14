/**
 * A Legoino example to control a train which has a motor connected
 * to the Port A of the Hub
 * 
 * (c) Copyright 2020 - Cornelius Munz
 * Released under MIT License
 * 
 */

#include "Lpf2Hub.h"
#include <stdio.h>
#define MAX 50
// create a hub instance
Lpf2Hub myTrainHub;
byte port = (byte)PoweredUpHubPort::A;
char input;
int flag_connect = 0;
const int buttonPinS = 2;
const int buttonPinU = 4;
const int buttonPinD = 3;
const int buttonPinDC = 5;

int buttonStateS = 0;
int buttonStateU = 0;
int buttonStateD = 0;
int buttonStateDC = 0;

bool isHwVersionAvailable = false;
bool isFwVersionAvailable = false;
bool isBatteryTypeAvailable = false;

bool isInitialized = false;
void hubPropertyChangeCallback(void *hub, HubPropertyReference hubProperty, uint8_t *pData) {
  Lpf2Hub *myHub = (Lpf2Hub *)hub;

  Serial.print("HubProperty: ");
  Serial.println((byte)hubProperty, HEX);

  if (hubProperty == HubPropertyReference::RSSI) {
    Serial.print("RSSI: ");
    Serial.println(myHub->parseRssi(pData), DEC);
    return;
  }

  if (hubProperty == HubPropertyReference::ADVERTISING_NAME) {
    Serial.print("Advertising Name: ");
    Serial.println(myHub->parseHubAdvertisingName(pData).c_str());
    return;
  }

  if (hubProperty == HubPropertyReference::BATTERY_VOLTAGE) {
    Serial.print("BatteryLevel: ");
    Serial.println(myHub->parseBatteryLevel(pData), DEC);
    return;
  }

  if (hubProperty == HubPropertyReference::BUTTON) {
    Serial.print("Button: ");
    Serial.println((byte)myHub->parseHubButton(pData), HEX);
    return;
  }

  if (hubProperty == HubPropertyReference::BATTERY_TYPE) {
    Serial.print("BatteryType: ");
    Serial.println(myHub->parseBatteryType(pData), HEX);
    isBatteryTypeAvailable = true;
    return;
  }

  if (hubProperty == HubPropertyReference::FW_VERSION) {
    Version version = myHub->parseVersion(pData);
    Serial.print("FWVersion: ");
    Serial.print(version.Major);
    Serial.print("-");
    Serial.print(version.Minor);
    Serial.print("-");
    Serial.print(version.Bugfix);
    Serial.print(" Build: ");
    Serial.println(version.Build);
    isFwVersionAvailable = true;
    return;
  }

  if (hubProperty == HubPropertyReference::HW_VERSION) {
    Version version = myHub->parseVersion(pData);
    Serial.print("HWVersion: ");
    Serial.print(version.Major);
    Serial.print("-");
    Serial.print(version.Minor);
    Serial.print("-");
    Serial.print(version.Bugfix);
    Serial.print(" Build: ");
    Serial.println(version.Build);
    isHwVersionAvailable = true;
    return;
  }
}

void portValueChangeCallback(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData) {
  Lpf2Hub *myHub = (Lpf2Hub *)hub;

  if (deviceType == DeviceType::VOLTAGE_SENSOR) {
    double voltage = myHub->parseVoltageSensor(pData);
    Serial.print("Voltage: ");
    Serial.println(voltage, 2);
    return;
  }

  if (deviceType == DeviceType::TRAIN_MOTOR) {
    // char current[MAX]= myHub->parsePortMessage(pData);
    Serial.print("test: Hello world ");
    // Serial.println(myHub->parseSpeedometer(pData));
    return;
  }

  if (deviceType == DeviceType::MOVE_HUB_TILT_SENSOR) {
    int x = myHub->parseBoostTiltSensorX(pData);
    int y = myHub->parseBoostTiltSensorY(pData);
    Serial.print("Tilt X: ");
    Serial.print(x, DEC);
    Serial.print(" Y: ");
    Serial.println(y, DEC);
  }
}



void setup() {
  Serial.begin(115200);
  pinMode(buttonPinS, INPUT);
  pinMode(buttonPinU, INPUT);
  pinMode(buttonPinD, INPUT);
  pinMode(buttonPinDC, INPUT);
}

int speed = 0;
char hubName[] = "IfEV";
// main loop
void loop() {

  if (!myTrainHub.isConnected() && !myTrainHub.isConnecting()) {
    myTrainHub.init();  // initalize the PoweredUpHub instance"9c:9a:c0:06:d1:80"
    //myTrainHub.init("90:84:2b:03:19:7f"); //example of initializing an hub with a specific address
  }

  // connect flow. Search for BLE services and try to connect if the uuid of the hub is found
  if (myTrainHub.isConnecting()) {
    myTrainHub.connectHub();
    if (myTrainHub.isConnected()) {
      Serial.println("Connected to HUB");
      Serial.print("Hub address: ");
      Serial.println(myTrainHub.getHubAddress().toString().c_str());
      Serial.print("Hub name: ");
      Serial.println(myTrainHub.getHubName().c_str());

      myTrainHub.activateHubPropertyUpdate(HubPropertyReference::ADVERTISING_NAME, hubPropertyChangeCallback);
      delay(50);
      myTrainHub.activateHubPropertyUpdate(HubPropertyReference::BATTERY_VOLTAGE, hubPropertyChangeCallback);
      delay(50);
      myTrainHub.activateHubPropertyUpdate(HubPropertyReference::BUTTON, hubPropertyChangeCallback);
      delay(50);
      myTrainHub.activateHubPropertyUpdate(HubPropertyReference::RSSI, hubPropertyChangeCallback);
      delay(50);
      // myTrainHub.activatePortDevice((byte)MoveHubPort::TILT, portValueChangeCallback);
      // delay(50);
      myTrainHub.activatePortDevice((byte)PoweredUpHubPort::A, portValueChangeCallback);
      // delay(50);
      // myTrainHub.activatePortDevice((byte)PoweredUpHubPort::VOLTAGE, portValueChangeCallback);
      isInitialized = true;

    } else {
      Serial.println("Failed to connect to HUB");
    }
  }
  buttonStateS = digitalRead(buttonPinS);
  buttonStateU = digitalRead(buttonPinU);
  buttonStateD = digitalRead(buttonPinD);
  buttonStateDC = digitalRead(buttonPinDC);
  if (buttonStateS == HIGH) {
    Serial.println("Input is STOP");
    Serial.println("Train halting");
    speed = 0;
    buttonStateS = 0;
  } else if (buttonStateU == HIGH) {
    Serial.println("Input is Increase Speed by 10");
    speed = speed + 10;
    Serial.print("Current Speed ");
    Serial.println(speed);
    buttonStateU = 0;
    delay(300);
  } else if (buttonStateD == HIGH) {
    Serial.println("Input is Decrease Speed by 10");
    speed = speed - 10;
    Serial.print("Current Speed ");
    Serial.println(speed);
    buttonStateD = 0;
    delay(300);
  } else if (buttonStateDC == HIGH) {
    if (flag_connect == 1) {
      flag_connect = 0;
    } else {
      myTrainHub.shutDownHub();
    }
  }

  //   if (Serial.available() > 0) {
  //   input = Serial.read();
  //   Serial.println("Input is");
  //   Serial.println(input);
  //   if (input == 'f') {
  //     speed = speed + 10;
  //   } else if (input == 's') {
  //     speed = speed - 10;
  //   } else if (input == 'h') {
  //     Serial.println("Train halting");
  //     speed = 0;
  //   }
  //   Serial.print("Current Speed ");
  //   Serial.println(speed);
  // }
  // if connected, you can set the name of the hub, the led color and shut it down
  if (myTrainHub.isConnected()) {

    if (strcmp(myTrainHub.getHubName().c_str(), hubName) != 0) {
      myTrainHub.setHubName(hubName);
      Serial.print("Hub name: ");
      Serial.println(myTrainHub.getHubName().c_str());
    }
    //     if(speed>0){
    // myTrainHub.setLedColor(GREEN);
    //     }
    //     else if(speed<0){
    //       myTrainHub.setLedColor(BLUE);
    //     }
    //     else
    //     {
    //       myTrainHub.setLedColor(RED);
    //     }
    if (speed == 0) {
      delay(50);
      myTrainHub.stopBasicMotor(port);
      // Serial.println("Train has stopped");
      // if (input == 'h') {
      //   myTrainHub.shutDownHub();
      // }
    } else {
      delay(50);
      myTrainHub.setBasicMotorSpeed(port, speed);
    }
    // if()
    //
    // // delay(1000);
    // myTrainHub.setBasicMotorSpeed(port, -15);
    // // delay(10000);
    // myTrainHub.stopBasicMotor(port);
    // // delay(1000);
  } else {
    if (flag_connect == 0) {
      Serial.println("Train hub is disconnected");
      flag_connect = 1;
    }
  }

}  // End of loop
