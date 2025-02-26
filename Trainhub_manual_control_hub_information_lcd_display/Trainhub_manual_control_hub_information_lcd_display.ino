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
#include <LiquidCrystal.h>
#define MAX 50
// create a hub instance
Lpf2Hub myTrainHub;
byte port = (byte)PoweredUpHubPort::A;
// char input;
const int rs = 10, en = 11, d4 = 6, d5 = 7, d6 = 8, d7 = 9, buttonPinS = 2, buttonPinU = 4, buttonPinD = 3, buttonPinDC = 5;

int buttonStateS = 0, buttonStateU = 0, buttonStateD = 0, buttonStateDC = 0, flag_connect = 0;

bool isHwVersionAvailable = false, isFwVersionAvailable = false, isBatteryTypeAvailable = false, isInitialized = false;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

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
  Serial.println("Port Value changed");
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
  if (deviceType == DeviceType::UNKNOWNDEVICE) {
    // double current =
    myHub->parsePortMessage(pData);
    Serial.print("Test read: ");
    // Serial.println(pData);
    for (size_t i = 0; i < 5; i++) {
      Serial.println(pData[i]);
    }
    return;

  } else {
    Serial.print("Test read: ");
    for (size_t i = 0; i < 5; i++) {
      Serial.println(pData[i]);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(buttonPinS, INPUT);
  pinMode(buttonPinU, INPUT);
  pinMode(buttonPinD, INPUT);
  pinMode(buttonPinDC, INPUT);


  //Initialise 16*2 LCD
  lcd.begin(16, 2);            //Initialise 16*2 LCD
  lcd.print("LEGO TrainHub");  //Intro Message line 1
  lcd.setCursor(0, 1);
  lcd.print(" -- IfEV --");  //Intro Message line 2
  delay(2000);
  lcd.clear();
  lcd.print("");
  lcd.setCursor(0, 1);
  lcd.print("SPEED:0");
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
      lcd.setCursor(0, 0);
      lcd.print("            ");
      lcd.setCursor(0, 0);
      lcd.print("CONNECTED");

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
      delay(50);
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
    } else {
      delay(50);
      myTrainHub.setBasicMotorSpeed(port, speed);
    }
  } else {
    if (flag_connect == 0) {
      Serial.println("Train hub is disconnected");
      lcd.setCursor(0, 0);
      lcd.print("            ");
      lcd.setCursor(0, 0);
      lcd.print("DISCONNECTED");
      Serial.println(myTrainHub.getModeForDeviceType((byte)DeviceType::TRAIN_MOTOR));
      flag_connect = 1;
    }
  }
  lcd.setCursor(6, 1);
      lcd.print("          ");
      lcd.setCursor(6, 1);
      lcd.print(speed);

}  // End of loop
