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
byte port2 = (byte)PoweredUpHubPort::B;
// char input;
const int rs = 10, en = 11, d4 = 6, d5 = 7, d6 = 8, d7 = 9, buttonPinS = 2, buttonPinU = 4, buttonPinD = 3, buttonPinDC = 5;

int buttonStateS = 0, buttonStateU = 0, buttonStateD = 0, buttonStateDC = 0, flag_connect = 0, flag_button_pressed = 0, sleeperCount = 0, lastColor = -1, lastDistance = -1;

bool isInitialized = false;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// callback function to handle updates of sensor values
void colorDistanceSensorCallback(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData) {
  Lpf2Hub *myTrainHub = (Lpf2Hub *)hub;

  Serial.print("sensorMessage callback for port: ");
  Serial.print(portNumber, DEC);
  Serial.print(" value: ");
  Serial.println(pData[4], DEC);
  if (deviceType == DeviceType::COLOR_DISTANCE_SENSOR) {
    int color = myTrainHub->parseColor(pData);
    double distance = myTrainHub->parseDistance(pData);
    Serial.print("Color: ");
    Serial.print(LegoinoCommon::ColorStringFromColor(color).c_str());
    Serial.print(" Distance: ");
    Serial.println(distance, DEC);
    myTrainHub->setLedColor((Color)color);
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

int speed = 0, speed2 = 0;
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
    if (myTrainHub.isConnected() && !isInitialized) {
      lcd.setCursor(0, 0);
      lcd.print("            ");
      lcd.setCursor(0, 0);
      lcd.print("CONNECTED");

      Serial.println("Connected to HUB");
      Serial.print("Hub address: ");
      Serial.println(myTrainHub.getHubAddress().toString().c_str());
      Serial.print("Hub name: ");
      Serial.println(myTrainHub.getHubName().c_str());

      delay(200);
      Serial.print("check ports... if needed sensor is already connected: ");
      byte portForDevice = myTrainHub.getPortForDeviceType((byte)DeviceType::COLOR_DISTANCE_SENSOR);
      Serial.println(portForDevice, DEC);
      if (portForDevice != 255) {
        Serial.println("activatePortDevice");
        myTrainHub.activatePortDevice(portForDevice, colorDistanceSensorCallback);
        delay(200);
        myTrainHub.setLedColor(GREEN);
        isInitialized = true;
      };
    } else {
      Serial.println("Failed to connect to HUB");
    }

    
  }


  if (myTrainHub.isConnected()) {
    if (digitalRead(buttonPinS) == HIGH) {
      Serial.println("Input is STOP");
      Serial.println("Train halting");
      speed = 0;
      myTrainHub.setLedColor(RED);
      flag_button_pressed = 1;
      delay(200);
    } else if (digitalRead(buttonPinU) == HIGH) {
      Serial.println("Input is Increase Speed by 10");
      speed = speed + 10;
      Serial.print("Current Speed ");
      Serial.println(speed);
      flag_button_pressed = 1;
      delay(300);
      if (speed < 0) {
        myTrainHub.setLedColor(BLUE);
        speed2 = speed - 10;
      } else {
        myTrainHub.setLedColor(GREEN);
        speed2 = speed + 10;
      }
    } else if (digitalRead(buttonPinD) == HIGH) {
      Serial.println("Input is Decrease Speed by 10");
      speed = speed - 10;
      Serial.print("Current Speed ");
      Serial.println(speed);
      flag_button_pressed = 1;
      if (speed < 0) {
        myTrainHub.setLedColor(BLUE);
      } else {
        myTrainHub.setLedColor(GREEN);
      }
      delay(300);
    } else if (digitalRead(buttonPinDC) == HIGH) {
      myTrainHub.shutDownHub();
      flag_button_pressed = 1;
    }

    if (flag_button_pressed == 1) {
      if (speed == 0) {
        delay(50);
        myTrainHub.stopBasicMotor(port);
        Serial.println("Inside button pressed blcok");
      } else {
        delay(50);
        myTrainHub.setBasicMotorSpeed(port, speed);
      }
      lcd.setCursor(6, 1);
      lcd.print("          ");
      lcd.setCursor(6, 1);
      lcd.print(speed);
      flag_button_pressed = 0;
    }


  } else {
      Serial.println("Train hub is disconnected");
      lcd.setCursor(0, 0);
      lcd.print("            ");
      lcd.setCursor(0, 0);
      lcd.print("DISCONNECTED");
  }

}  // End of loop
