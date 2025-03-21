#include "Lpf2Hub.h"  // Lego Ino Library
#include <LiquidCrystal.h>

// create a hub instance
Lpf2Hub myHub1;  // hub instance for arduino ble nano 33
Lpf2Hub myHub2;  // hub instance for train hub
byte port = (byte)PoweredUpHubPort::A;

const int rs = 10, en = 11, d4 = 6, d5 = 7, d6 = 8, d7 = 9, buttonPinS = 2, buttonPinU = 4, buttonPinD = 3, buttonPinDC = 5;

int buttonStateS = 0, buttonStateU = 0, buttonStateD = 0, buttonStateDC = 0, flag_connect = 0;

bool isInitialized = false;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

volatile float speed = 0, distance=0, rpm=0;
char hubName[] = "IfEV";
int motorSpeed=0;

void handleNotification(uint8_t* data, size_t length) {
  if (length == 4) {
    float value;
    memcpy(&value, data, sizeof(value));  // interpret 4 bytes as int32_t
    Serial.print("Received int32 value: ");
    Serial.println(value);
    distance = value;
  }
}

void handleNotification2(uint8_t* data, size_t length) {
  if (length == 4) {
    float value;
    memcpy(&value, data, sizeof(value));  // interpret 4 bytes as int32_t
    Serial.print("Received int32 value: ");
    Serial.println(value);
    speed = value;
  }
}

void handleNotification3(uint8_t* data, size_t length) {
  if (length == 4) {
    float value;
    memcpy(&value, data, sizeof(value));  // interpret 4 bytes as int32_t
    Serial.print("Received int32 value: ");
    Serial.println(value);
    rpm = value;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  myHub1.init();
  Serial.println("- Setting Up .....");

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
  lcd.print("D1:DIS D2:DIS");
  lcd.setCursor(0, 1);
  lcd.print("D:0   S:0   R:0 ");
}

void loop() {
  if (myHub1.isConnecting()) {
    myHub1.connectHub();
    if (myHub1.isConnected()) {
      Serial.println("Connected to HUB 1");
      lcd.setCursor(3, 0);
      lcd.print("   ");
      lcd.setCursor(3, 0);
      lcd.print("CON");
      Serial.print("Hub address: ");
      Serial.print(myHub1.getHubAddress().toString().c_str());
      Serial.print("  Hub name: ");
      Serial.println(myHub1.getHubName().c_str());
      delay(50);  //needed because otherwise the message is to fast after the connection procedure and the message will get lost
      //  myHub2.init();
      myHub1.subscribeToCustomCharacteristic("00001623-1212-efde-1623-785feabcd123", "00001624-1212-efde-1623-785feabcd123", handleNotification);
      myHub1.subscribeToCustomCharacteristic("00001623-1212-efde-1623-785feabcd123", "00001625-1212-efde-1623-785feabcd123", handleNotification2);
      myHub1.subscribeToCustomCharacteristic("00001623-1212-efde-1623-785feabcd123", "00001626-1212-efde-1623-785feabcd123", handleNotification3);
      Serial.print("success");
      // isInitialized = true;
      delay(50);
      myHub2.init();
      delay(50);
    } else {
      Serial.println("Failed to connect to HUB 1");
    }
  }
  if (myHub2.isConnecting()) {
    myHub2.connectHub();
    if (myHub2.isConnected()) {
      Serial.println("Connected to HUB 2");

      lcd.setCursor(10, 0);
      lcd.print("   ");
      lcd.setCursor(10, 0);
      lcd.print("CON");
      Serial.println("");
      Serial.print("Hub address: ");
      Serial.print(myHub2.getHubAddress().toString().c_str());
      Serial.print("  Hub name: ");
      Serial.println(myHub2.getHubName().c_str());

      delay(50);
    } else {
      Serial.println("Failed to connect to HUB 2");
    }
  }
  if (myHub1.isConnected() && myHub2.isConnected()) {
    buttonStateS = digitalRead(buttonPinS);
    buttonStateU = digitalRead(buttonPinU);
    buttonStateD = digitalRead(buttonPinD);
    buttonStateDC = digitalRead(buttonPinDC);

    if (buttonStateS == HIGH) {
      Serial.println("Input is STOP");
      Serial.println("Train halting");
      motorSpeed = 0;
      buttonStateS = 0;
    } else if (buttonStateU == HIGH) {
      Serial.println("Input is Increase Speed by 10");
      motorSpeed = motorSpeed + 10;
      Serial.print("Current Speed ");
      Serial.println(motorSpeed);
      buttonStateU = 0;
      delay(300);
    } else if (buttonStateD == HIGH) {
      Serial.println("Input is Decrease Speed by 10");
      motorSpeed = motorSpeed - 10;
      Serial.print("Current Speed ");
      Serial.println(motorSpeed);
      buttonStateD = 0;
      delay(300);
    } else if (buttonStateDC == HIGH) {
      Serial.println("**** Hubs are disconnected");
        myHub1.shutDownHub();
        myHub2.shutDownHub();
        motorSpeed=0;
        lcd.setCursor(10, 0);
        lcd.print("   ");
        lcd.setCursor(10, 0);
        lcd.print("DIS");
        lcd.setCursor(3, 0);
        lcd.print("   ");
        lcd.setCursor(3, 0);
        lcd.print("DIS");
      
    }
    if (motorSpeed == 0) {
      delay(50);
      myHub2.stopBasicMotor(port);
    } else {
      delay(50);
      myHub2.setBasicMotorSpeed(port, motorSpeed);
    }
    lcd.setCursor(8, 1);
        lcd.print("   ");
        lcd.setCursor(8, 1);
        lcd.print(speed,1);

        lcd.setCursor(2, 1);
        lcd.print("   ");
        lcd.setCursor(2, 1);
        lcd.print(distance,1);
        lcd.setCursor(14, 1);
        lcd.print("   ");
        lcd.setCursor(14, 1);
        lcd.print(rpm,1);
  }
  // else{
  //   Serial.println("Hubs are disconnected");
  // }
}
