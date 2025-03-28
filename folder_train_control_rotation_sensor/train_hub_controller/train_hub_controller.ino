#include "Lpf2Hub.h"  // Include Lego Ino Library for interacting with LEGO PoweredUp Hub
#include <LiquidCrystal.h>  // Include the library for controlling the LCD screen

// Create instances for two hubs
Lpf2Hub myHub1;  // Hub instance for Arduino BLE Nano 33
Lpf2Hub myHub2;  // Hub instance for Train Hub (second hub)
byte port = (byte)PoweredUpHubPort::A;  // Define the motor port for communication with the motor

// Pin assignments for buttons and LCD
const int rs = 10, en = 11, d4 = 6, d5 = 7, d6 = 8, d7 = 9;  // LCD control pins
const int buttonPinS = 2, buttonPinU = 4, buttonPinD = 3, buttonPinDC = 5;  // Button pins for controlling the train
/*
buttonPinS is for Stop command and configured with digital pin 2 of the board
buttonPinU is to  Increase motor speed by 10  command and configured with digital pin 4 of the board
buttonPinD is  to decrease motor speed by 10 and configured with digital pin 3 of the board
buttonPinDC is for Dsconnection and connection and configured with digital pin 5 of the board
*/
// Variables for button states and connection flags
int buttonStateS = 0, buttonStateU = 0, buttonStateD = 0, buttonStateDC = 0;  

bool isInitialized = false;  // Flag to track initialization state of hubs

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);  // Create LCD object

// Volatile variables for speed, distance, and RPM (updated by notifications)
volatile float speed = 0, distance = 0, rpm = 0;  
char hubName[] = "IfEV";  // Hub name for identification (could be used for display or debugging)
int motorSpeed = 0;  // Motor speed variable (ranges from 0 to maximum speed)

// Notification handler for distance data received from the hub
void handleNotification(uint8_t* data, size_t length) {
  if (length == 4) {  // Expecting 4 bytes of data
    float value;
    memcpy(&value, data, sizeof(value));  // Interpret 4 bytes as a float
    Serial.print("Received distance: ");
    Serial.println(value);
    distance = value;  // Store the received distance value
  }
}

// Notification handler for speed data received from the hub
void handleNotification2(uint8_t* data, size_t length) {
  if (length == 4) {  // Expecting 4 bytes of data
    float value;
    memcpy(&value, data, sizeof(value));  // Interpret 4 bytes as a float
    Serial.print("Received speed: ");
    Serial.println(value);
    speed = value;  // Store the received speed value
  }
}

// Notification handler for RPM data received from the hub
void handleNotification3(uint8_t* data, size_t length) {
  if (length == 4) {  // Expecting 4 bytes of data
    float value;
    memcpy(&value, data, sizeof(value));  // Interpret 4 bytes as a float
    Serial.print("Received RPM: ");
    Serial.println(value);
    rpm = value;  // Store the received RPM value
  }
}

// Setup function to initialize serial communication, LCD, and hub connections
void setup() {
  Serial.begin(115200);  // Start serial communication at 115200 baud rate
   // while (!Serial);  // Unless and until the board is not connected to the serial the program will not proceed. This code is commented out as it is not required.
  myHub1.init();  // Initialize the first hub (BLE Nano 33)
  Serial.println("- Setting Up .....");

  // Set button pins as input
  pinMode(buttonPinS, INPUT); 
  pinMode(buttonPinU, INPUT);
  pinMode(buttonPinD, INPUT);
  pinMode(buttonPinDC, INPUT);

  // Initialize 16x2 LCD
  lcd.begin(16, 2);  // Set LCD to 16 columns and 2 rows
  lcd.print("LEGO TrainHub");  // Display introductory message
  lcd.setCursor(0, 1);  // Move cursor to second line
  lcd.print(" -- IfEV --");  // Display the hub name
  delay(2000);  // Wait for 2 seconds to show introductory message
  lcd.clear();  // Clear the LCD screen
  lcd.print("");  // Clear any leftover text
  lcd.print("D1:D D2:D S:0");  // Display motor port status and speed
  lcd.setCursor(0, 1);
  lcd.print("D:0     R:0");  // Display initial distance and RPM values
}

// Main loop where all the logic is handled continuously
void loop() {
  // Attempt to connect to Hub 1 (Arduino nano ble 33) if not connected and initialization is not done
  if (myHub1.isConnecting() && !isInitialized) {
    myHub1.connectHub();  // Try to connect Hub 1 (Arduino nano ble 33)
    if (myHub1.isConnected()) {  // If connection is successful
      Serial.println("Connected to Hub 1 (Arduino nano ble 33)");
      lcd.setCursor(3, 0);  // Update LCD to show "C" for connection
      lcd.print(" ");
      lcd.setCursor(3, 0);
      lcd.print("C");
      Serial.print("Hub address: ");
      Serial.print(myHub1.getHubAddress().toString().c_str());
      Serial.print("  Hub name: ");
      Serial.println(myHub1.getHubName().c_str());
      delay(50);  // Small delay to ensure the message is readable
      myHub1.subscribeToCustomCharacteristic("00001623-1212-efde-1623-785feabcd123", "00001624-1212-efde-1623-785feabcd123", handleNotification);  // Subscribe to custom characteristics for distance
      myHub1.subscribeToCustomCharacteristic("00001623-1212-efde-1623-785feabcd123", "00001625-1212-efde-1623-785feabcd123", handleNotification2);  // Subscribe for speed
      myHub1.subscribeToCustomCharacteristic("00001623-1212-efde-1623-785feabcd123", "00001626-1212-efde-1623-785feabcd123", handleNotification3);  // Subscribe for RPM
      isInitialized = true;  // Mark initialization as complete
      delay(50);
      myHub2.init();  // Initialize Hub 2 (Train Hub)
      delay(50);
    } else {
      Serial.println("Failed to connect to Hub 1 (Arduino nano ble 33)");
    }
  }

  // Similar connection logic for Hub 2 (Train Hub)
  if (myHub2.isConnecting()) {
    myHub2.connectHub();
    if (myHub2.isConnected()) {
      Serial.println("Connected to HUB 2");
      lcd.setCursor(8, 0);  // Update LCD to show "C" for Hub 2 connection
      lcd.print(" ");
      lcd.setCursor(8, 0);
      lcd.print("C");
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

  // Handle button presses for controlling the train's motor
  buttonStateDC = digitalRead(buttonPinDC);  // Read the state of the Disconnect button
  if (buttonStateDC == HIGH && !myHub1.isConnected() && !myHub2.isConnected()) {
    isInitialized = false;  // Reset initialization if disconnect button is pressed
    myHub1.init();  // Reinitialize Hub 1 (Arduino nano ble 33)
  }

  // If both hubs are connected, read button states and control motor speed
  if (myHub1.isConnected() && myHub2.isConnected()) {
    buttonStateS = digitalRead(buttonPinS);  // Read the Stop button state
    buttonStateU = digitalRead(buttonPinU);  // Read the Increase Speed button state
    buttonStateD = digitalRead(buttonPinD);  // Read the Decrease Speed button state

    // Control logic for motor based on button presses
    if (buttonStateS == HIGH) {
      Serial.println("Input is STOP");
      Serial.println("Train halting");
      motorSpeed = 0;  // Stop the motor
      buttonStateS = 0;  // Reset button state
    } else if (buttonStateU == HIGH) {
      Serial.println("Input is Increase Speed by 10");
      motorSpeed += 10;  // Increase speed by 10
      Serial.print("Current Speed ");
      Serial.println(motorSpeed);
      buttonStateU = 0;  // Reset button state
      delay(300);  // Small delay to prevent multiple triggers
    } else if (buttonStateD == HIGH) {
      Serial.println("Input is Decrease Speed by 10");
      motorSpeed -= 10;  // Decrease speed by 10
      Serial.print("Current Speed ");
      Serial.println(motorSpeed);
      buttonStateD = 0;  // Reset button state
      delay(300);
    } else if (buttonStateDC == HIGH) {
      Serial.println("**** Hubs are disconnected");
      myHub1.disconnectFromHub();  // Disconnect Hub 1 (Arduino nano ble 33)
      delay(500);  // Short delay
      myHub2.shutDownHub();  // Shut down Hub 2
      motorSpeed = 0;  // Stop motor when hubs are disconnected
    }

    // Set motor speed or stop motor based on the motorSpeed variable
    if (motorSpeed == 0) {
      delay(50);
      myHub2.stopBasicMotor(port);  // Stop the motor
    } else {
      delay(50);
      myHub2.setBasicMotorSpeed(port, motorSpeed);  // Set the motor speed
    }

    // Update LCD with speed, distance, and RPM
    lcd.setCursor(12, 0);
    lcd.print("   ");
    lcd.setCursor(12, 0);
    lcd.print(speed, 1);  // Display speed on LCD

    lcd.setCursor(2, 1);
    lcd.print("     ");
    lcd.setCursor(2, 1);
    lcd.print(distance, 1);  // Display distance on LCD
    lcd.setCursor(10, 1);
    lcd.print("      ");
    lcd.setCursor(10, 1);
    lcd.print(rpm, 1);  // Display RPM on LCD
  } 

  // Display disconnected state on LCD if any hub is not connected
  if (!myHub1.isConnected()) {
    lcd.setCursor(3, 0);
    lcd.print(" ");
    lcd.setCursor(3, 0);
    lcd.print("D");
    delay(1000);  // Wait for 1 second before checking again
  }

  if  (!myHub2.isConnected()){
    lcd.setCursor(8, 0);
    lcd.print(" ");
    lcd.setCursor(8, 0);
    lcd.print("D");
    delay(1000);  // Wait for 1 second before checking again
  }
  if(!myHub2.isConnected() && !myHub1.isConnected()){
    Serial.println("* Hubs are disconnected");
  }
}
