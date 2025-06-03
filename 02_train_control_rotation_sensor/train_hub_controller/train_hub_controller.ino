#include "Lpf2Hub.h"        // Include Lego Ino Library for interacting with LEGO PoweredUp Hub
#include <LiquidCrystal.h>  // Include the library for controlling the LCD screen
#include <NimBLEDevice.h>

// Create a pointer to a NimBLEClient object
NimBLEClient* pClient = nullptr;

// Create instances for two hubs
Lpf2Hub myHub1;                         // Hub instance for Arduino BLE Nano 33
Lpf2Hub myHub2;                         // Hub instance for Train Hub (second hub)
byte port = (byte)PoweredUpHubPort::A;  // Define the motor port for communication with the motor

// Pin assignments for buttons and LCD
const int rs = 10, en = 11, d4 = 6, d5 = 7, d6 = 8, d7 = 9;                 // LCD control pins
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
int motorSpeed = 0;       // Motor speed variable (ranges from 0 to maximum speed)

#include <NimBLEDevice.h>

/**
 * @brief Subscribe to a custom characteristic of a connected BLE device.
 *
 * This function subscribes to a specified characteristic on a BLE device and sets up a callback to
 * handle notifications from that characteristic. It checks if the device is connected and if the 
 * characteristic supports notifications before subscribing.
 *
 * @param pClient Pointer to the NimBLEClient object representing the connected BLE device.
 * @param serviceUUID UUID of the service containing the characteristic.
 * @param charUUID UUID of the characteristic to subscribe to.
 * @param callback Function to handle the data received from notifications.
 */
void subscribeToCharacteristic(NimBLEClient* pClient, const char* serviceUUID, const char* charUUID, std::function<void(uint8_t*, size_t)> callback) {
  // Check if the client is connected
  if (!pClient || !pClient->isConnected()) {
    Serial.println("BLE client not connected.");
    return; // Exit if the client is not connected
  }

  // Get the service from the connected BLE device using the service UUID
  NimBLERemoteService* pService = pClient->getService(serviceUUID);
  if (!pService) {
    Serial.print("Service UUID not found: ");
    Serial.println(serviceUUID);
    return; // Exit if the service is not found
  }

  // Get the characteristic from the service using the characteristic UUID
  NimBLERemoteCharacteristic* pChar = pService->getCharacteristic(charUUID);
  if (!pChar) {
    Serial.print("Characteristic UUID not found: ");
    Serial.println(charUUID);
    return; // Exit if the characteristic is not found
  }

  // Check if the characteristic supports notifications
  if (!pChar->canNotify()) {
    Serial.println("Characteristic does not support notifications.");
    return; // Exit if notifications are not supported
  }

  // Subscribe to the characteristic and set the callback function for notifications
  bool result = pChar->subscribe(true, [callback](NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    callback(pData, length); // Call the callback function with the received data
  });

  // Log whether the subscription was successful or not
  if (result) {
    Serial.print("Subscribed to ");
    Serial.println(charUUID);
  } else {
    Serial.print("Failed to subscribe to ");
    Serial.println(charUUID);
  }
}

/**
 * @brief Disconnect from a connected BLE device.
 *
 * This function disconnects from the currently connected BLE device and provides feedback 
 * on the disconnection status. It checks if the client is connected before attempting to disconnect.
 *
 * @param pClient Pointer to the NimBLEClient object representing the connected BLE device.
 */
void disconnectFromHub(NimBLEClient* pClient) {
  // Check if the client is connected
  if (!pClient || !pClient->isConnected()) {
    Serial.println("Not connected to any BLE device.");
    return; // Exit if the client is not connected
  }

  // Disconnect from the BLE device
  Serial.println("Disconnecting from BLE device...");
  pClient->disconnect();
  delay(500);  // Allow time for disconnection to complete

  // Log the disconnection status
  Serial.println("Successfully disconnected from hub.");
}


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
  myHub1.init();         // Initialize the first hub (BLE Nano 33)
  Serial.println("- Setting Up .....");

  // Set button pins as input
  pinMode(buttonPinS, INPUT);
  pinMode(buttonPinU, INPUT);
  pinMode(buttonPinD, INPUT);
  pinMode(buttonPinDC, INPUT);

  // Initialize 16x2 LCD
  lcd.begin(16, 2);            // Set LCD to 16 columns and 2 rows
  lcd.print("LEGO TrainHub");  // Display introductory message
  lcd.setCursor(0, 1);         // Move cursor to second line
  lcd.print(" -- IfEV --");    // Display the hub name
  delay(2000);                 // Wait for 2 seconds to show introductory message
  lcd.clear();                 // Clear the LCD screen
  lcd.print("");               // Clear any leftover text
  lcd.print("D1:D D2:D S:0");  // Display motor port status and speed
  lcd.setCursor(0, 1);
  lcd.print("D:0     R:0");  // Display initial distance and RPM values
}

// Main loop where all the logic is handled continuously
void loop() {
  // Attempt to connect to Hub 1 (Arduino nano ble 33) if not connected and initialization is not done
  if (myHub1.isConnecting() && !isInitialized) {
    myHub1.connectHub();         // Try to connect Hub 1 (Arduino nano ble 33)
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
      Serial.println("Connected to HUB 1");

      pClient = NimBLEDevice::getClientByPeerAddress(myHub1.getHubAddress());
      if (pClient) {
        subscribeToCharacteristic(pClient, "00001623-1212-efde-1623-785feabcd123", "00001624-1212-efde-1623-785feabcd123", handleNotification);
        subscribeToCharacteristic(pClient, "00001623-1212-efde-1623-785feabcd123", "00001625-1212-efde-1623-785feabcd123", handleNotification2);
        subscribeToCharacteristic(pClient, "00001623-1212-efde-1623-785feabcd123", "00001626-1212-efde-1623-785feabcd123", handleNotification3);
      } else {
        Serial.println("Failed to get BLE client.");
      }

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
    myHub1.init();          // Reinitialize Hub 1 (Arduino nano ble 33)
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
      motorSpeed = 0;    // Stop the motor
      buttonStateS = 0;  // Reset button state
    } else if (buttonStateU == HIGH) {
      Serial.println("Input is Increase Speed by 10");
      motorSpeed += 10;  // Increase speed by 10
      Serial.print("Current Speed ");
      Serial.println(motorSpeed);
      buttonStateU = 0;  // Reset button state
      delay(300);        // Small delay to prevent multiple triggers
    } else if (buttonStateD == HIGH) {
      Serial.println("Input is Decrease Speed by 10");
      motorSpeed -= 10;  // Decrease speed by 10
      Serial.print("Current Speed ");
      Serial.println(motorSpeed);
      buttonStateD = 0;  // Reset button state
      delay(300);
    } else if (buttonStateDC == HIGH) {
      Serial.println("**** Hubs are disconnected");
      disconnectFromHub(pClient);// Disconnect Hub 1 (Arduino nano ble 33)
      delay(100);            // Short delay
      myHub2.shutDownHub();  // Shut down Hub 2
      motorSpeed = 0;        // Stop motor when hubs are disconnected
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

  if (!myHub2.isConnected()) {
    lcd.setCursor(8, 0);
    lcd.print(" ");
    lcd.setCursor(8, 0);
    lcd.print("D");
    delay(1000);  // Wait for 1 second before checking again
  }
  if (!myHub2.isConnected() && !myHub1.isConnected()) {
    Serial.println("* Hubs are disconnected");
  }
}
