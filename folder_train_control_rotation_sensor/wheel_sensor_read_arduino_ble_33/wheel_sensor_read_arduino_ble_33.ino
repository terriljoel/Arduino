#include <ArduinoBLE.h>  // Include the Arduino BLE library for Bluetooth Low Energy communication

// Define and initialize variables for wheel rotation and speed calculations
int right_intr = 0;  // right_intr will count the wheel rotations
float radius_of_wheel = 0.00825; // Radius of the wheel in centimeters, used to calculate velocity and distance
volatile byte rotation;          // Variable to track the number of wheel rotations, marked as volatile because it is updated in an interrupt handler
float timetaken, rpm, dtime, v, distance;  // Variables for time taken per rotation, RPM, velocity, and total distance
unsigned long pevtime;          // Previous time of wheel rotation for RPM calculation
int flag = 1;                    // Flag to control the display and data transmission logic
const int no_of_holes = 2;      // The number of holes on the wheel for the sensor to detect rotations
//The above value needs to be changed depending the on the number of holes in the wheel 
int rotation_sensor_changes = 2 * no_of_holes; // Number of sensor changes per full rotation of the wheel

// Bluetooth Low Energy service and characteristic UUIDs
const char *deviceServiceUuid = "00001623-1212-efde-1623-785feabcd123";
const char *deviceServiceCharacteristicUuid1 = "00001624-1212-efde-1623-785feabcd123";
const char *deviceServiceCharacteristicUuid2 = "00001625-1212-efde-1623-785feabcd123";
const char *deviceServiceCharacteristicUuid3 = "00001626-1212-efde-1623-785feabcd123";

// Define the BLE service and characteristics
BLEService wheelRotationService(deviceServiceUuid);  // BLE service for wheel rotation data

// BLE Characteristics for Distance, Speed, and RPM with read and notify properties
BLECharacteristic distanceChar(deviceServiceCharacteristicUuid1, BLERead | BLENotify, 4);
BLECharacteristic speedChar(deviceServiceCharacteristicUuid2, BLERead | BLENotify, 4);
BLECharacteristic rpmChar(deviceServiceCharacteristicUuid3, BLERead | BLENotify, 4);

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600); 

  rotation = rpm = pevtime = 0;  // Initialize rotation count, RPM, and previous time variables to zero

  Serial.println("Speed Sensor");  // Print an introductory message to the serial monitor
  delay(2000);  // Wait for 2 seconds before proceeding

  // Attach interrupt to digital pin 2, which will trigger the Right_ISR function on a change (rise or fall) of the signal
  attachInterrupt(digitalPinToInterrupt(2), Right_ISR, CHANGE);  // Right_ISR is called when the wheel sensor is triggered

  // Initialize BLE functionality, if it fails, print an error and halt the program
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);  // Stay in this loop if BLE fails to initialize
  }

  // Set up BLE local device name and add the wheel rotation service
  BLE.setLocalName("RotationSensor");
  BLE.setAdvertisedService(wheelRotationService);  // Add the wheel rotation service UUID
  wheelRotationService.addCharacteristic(distanceChar);  // Add the distance characteristic
  wheelRotationService.addCharacteristic(speedChar);  // Add the speed characteristic
  wheelRotationService.addCharacteristic(rpmChar);    // Add the RPM characteristic
  BLE.addService(wheelRotationService);  // Register the wheel rotation service with BLE

  // Start advertising the BLE service, making it discoverable by central devices
  BLE.advertise();
}

void loop() {
  // Wait for a connection from a BLE central device (e.g., smartphone, tablet, etc.)
  BLEDevice central = BLE.central();

  if (central) {  // If a central device is connected
    Serial.print("Connected to central: ");
    Serial.println(central.address());  // Print the central device's address to the serial monitor

    digitalWrite(LED_BUILTIN, HIGH);  // Turn on the built-in LED to indicate that the device is connected

    // While the central device is still connected
    while (central.connected()) {
      // Check if there is any serial input (from the user via Serial Monitor)
      if (Serial.available() > 0) {
        char input = Serial.read();  // Read the input character from the serial monitor
        if (input == 'r') {  // Reset the rotation count and distance
          right_intr = 0;
          flag = 1;
          distance = 0;
          Serial.println("count reset");
        }
        else if (input == 's') {  // Display the current RPM, velocity, and distance. Stop the sensor reading 
          Serial.print("RPM- ");
          Serial.print(rpm);
          Serial.print("  Velocity- ");
          Serial.print(v);
          Serial.print("  Rotation- ");
          Serial.print(right_intr / 40);
          Serial.print("  Distance- ");
          Serial.println(distance);
          flag = 0;
        }
      }

      // If the flag is set to 1, update the display and send data
      if (flag) {
        // If no interrupts have been detected for 500ms, reset RPM and velocity
        if (millis() - dtime > 500) {
          rpm = v = 0;  // Reset RPM and velocity to zero
          dtime = millis();  // Update the last time a change was detected
        }
        // Calculate velocity and distance based on wheel rotation
        v = radius_of_wheel * rpm * 0.1047;  // Velocity in cm/s (conversion factor for RPM to velocity)
        distance = (2 * 3.141 * radius_of_wheel) * (right_intr / rotation_sensor_changes);  // Total distance traveled

        // Print current rotation and grid count for debugging
        Serial.print("  Rotation- ");
        Serial.print(right_intr / rotation_sensor_changes);  // Rotation count is divided by 4 (since 2 holes are detected per rotation)
        Serial.print("  Grid Count- ");
        Serial.println(right_intr);

        // Write the calculated distance, speed, and RPM to the BLE characteristics
        Serial.print("* Writing Distance: ");
        Serial.print(distance);
        distanceChar.writeValue((const void *)&distance, sizeof(distance));
        Serial.print("   Speed: ");
        Serial.print(v);
        speedChar.writeValue((const void *)&v, sizeof(v));
        Serial.print("   Rpm: ");
        Serial.println(rpm);
        rpmChar.writeValue((const void *)&rpm, sizeof(rpm));
        delay(500);  // Wait for 500ms before sending the next data
      }
    }

    // When the central device disconnects, turn off the LED and reset variables
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    right_intr = 0;  // Reset rotation count
    flag = 1;        // Reset flag
    distance = 0;    // Reset distance
    Serial.println("count reset");
  }
}

// Interrupt Service Routine (ISR) for detecting wheel rotation
void Right_ISR() {
  right_intr++;  // Increment the rotation count on each interrupt (wheel rotation)
  rotation++;    // Increment the rotation variable for RPM calculation
  dtime = millis();  // Update the time of the last interrupt

  // If the number of rotations reaches the sensor threshold, calculate RPM
  if (rotation >= rotation_sensor_changes) {
    timetaken = millis() - pevtime;  // Calculate the time taken for one full rotation in milliseconds
    rpm = (1000 / timetaken) * 60;   // Calculate RPM based on the time taken for one full rotation
    pevtime = millis();  // Update the previous time for the next calculation
    rotation = 0;  // Reset the rotation counter
  }
}
