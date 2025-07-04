/*
 * ESP32 LEGO Train Bridge with Rotation Sensor
 * Advanced Train Controller v3.0
 * 
 * This code creates a bridge between a laptop (via BLE) and a LEGO Powered Up hub,
 * with additional rotation sensor feedback for precise distance measurement.
 * 
 * Features:
 * - BLE communication with laptop for remote control
 * - LEGO Powered Up hub connection for motor control
 * - Rotation sensor for precise distance and speed measurement
 * - Emergency stop functionality
 * - LED status indicators
 * - Timed and distance-based movement control
 * - Real-time sensor data streaming
 */

#include "Lpf2Hub.h"
#include <NimBLEDevice.h>

// ============================================================================
// PIN DEFINITIONS - Arduino Nano ESP32 built-in LEDs
// ============================================================================
#define LED_RED 14      // RGB Red pin for status indication
#define LED_GREEN 16    // RGB Green pin for status indication
#define LED_BLUE 15     // RGB Blue pin for status indication

// ============================================================================
// ROTATION SENSOR CONFIGURATION
// ============================================================================
#define ROTATION_SENSOR_PIN 2  // Interrupt-capable pin for rotation sensor
const float WHEEL_RADIUS = 0.00825; // Radius of wheel in meters (8.25mm) - calibrate this for your wheel
const int NO_OF_HOLES = 2;          // Number of holes/slots in the encoder wheel
const int SENSOR_CHANGES_PER_ROTATION = 2 * NO_OF_HOLES; // 4 changes per rotation (rising + falling edges)

// ============================================================================
// ROTATION SENSOR VARIABLES
// ============================================================================
volatile int rotationInterruptCount = 0;  // Total interrupt count since reset (volatile for ISR access)
volatile int rotationCount = 0;           // Count for RPM calculation (resets after each RPM calculation)
volatile unsigned long lastInterruptTime = 0; // Last interrupt timestamp for debouncing
float sensorDistance = 0.0;              // Total distance traveled from sensor (cm)
float sensorSpeed = 0.0;                 // Current speed from sensor (cm/s)
float sensorRPM = 0.0;                   // Current RPM from sensor
unsigned long previousRPMTime = 0;       // Previous time for RPM calculation
unsigned long lastSensorUpdate = 0;      // Last time sensor data was updated
bool sensorEnabled = true;               // Enable/disable sensor functionality

// ============================================================================
// BLE SERVER VARIABLES - For laptop communication
// ============================================================================
NimBLEServer* pServer = nullptr;          // BLE server instance
NimBLECharacteristic* pCharacteristic = nullptr; // BLE characteristic for data exchange
bool laptopConnected = false;             // Track laptop connection status

// Service and Characteristic UUIDs for laptop communication
#define LAPTOP_SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define LAPTOP_CHARACTERISTIC_UUID "87654321-4321-4321-4321-cba987654321"

// ============================================================================
// LEGO HUB VARIABLES
// ============================================================================
Lpf2Hub legoHub;                         // LEGO Powered Up hub instance
byte motorPort = (byte)PoweredUpHubPort::A; // Motor port (A, B, C, or D)

// ============================================================================
// SYSTEM STATUS VARIABLES
// ============================================================================
bool isInitialized = false;              // Track if hub is properly initialized
int motorSpeed = 0;                      // Current motor speed (-100 to 100)
int hubBatteryLevel = -1;                // Hub battery level percentage
int hubRssi = 0;                         // Hub RSSI signal strength
String hubName = "";                     // Hub device name
int currentLedColor = 0;                 // Current ESP32 LED color

// ============================================================================
// MOVEMENT TRACKING VARIABLES
// ============================================================================
bool isMoving = false;                   // Track if robot is currently moving
unsigned long moveStartTime = 0;         // When current movement started
unsigned long moveDuration = 0;          // Duration for timed movements (milliseconds)
int targetSpeed = 0;                     // Target speed for current movement
String currentCommand = "";              // Current command being executed

// ============================================================================
// DISTANCE-BASED MOVEMENT CONTROL VARIABLES
// ============================================================================
bool isDistanceMovement = false;         // True if current movement is distance-based
float targetDistance = 0.0;             // Target distance to travel (cm)
float startDistance = 0.0;              // Starting distance when movement began
float remainingDistance = 0.0;          // Remaining distance to travel
bool useClosedLoopDistance = true;      // Use sensor feedback for distance control vs time estimation

// ============================================================================
// MOVEMENT STATISTICS VARIABLES
// ============================================================================
float totalDistance = 0.0;              // Total estimated distance traveled
unsigned long totalMoveTime = 0;        // Total time spent moving (milliseconds)
int maxSpeedUsed = 0;                   // Maximum speed used in any movement
int totalMovements = 0;                 // Total number of movements executed

// ============================================================================
// SAFETY AND CONTROL VARIABLES
// ============================================================================
bool emergencyStopActive = false;       // Emergency stop flag - prevents all movement
bool manualLedActive = false;           // Manual LED control override flag
unsigned long manualLedStartTime = 0;   // When manual LED control started
const unsigned long MANUAL_LED_TIMEOUT = 10000; // Manual LED timeout (10 seconds)

// ============================================================================
// ROTATION SENSOR INTERRUPT SERVICE ROUTINE
// This function is called every time the sensor detects a change (hole/no hole)
// IRAM_ATTR ensures it runs from RAM for faster execution
// ============================================================================
void IRAM_ATTR rotationSensorISR() {
    unsigned long currentTime = millis();
    
    // Optional debouncing - uncomment if sensor is noisy
    // Ignore interrupts that occur within 5ms of each other
    // if (currentTime - lastInterruptTime > 5) {
        rotationInterruptCount++;        // Increment total interrupt count
        rotationCount++;                 // Increment count for RPM calculation
        lastInterruptTime = currentTime; // Update last interrupt time
        
        // Calculate RPM when we have completed a full rotation
        if (rotationCount >= SENSOR_CHANGES_PER_ROTATION) {
            unsigned long timeTaken = currentTime - previousRPMTime;
            if (timeTaken > 0) {
                // RPM = (60 seconds * 1000 ms/second) / time for one rotation
                sensorRPM = (60000.0 / timeTaken);
                previousRPMTime = millis();
            }
            rotationCount = 0; // Reset count for next rotation
        }
    // }
}

// ============================================================================
// UPDATE SENSOR CALCULATIONS
// Process raw sensor data into meaningful distance, speed, and RPM values
// ============================================================================
void updateSensorData() {
    unsigned long currentTime = millis();
    
    // Reset RPM and speed if no movement detected for 1 second
    // This prevents showing old speed values when stopped
    if (currentTime - lastInterruptTime > 1000) {
        sensorRPM = 0.0;
        sensorSpeed = 0.0;
    } else {
        // Calculate linear speed from rotational speed
        // Speed = radius * angular_velocity
        // Convert RPM to rad/s: RPM * (2π/60) = RPM * 0.1047
        // Convert to cm/s: multiply by 100
        sensorSpeed = WHEEL_RADIUS * sensorRPM * 0.1047; // cm/s
    }
    
    // Calculate total distance traveled
    // Distance = circumference * number_of_rotations
    // Circumference = 2 * π * radius
    // Number of rotations = total_interrupts / interrupts_per_rotation
    sensorDistance = (2 * PI * WHEEL_RADIUS * 100) * (rotationInterruptCount / (float)SENSOR_CHANGES_PER_ROTATION); // in cm
    
    lastSensorUpdate = currentTime;
}

// ============================================================================
// HUB PROPERTY CHANGE CALLBACK
// Called when hub properties (battery, RSSI) are updated
// ============================================================================
void hubPropertyChangeCallback(void *hub, HubPropertyReference hubProperty, uint8_t *pData) {
    Lpf2Hub *myHub = (Lpf2Hub *)hub;
    
    // Handle battery level updates
    if (hubProperty == HubPropertyReference::BATTERY_VOLTAGE) {
        hubBatteryLevel = myHub->parseBatteryLevel(pData);
        Serial.println("Battery Level Updated: " + String(hubBatteryLevel) + "%");
        
        // Send battery update to laptop if connected
        if (laptopConnected) {
            String response = "BATTERY_UPDATE:" + String(hubBatteryLevel);
            const char* responseCStr = response.c_str();
            pCharacteristic->setValue((uint8_t*)responseCStr, response.length());
            pCharacteristic->notify();
        }
    }
    
    // Handle RSSI (signal strength) updates
    if (hubProperty == HubPropertyReference::RSSI) {
        hubRssi = myHub->parseRssi(pData);
        Serial.println("RSSI Updated: " + String(hubRssi) + " dB");
        
        // Send RSSI update to laptop if connected
        if (laptopConnected) {
            String response = "RSSI_UPDATE:" + String(hubRssi);
            const char* responseCStr = response.c_str();
            pCharacteristic->setValue((uint8_t*)responseCStr, response.length());
            pCharacteristic->notify();
        }
    }
}

// ============================================================================
// RGB LED CONTROL FUNCTION
// Set the ESP32's built-in RGB LED to specific colors for status indication
// Colors: 0=OFF, 1=RED, 2=BLUE, 3=GREEN, 4=YELLOW, 5=PURPLE, 6=CYAN, 7=WHITE
// ============================================================================
void setRGBLed(int color) {
    // Start with all LEDs off (HIGH = off for common anode RGB)
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    
    // Set specific color by pulling appropriate pins LOW
    switch(color) {
        case 0: break; // OFF - all remain HIGH
        case 1: digitalWrite(LED_RED, LOW); break; // RED
        case 2: digitalWrite(LED_BLUE, LOW); break; // BLUE
        case 3: digitalWrite(LED_GREEN, LOW); break; // GREEN
        case 4: digitalWrite(LED_RED, LOW); digitalWrite(LED_BLUE, LOW); break; // YELLOW (red + blue)
        case 5: digitalWrite(LED_RED, LOW); digitalWrite(LED_GREEN, LOW); break; // PURPLE (red + green)
        case 6: digitalWrite(LED_BLUE, LOW); digitalWrite(LED_GREEN, LOW); break; // CYAN (blue + green)
        case 7: digitalWrite(LED_RED, LOW); digitalWrite(LED_BLUE, LOW); digitalWrite(LED_GREEN, LOW); break; // WHITE
    }
    currentLedColor = color;
}

// ============================================================================
// LED STATUS UPDATE FUNCTION
// Automatically control LED based on system status (unless manual override active)
// LED Status Meanings:
// - Fast red blink: Emergency stop active
// - Yellow: Moving normally
// - Green: Connected to both hub and laptop, stopped
// - Blue blink: Partially connected (hub OR laptop)
// - Red blink: Not connected to either
// ============================================================================
void updateLEDStatus() {
    static unsigned long lastLedBlink = 0;  // Track last blink time
    static bool ledState = false;           // Track current blink state
    unsigned long currentTime = millis();
    
    // Check if manual LED control has timed out (10 seconds)
    if (manualLedActive && (currentTime - manualLedStartTime > MANUAL_LED_TIMEOUT)) {
        manualLedActive = false;
        Serial.println("Manual LED control timeout - returning to automatic status");
    }
    
    // Skip automatic LED updates if manual control is active
    if (manualLedActive) {
        return;
    }
    
    // Priority 1: Emergency stop - fast red blink
    if (emergencyStopActive) {
        if (currentTime - lastLedBlink > 150) { // 150ms interval = fast blink
            ledState = !ledState;
            setRGBLed(ledState ? 1 : 0); // Alternate between red and off
            lastLedBlink = currentTime;
        }
    } 
    // Priority 2: Fully connected (hub AND laptop)
    else if (legoHub.isConnected() && laptopConnected) {
        if (isMoving) {
            setRGBLed(4); // Yellow when moving
        } else {
            setRGBLed(2); // Green when connected and stopped
        }
    } 
    // Priority 3: Partially connected (hub OR laptop, but not both)
    else if (legoHub.isConnected() || laptopConnected) {
        if (currentTime - lastLedBlink > 500) { // 500ms interval = medium blink
            ledState = !ledState;
            setRGBLed(ledState ? 3 : 0); // Blue blink
            lastLedBlink = currentTime;
        }
    } 
    // Priority 4: Not connected to anything
    else {
        if (currentTime - lastLedBlink > 1000) { // 1000ms interval = slow blink
            ledState = !ledState;
            setRGBLed(ledState ? 1 : 0); // Red blink
            lastLedBlink = currentTime;
        }
    }
}

// ============================================================================
// BLE SERVER CALLBACKS
// Handle laptop connection and disconnection events
// ============================================================================
class LaptopServerCallbacks: public NimBLEServerCallbacks {
    // Called when laptop connects via BLE
    void onConnect(NimBLEServer* pServer) {
        laptopConnected = true;
        Serial.println(">>> LAPTOP CONNECTED VIA BLE <<<");
        sendStatusToLaptop(); // Send current status immediately
    }

    // Called when laptop disconnects from BLE
    void onDisconnect(NimBLEServer* pServer) {
        laptopConnected = false;
        Serial.println(">>> LAPTOP DISCONNECTED <<<");
        stopCurrentMovement(); // Safety: stop any ongoing movement
        pServer->startAdvertising(); // Resume advertising for new connections
    }
};

// ============================================================================
// BLE CHARACTERISTIC CALLBACKS
// Handle incoming commands from laptop
// ============================================================================
class LaptopCharacteristicCallbacks: public NimBLECharacteristicCallbacks {
    // Called when laptop writes a command to the characteristic
    void onWrite(NimBLECharacteristic *pCharacteristic) {
        String command = pCharacteristic->getValue().c_str();
        Serial.println(">>> COMMAND: " + command + " <<<");
        processCommand(command); // Process the received command
    }
};

// ============================================================================
// SEND LIVE SENSOR DATA TO LAPTOP
// Stream real-time sensor data for monitoring
// ============================================================================
void sendSensorDataToLaptop() {
    if (!laptopConnected) return;
    
    // Format sensor data as comma-separated values
    String sensorData = "SENSOR_LIVE:";
    sensorData += "DISTANCE:" + String(sensorDistance, 2) + ",";
    sensorData += "SPEED:" + String(sensorSpeed, 2) + ",";
    sensorData += "RPM:" + String(sensorRPM, 1);
    
    // Send via BLE notification
    const char* sensorCStr = sensorData.c_str();
    pCharacteristic->setValue((uint8_t*)sensorCStr, sensorData.length());
    pCharacteristic->notify();
}

// ============================================================================
// SEND COMPLETE STATUS TO LAPTOP
// Send comprehensive system status including all relevant parameters
// ============================================================================
void sendStatusToLaptop() {
    if (!laptopConnected) return;
    
    updateSensorData(); // Ensure sensor data is current
    
    // Build comprehensive status string
    String status = "STATUS:";
    status += "HUB:" + String(legoHub.isConnected() ? "C" : "D") + ",";
    status += "BATTERY:" + String(hubBatteryLevel) + ",";
    status += "MOTOR_SPEED:" + String(motorSpeed) + ",";
    status += "HUB_NAME:" + hubName + ",";
    status += "MOVING:" + String(isMoving ? "Y" : "N") + ",";
    status += "TOTAL_DISTANCE:" + String(totalDistance, 2) + ",";
    status += "SENSOR_DISTANCE:" + String(sensorDistance, 2) + ",";
    status += "SENSOR_SPEED:" + String(sensorSpeed, 2) + ",";
    status += "SENSOR_RPM:" + String(sensorRPM, 1) + ",";
    status += "TOTAL_TIME:" + String(totalMoveTime / 1000) + ",";
    status += "MAX_SPEED:" + String(maxSpeedUsed) + ",";
    status += "TOTAL_MOVEMENTS:" + String(totalMovements) + ",";
    status += "EMERGENCY:" + String(emergencyStopActive ? "Y" : "N") + ",";
    status += "SENSOR_ENABLED:" + String(sensorEnabled ? "Y" : "N") + ",";
    status += "DISTANCE_MODE:" + String(useClosedLoopDistance ? "SENSOR" : "TIME");
    
    // Add distance movement progress if currently doing distance-based movement
    if (isDistanceMovement) {
        float traveledDistance = abs(sensorDistance - startDistance);
        status += ",DISTANCE_PROGRESS:" + String(traveledDistance, 2) + "/" + String(targetDistance, 2);
    }
    
    Serial.println("Sending status to laptop: " + status);
    
    // Send status via BLE
    const char* statusCStr = status.c_str();
    pCharacteristic->setValue((uint8_t*)statusCStr, status.length());
    pCharacteristic->notify();
}

// ============================================================================
// STOP CURRENT MOVEMENT
// Safely stop any ongoing movement and update statistics
// ============================================================================
void stopCurrentMovement() {
    if (isMoving) {
        // Update total movement time
        totalMoveTime += (millis() - moveStartTime);
        
        // Calculate actual distance traveled for both movement types
        float actualDistance = abs(sensorDistance - startDistance);
        unsigned long actualTime = millis() - moveStartTime;
        
        // Handle distance-based movement completion
        if (isDistanceMovement) {
            Serial.println("Distance movement completed: Target=" + String(targetDistance, 2) + 
                          "cm, Actual=" + String(actualDistance, 2) + "cm, Time=" + String(actualTime) + "ms");
            
            // Send completion report to laptop
            if (laptopConnected) {
                String response = "DISTANCE_COMPLETED:TARGET:" + String(targetDistance, 2) + 
                                 ",ACTUAL:" + String(actualDistance, 2) + 
                                 ",ERROR:" + String(abs(targetDistance - actualDistance), 2) + 
                                 ",TIME:" + String(actualTime);
                const char* responseCStr = response.c_str();
                pCharacteristic->setValue((uint8_t*)responseCStr, response.length());
                pCharacteristic->notify();
            }
        } 
        // Handle timed movement completion
        else if (moveDuration > 0) {
            Serial.println("Timed movement completed: Duration=" + String(moveDuration) + 
                          "ms, Actual=" + String(actualTime) + "ms, Distance=" + String(actualDistance, 2) + "cm");
            
            // Send completion report to laptop
            if (laptopConnected) {
                String response = "TIMED_COMPLETED:PLANNED:" + String(moveDuration) + 
                                 ",ACTUAL:" + String(actualTime) + 
                                 ",DISTANCE:" + String(actualDistance, 2) + 
                                 ",SPEED:" + String(motorSpeed);
                const char* responseCStr = response.c_str();
                pCharacteristic->setValue((uint8_t*)responseCStr, response.length());
                pCharacteristic->notify();
            }
        }
        
        // Reset all movement-related variables
        isMoving = false;
        isDistanceMovement = false;
        moveStartTime = 0;
        moveDuration = 0;
        targetDistance = 0.0;
        startDistance = 0.0;
        remainingDistance = 0.0;
        currentCommand = "";
        
        // Stop the motor
        if (legoHub.isConnected()) {
            legoHub.stopBasicMotor(motorPort);
        }
        motorSpeed = 0;
        
        Serial.println(">>> MOVEMENT STOPPED <<<");
        
        // Send general completion message for manual stops
        if (laptopConnected && !isDistanceMovement && moveDuration == 0) {
            String response = "MOVEMENT_COMPLETED";
            const char* responseCStr = response.c_str();
            pCharacteristic->setValue((uint8_t*)responseCStr, response.length());
            pCharacteristic->notify();
        }
    }
}

// ============================================================================
// START DISTANCE-BASED MOVEMENT
// Begin a movement that will stop when target distance is reached
// ============================================================================
void startDistanceMovement(int speed, float distance) {
    // Safety check: ensure hub is connected
    if (!legoHub.isConnected()) {
        Serial.println("ERROR: Hub not connected");
        if (laptopConnected) {
            String response = "ERROR:HUB_NOT_CONNECTED";
            const char* responseCStr = response.c_str();
            pCharacteristic->setValue((uint8_t*)responseCStr, response.length());
            pCharacteristic->notify();
        }
        return;
    }
    
    // Safety check: ensure emergency stop is not active
    if (emergencyStopActive) {
        Serial.println("ERROR: Emergency stop active");
        if (laptopConnected) {
            String response = "ERROR:EMERGENCY_STOP_ACTIVE";
            const char* responseCStr = response.c_str();
            pCharacteristic->setValue((uint8_t*)responseCStr, response.length());
            pCharacteristic->notify();
        }
        return;
    }
    
    // Stop any current movement before starting new one
    stopCurrentMovement();
    
    // Initialize movement parameters
    motorSpeed = speed;
    targetSpeed = speed;
    moveStartTime = millis();
    isMoving = true;
    isDistanceMovement = true;
    targetDistance = distance;
    startDistance = sensorDistance; // Use current sensor distance as starting point
    remainingDistance = distance;
    totalMovements++;
    
    // Update maximum speed statistic
    if (abs(speed) > maxSpeedUsed) {
        maxSpeedUsed = abs(speed);
    }
    
    // Start the motor
    legoHub.setBasicMotorSpeed(motorPort, speed);
    
    Serial.println("Started distance movement: Speed=" + String(speed) + ", Target=" + String(distance) + "cm, Start=" + String(startDistance, 2) + "cm");
    
    // Notify laptop of movement start
    if (laptopConnected) {
        String response = "DISTANCE_MOVEMENT_STARTED:SPEED:" + String(speed) + 
                         ",TARGET:" + String(distance) + 
                         ",START:" + String(startDistance, 2) + 
                         ",MODE:" + (useClosedLoopDistance ? "SENSOR" : "TIME");
        const char* responseCStr = response.c_str();
        pCharacteristic->setValue((uint8_t*)responseCStr, response.length());
        pCharacteristic->notify();
    }
}

// ============================================================================
// START TIMED MOVEMENT
// Begin a movement that will stop after specified duration
// ============================================================================
void startTimedMovement(int speed, unsigned long duration) {
    // Safety check: ensure hub is connected
    if (!legoHub.isConnected()) {
        Serial.println("ERROR: Hub not connected");
        if (laptopConnected) {
            String response = "ERROR:HUB_NOT_CONNECTED";
            const char* responseCStr = response.c_str();
            pCharacteristic->setValue((uint8_t*)responseCStr, response.length());
            pCharacteristic->notify();
        }
        return;
    }
    
    // Safety check: ensure emergency stop is not active
    if (emergencyStopActive) {
        Serial.println("ERROR: Emergency stop active");
        if (laptopConnected) {
            String response = "ERROR:EMERGENCY_STOP_ACTIVE";
            const char* responseCStr = response.c_str();
            pCharacteristic->setValue((uint8_t*)responseCStr, response.length());
            pCharacteristic->notify();
        }
        return;
    }
    
    // Stop any current movement before starting new one
    stopCurrentMovement();
    
    // Initialize movement parameters
    motorSpeed = speed;
    targetSpeed = speed;
    moveStartTime = millis();
    moveDuration = duration;
    isMoving = true;
    isDistanceMovement = false; // This is timed movement, not distance-based
    startDistance = sensorDistance; // Track starting distance for statistics
    totalMovements++;
    
    // Update maximum speed statistic
    if (abs(speed) > maxSpeedUsed) {
        maxSpeedUsed = abs(speed);
    }
    
    // Start the motor
    legoHub.setBasicMotorSpeed(motorPort, speed);
    
    Serial.println("Started timed movement: Speed=" + String(speed) + ", Duration=" + String(duration) + "ms");
    
    // Notify laptop of movement start
    if (laptopConnected) {
        String response = "TIMED_MOVEMENT_STARTED:SPEED:" + String(speed) + 
                         ",DURATION:" + String(duration) + 
                         ",START_DISTANCE:" + String(sensorDistance, 2);
        const char* responseCStr = response.c_str();
        pCharacteristic->setValue((uint8_t*)responseCStr, response.length());
        pCharacteristic->notify();
    }
}

// ============================================================================
// PROCESS INCOMING COMMANDS
// Parse and execute commands received from laptop via BLE
// ============================================================================
void processCommand(String command) {
    String response = "";
    
    // GET_STATUS: Request current system status
    if (command == "GET_STATUS") {
        sendStatusToLaptop();
        return;
    }
    // GET_SENSOR_DATA: Request current sensor readings
    else if (command == "GET_SENSOR_DATA") {
        updateSensorData();
        response = "SENSOR_DATA:DISTANCE:" + String(sensorDistance, 2) + 
                  ",SPEED:" + String(sensorSpeed, 2) + 
                  ",RPM:" + String(sensorRPM, 1) + 
                  ",ROTATIONS:" + String(rotationInterruptCount / SENSOR_CHANGES_PER_ROTATION, 2);
    }
    // DEBUG_DISTANCE: Debug information for distance movement
    else if (command == "DEBUG_DISTANCE") {
        String debug = "DEBUG:START=" + String(startDistance, 2) + 
                      ",CURRENT=" + String(sensorDistance, 2) + 
                      ",TARGET=" + String(targetDistance, 2) + 
                      ",MOVING=" + String(isMoving ? "YES" : "NO") + 
                      ",DISTANCE_MODE=" + String(isDistanceMovement ? "YES" : "NO") + 
                      ",TRAVELED=" + String(abs(sensorDistance - startDistance), 2);
        response = debug;
        Serial.println(">>> DISTANCE DEBUG: " + debug + " <<<");
    }
    // RESET_SENSOR: Reset all sensor readings to zero
    else if (command == "RESET_SENSOR") {
        rotationInterruptCount = 0;
        rotationCount = 0;
        sensorDistance = 0.0;
        sensorSpeed = 0.0;
        sensorRPM = 0.0;
        previousRPMTime = millis();
        response = "SENSOR_RESET";
        Serial.println(">>> ROTATION SENSOR RESET <<<");
    }
    // SENSOR_ENABLE: Enable sensor functionality
    else if (command == "SENSOR_ENABLE") {
        sensorEnabled = true;
        response = "SENSOR_ENABLED";
        Serial.println(">>> ROTATION SENSOR ENABLED <<<");
    }
    // SENSOR_DISABLE: Disable sensor functionality
    else if (command == "SENSOR_DISABLE") {
        sensorEnabled = false;
        response = "SENSOR_DISABLED";
        Serial.println(">>> ROTATION SENSOR DISABLED <<<");
    }
    // PING: Simple connectivity test
    else if (command == "PING") {
        response = "PONG";
        Serial.println(">>> PING received, sending PONG <<<");
    }
    // EMERGENCY_STOP: Immediately stop all movement and prevent new movements
    else if (command == "EMERGENCY_STOP") {
        emergencyStopActive = true;
        stopCurrentMovement();
        response = "EMERGENCY_STOP_EXECUTED";
        Serial.println(">>> EMERGENCY STOP ACTIVATED <<<");
    }
    // CLEAR_EMERGENCY: Clear emergency stop condition
    else if (command == "CLEAR_EMERGENCY") {
        emergencyStopActive = false;
        response = "EMERGENCY_CLEARED";
        Serial.println(">>> EMERGENCY STOP CLEARED <<<");
    }
    // INIT_SYSTEM: Initialize/reset system to known state
    else if (command == "INIT_SYSTEM") {
        emergencyStopActive = false;
        stopCurrentMovement();
        response = "SYSTEM_INITIALIZED";
        Serial.println(">>> SYSTEM INITIALIZED <<<");
    }
    // TOGGLE_DISTANCE_MODE: Switch between sensor feedback and time estimation
    else if (command == "TOGGLE_DISTANCE_MODE") {
        useClosedLoopDistance = !useClosedLoopDistance;
        response = "DISTANCE_MODE:" + String(useClosedLoopDistance ? "SENSOR" : "TIME");
        Serial.println(">>> DISTANCE MODE: " + String(useClosedLoopDistance ? "SENSOR_FEEDBACK" : "TIME_ESTIMATION") + " <<<");
    }
    // GET_DISTANCE_MODE: Get current distance control mode
    else if (command == "GET_DISTANCE_MODE") {
        response = "DISTANCE_MODE:" + String(useClosedLoopDistance ? "SENSOR" : "TIME");
    }
    // MOTOR_SPEED_XX: Set motor speed directly (XX = speed from -100 to 100)
    else if (command.startsWith("MOTOR_SPEED_")) {
        int speed = command.substring(12).toInt(); // Extract speed value
        if (speed >= -100 && speed <= 100) {
            if (emergencyStopActive) {
                response = "ERROR:EMERGENCY_STOP_ACTIVE";
            } else if (legoHub.isConnected()) {
                motorSpeed = speed;
                if (speed == 0) {
                    stopCurrentMovement(); // Stop if speed is 0
                } else {
                    // Update max speed statistic
                    if (abs(speed) > maxSpeedUsed) maxSpeedUsed = abs(speed);
                    legoHub.setBasicMotorSpeed(motorPort, speed);
                    // If not currently moving, start movement tracking
                    if (!isMoving) {
                        isMoving = true;
                        moveStartTime = millis();
                        totalMovements++;
                    }
                }
                response = "MOTOR_SPEED_SET:" + String(speed);
            } else {
                response = "ERROR:HUB_NOT_CONNECTED";
            }
        } else {
            response = "ERROR:INVALID_SPEED";
        }
    }
    // MOVE_TIME_XXXX_SPEED_YY: Timed movement (XXXX=time in ms, YY=speed)
    else if (command.startsWith("MOVE_TIME_")) {
        int timeIndex = command.indexOf("_SPEED_");
        if (timeIndex > 0) {
            unsigned long moveTime = command.substring(10, timeIndex).toInt(); // Extract time
            int speed = command.substring(timeIndex + 7).toInt(); // Extract speed
            
            // Validate parameters (max 5 minutes, speed -100 to 100)
            if (moveTime > 0 && moveTime <= 300000 && speed >= -100 && speed <= 100) {
                startTimedMovement(speed, moveTime);
                response = "TIMED_MOVEMENT_STARTED";
            } else {
                response = "ERROR:INVALID_TIME_OR_SPEED";
            }
        } else {
            response = "ERROR:INVALID_FORMAT";
        }
    }
    // MOVE_DISTANCE_XX.X_SPEED_YY: Distance-based movement (XX.X=distance in cm, YY=speed)
    else if (command.startsWith("MOVE_DISTANCE_")) {
        int speedIndex = command.indexOf("_SPEED_");
        if (speedIndex > 0) {
            float distance = command.substring(14, speedIndex).toFloat(); // Extract distance
            int speed = command.substring(speedIndex + 7).toInt(); // Extract speed
            
            // Validate parameters (max 1000cm, speed -100 to 100)
            if (distance > 0 && distance <= 1000 && speed >= -100 && speed <= 100) {
                if (useClosedLoopDistance && sensorEnabled) {
                    // Use sensor-based distance control for precision
                    startDistanceMovement(speed, distance);
                    response = "SENSOR_DISTANCE_STARTED:DISTANCE:" + String(distance) + ",SPEED:" + String(speed);
                } else {
                    // Fall back to time-based estimation when sensor unavailable
                    float speedFactor = 0.3; // Calibration factor (cm per second per speed unit)
                    unsigned long estimatedTime = (distance / (abs(speed) * speedFactor)) * 1000;
                    estimatedTime = constrain(estimatedTime, 100, 300000); // 0.1s to 5 minutes
                    
                    startTimedMovement(speed, estimatedTime);
                    totalDistance += distance; // Add to estimated total distance
                    
                    response = "TIME_DISTANCE_STARTED:EST_TIME:" + String(estimatedTime) + 
                              ",DISTANCE:" + String(distance) + ",SPEED:" + String(speed);
                }
            } else {
                response = "ERROR:INVALID_DISTANCE_OR_SPEED";
            }
        } else {
            response = "ERROR:INVALID_FORMAT";
        }
    }
    // HUB_LED_COLOR: Control LEGO hub LED color
    else if (command.startsWith("HUB_LED_")) {
        String color = command.substring(8); // Extract color name
        if (legoHub.isConnected()) {
            // Set hub LED to specified color
            if (color == "RED") legoHub.setLedColor(RED);
            else if (color == "GREEN") legoHub.setLedColor(GREEN);
            else if (color == "BLUE") legoHub.setLedColor(BLUE);
            else if (color == "YELLOW") legoHub.setLedColor(YELLOW);
            else if (color == "PURPLE") legoHub.setLedColor(PURPLE);
            else if (color == "WHITE") legoHub.setLedColor(WHITE);
            else if (color == "OFF") legoHub.setLedColor(BLACK);
            
            response = "HUB_LED_SET:" + color;
        } else {
            response = "ERROR:HUB_NOT_CONNECTED";
        }
    }
    // ESP32_LED_ACTION: Control ESP32 RGB LED
    else if (command.startsWith("ESP32_LED_")) {
        String action = command.substring(10); // Extract LED action
        
        if (action == "AUTO") {
            // Return to automatic status indication
            manualLedActive = false;
            Serial.println("LED: Returning to automatic status mode");
            response = "ESP32_LED_AUTO_DONE";
        }
        else if (action == "OFF") {
            // Turn off LED and return to automatic mode
            setRGBLed(0);
            manualLedActive = false;
            Serial.println("LED: Manual OFF - returning to automatic status mode");
            response = "ESP32_LED_OFF_DONE";
        }
        else {
            // Manual color override (with 10-second timeout)
            manualLedActive = true;
            manualLedStartTime = millis();
            
            // Set specific colors or special effects
            if (action == "RED") setRGBLed(1);
            else if (action == "GREEN") setRGBLed(2);
            else if (action == "BLUE") setRGBLed(3);
            else if (action == "YELLOW") setRGBLed(4);
            else if (action == "PURPLE") setRGBLed(5);
            else if (action == "CYAN") setRGBLed(6);
            else if (action == "WHITE") setRGBLed(7);
            else if (action == "BLINK") {
                // Fun blinking sequence
                for (int i = 0; i < 3; i++) {
                    setRGBLed(1); delay(200); // Red
                    setRGBLed(2); delay(200); // Green
                    setRGBLed(3); delay(200); // Blue
                    setRGBLed(0); delay(200); // Off
                }
            }
            
            response = "ESP32_LED_" + action + "_DONE";
            Serial.println("LED: Manual color override (" + action + ") - will return to auto in 10s");
        }
    }
    // RESET_STATS: Reset all movement statistics and sensor data
    else if (command == "RESET_STATS") {
        totalDistance = 0.0;
        totalMoveTime = 0;
        maxSpeedUsed = 0;
        totalMovements = 0;
        // Also reset sensor data
        rotationInterruptCount = 0;
        sensorDistance = 0.0;
        // Reset distance movement tracking
        isDistanceMovement = false;
        targetDistance = 0.0;
        startDistance = 0.0;
        remainingDistance = 0.0;
        response = "STATS_RESET";
        Serial.println(">>> STATISTICS, SENSOR, AND DISTANCE TRACKING RESET <<<");
    }
    // GET_BATTERY: Request hub battery level update
    else if (command == "GET_BATTERY") {
        if (legoHub.isConnected()) {
            legoHub.activateHubPropertyUpdate(HubPropertyReference::BATTERY_VOLTAGE, hubPropertyChangeCallback);
            response = "BATTERY_REQUEST_SENT";
        } else {
            response = "ERROR:HUB_NOT_CONNECTED";
        }
    }
    // DISCONNECT_HUB: Safely disconnect from LEGO hub
    else if (command == "DISCONNECT_HUB") {
        if (legoHub.isConnected()) {
            stopCurrentMovement(); // Stop any movement before disconnecting
            legoHub.shutDownHub();
            response = "HUB_DISCONNECTED";
            isInitialized = false;
        } else {
            response = "ERROR:HUB_ALREADY_DISCONNECTED";
        }
    }
    // RECONNECT_HUB: Attempt to reconnect to LEGO hub
    else if (command == "RECONNECT_HUB") {
        isInitialized = false;
        legoHub.init(); // Reinitialize hub connection
        response = "HUB_RECONNECTION_STARTED";
    }
    // FORCE_HUB_CHECK: Check current hub connection status
    else if (command == "FORCE_HUB_CHECK") {
        response = "HUB_CHECK:HUB_STATUS:" + String(legoHub.isConnected() ? "CONNECTED" : "DISCONNECTED");
        if (legoHub.isConnected()) {
            response += ",HUB_NAME:" + hubName;
        }
        Serial.println("Force hub check - Hub connected: " + String(legoHub.isConnected()));
    }
    // GET_DETAILED_STATUS: Send comprehensive status with all parameters
    else if (command == "GET_DETAILED_STATUS") {
        updateSensorData(); // Ensure current sensor data
        String detailedStatus = "DETAILED_STATUS:";
        detailedStatus += "HUB_CONNECTED:" + String(legoHub.isConnected() ? "true" : "false") + ",";
        detailedStatus += "HUB_NAME:" + hubName + ",";
        detailedStatus += "BATTERY_LEVEL:" + String(hubBatteryLevel) + ",";
        detailedStatus += "RSSI:" + String(hubRssi) + ",";
        detailedStatus += "MOTOR_SPEED:" + String(motorSpeed) + ",";
        detailedStatus += "IS_MOVING:" + String(isMoving ? "true" : "false") + ",";
        detailedStatus += "TOTAL_DISTANCE:" + String(totalDistance, 2) + ",";
        detailedStatus += "SENSOR_DISTANCE:" + String(sensorDistance, 2) + ",";
        detailedStatus += "SENSOR_SPEED:" + String(sensorSpeed, 2) + ",";
        detailedStatus += "SENSOR_RPM:" + String(sensorRPM, 1) + ",";
        detailedStatus += "TOTAL_TIME:" + String(totalMoveTime / 1000) + ",";
        detailedStatus += "MAX_SPEED:" + String(maxSpeedUsed) + ",";
        detailedStatus += "TOTAL_MOVEMENTS:" + String(totalMovements) + ",";
        detailedStatus += "EMERGENCY_STOP:" + String(emergencyStopActive ? "true" : "false") + ",";
        detailedStatus += "SENSOR_ENABLED:" + String(sensorEnabled ? "true" : "false") + ",";
        detailedStatus += "DISTANCE_MODE:" + String(useClosedLoopDistance ? "SENSOR" : "TIME") + ",";
        detailedStatus += "DISTANCE_MOVEMENT:" + String(isDistanceMovement ? "true" : "false");
        
        // Add distance movement progress if currently active
        if (isDistanceMovement) {
            float traveledDistance = abs(sensorDistance - startDistance);
            detailedStatus += ",TARGET_DISTANCE:" + String(targetDistance, 2) + 
                            ",TRAVELED_DISTANCE:" + String(traveledDistance, 2) + 
                            ",REMAINING_DISTANCE:" + String(targetDistance - traveledDistance, 2);
        }
        
        // Send detailed status immediately
        const char* detailedCStr = detailedStatus.c_str();
        pCharacteristic->setValue((uint8_t*)detailedCStr, detailedStatus.length());
        pCharacteristic->notify();
        return;
    }
    // Unknown command handling
    else {
        response = "UNKNOWN_COMMAND:" + command;
    }
    
    // Send response if one was generated and laptop is connected
    if (response.length() > 0 && laptopConnected) {
        const char* responseCStr = response.c_str();
        pCharacteristic->setValue((uint8_t*)responseCStr, response.length());
        pCharacteristic->notify();
        Serial.println("Response: " + response);
    }
}

// ============================================================================
// INITIALIZE BLE SERVER
// Set up BLE server for laptop communication
// ============================================================================
void initializeBLE() {
    Serial.println(">>> INITIALIZING BLE SERVER <<<");
    
    // Initialize BLE device with name
    NimBLEDevice::init("ESP32-LEGO-Train");
    
    // Create BLE server and set callbacks
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new LaptopServerCallbacks());

    // Create BLE service with custom UUID
    NimBLEService *pService = pServer->createService(LAPTOP_SERVICE_UUID);

    // Create BLE characteristic for bidirectional communication
    pCharacteristic = pService->createCharacteristic(
                        LAPTOP_CHARACTERISTIC_UUID,
                        NIMBLE_PROPERTY::READ |   // Allow reading
                        NIMBLE_PROPERTY::WRITE |  // Allow writing (commands)
                        NIMBLE_PROPERTY::NOTIFY   // Allow notifications (responses)
                      );

    // Set characteristic callbacks for handling incoming data
    pCharacteristic->setCallbacks(new LaptopCharacteristicCallbacks());
    pService->start();

    // Start advertising so laptop can find and connect
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(LAPTOP_SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);
    pServer->getAdvertising()->start();
    
    Serial.println(">>> BLE ADVERTISING STARTED <<<");
}

// ============================================================================
// INITIALIZE ROTATION SENSOR
// Set up interrupt-based rotation sensor for precise distance measurement
// ============================================================================
void initializeRotationSensor() {
    // Configure sensor pin with internal pull-up resistor
    pinMode(ROTATION_SENSOR_PIN, INPUT_PULLUP);
    
    // Attach interrupt to trigger on both rising and falling edges
    // This gives us maximum resolution (4 interrupts per rotation with 2 holes)
    attachInterrupt(digitalPinToInterrupt(ROTATION_SENSOR_PIN), rotationSensorISR, CHANGE);
    
    // Initialize all sensor variables to known state
    rotationInterruptCount = 0;
    rotationCount = 0;
    sensorDistance = 0.0;
    sensorSpeed = 0.0;
    sensorRPM = 0.0;
    previousRPMTime = millis();
    lastInterruptTime = 0;
    lastSensorUpdate = 0;
    
    Serial.println(">>> ROTATION SENSOR INITIALIZED ON PIN " + String(ROTATION_SENSOR_PIN) + " <<<");
    Serial.println("Wheel radius: " + String(WHEEL_RADIUS * 100) + " cm");
    Serial.println("Sensor changes per rotation: " + String(SENSOR_CHANGES_PER_ROTATION));
}

// ============================================================================
// ARDUINO SETUP FUNCTION
// Initialize all systems and prepare for operation
// ============================================================================
void setup() {
    // Initialize serial communication for debugging
    Serial.begin(115200);
    delay(1000); // Allow serial to stabilize
    
    // Print startup banner
    Serial.println("\n>>> ESP32 LEGO TRAIN BRIDGE WITH ROTATION SENSOR <<<");
    Serial.println("Advanced Train Controller v3.0");
    Serial.println("================================================");
    
    // Configure RGB LED pins as outputs
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    
    // Initialize safety and control flags
    emergencyStopActive = false;  // Start with emergency stop cleared
    manualLedActive = false;      // Start with automatic LED control
    
    // Initialize distance movement variables to safe state
    isDistanceMovement = false;
    targetDistance = 0.0;
    startDistance = 0.0;
    remainingDistance = 0.0;
    
    // Set initial LED to red (not connected)
    setRGBLed(1);
    
    // Initialize hardware subsystems
    initializeRotationSensor(); // Set up rotation sensor with interrupts
    initializeBLE();            // Set up BLE server for laptop communication
    legoHub.init();             // Initialize LEGO hub connection
    
    // Print final setup status
    Serial.println(">>> SETUP COMPLETE <<<");
    Serial.println("Emergency Stop: CLEAR");
    Serial.println("LED Control: Automatic");
    Serial.println("Rotation Sensor: Enabled");
    Serial.println("Distance Control: " + String(useClosedLoopDistance ? "Sensor Feedback" : "Time Estimation"));
    Serial.println("Waiting for connections...");
}

// ============================================================================
// ARDUINO MAIN LOOP
// Continuously monitor systems and handle ongoing operations
// ============================================================================
void loop() {
    // Update LED status based on current system state
    updateLEDStatus();
    
    // Sensor data update and transmission timing variables
    static unsigned long lastSensorDataUpdate = 0;
    static unsigned long lastSensorDataSent = 0;
    
    // Update sensor calculations every 50ms for smooth data
    if (millis() - lastSensorDataUpdate > 50) {
        updateSensorData();
        lastSensorDataUpdate = millis();
    }
    
    // Send live sensor data to laptop every 200ms if connected
    if (laptopConnected && (millis() - lastSensorDataSent > 200)) {
        sendSensorDataToLaptop();
        lastSensorDataSent = millis();
    }
    
    // ========================================================================
    // LEGO HUB CONNECTION MANAGEMENT
    // Handle hub connection establishment and status updates
    // ========================================================================
    if (legoHub.isConnecting() && !isInitialized) {
        legoHub.connectHub(); // Attempt to complete connection
        
        if (legoHub.isConnected()) {
            Serial.println(">>> LEGO HUB CONNECTED <<<");
            hubName = String(legoHub.getHubName().c_str());
            
            // Activate property updates for battery and signal strength monitoring
            legoHub.activateHubPropertyUpdate(HubPropertyReference::BATTERY_VOLTAGE, hubPropertyChangeCallback);
            legoHub.activateHubPropertyUpdate(HubPropertyReference::RSSI, hubPropertyChangeCallback);
            
            isInitialized = true;
            
            // Notify laptop of successful hub connection
            if (laptopConnected) {
                delay(100); // Small delay to ensure stable connection
                sendStatusToLaptop();
                Serial.println("Hub connection status sent to laptop");
            }
        }
    }
    
    // ========================================================================
    // TIMED MOVEMENT MONITORING
    // Check for completion of time-based movements and send progress updates
    // ========================================================================
    if (isMoving && moveDuration > 0 && !isDistanceMovement) {
        unsigned long elapsed = millis() - moveStartTime;
        
        // Send progress update every 1 seconds for timed movements
        static unsigned long lastTimedProgressUpdate = 0;
        if (millis() - lastTimedProgressUpdate > 1000) {
            if (laptopConnected) {
                float elapsedSeconds = elapsed / 1000.0;
                float totalSeconds = moveDuration / 1000.0;
                float progressPercent = (elapsed / (float)moveDuration) * 100.0;
                String progress = "TIMED_PROGRESS:ELAPSED:" + String(elapsedSeconds, 1) + 
                                 ",TOTAL:" + String(totalSeconds, 1) + 
                                 ",PROGRESS:" + String(progressPercent, 1) + 
                                 ",DISTANCE:" + String(abs(sensorDistance - startDistance), 2);
                const char* progressCStr = progress.c_str();
                pCharacteristic->setValue((uint8_t*)progressCStr, progress.length());
                pCharacteristic->notify();
            }
            lastTimedProgressUpdate = millis();
        }
        
        // Check if movement duration has elapsed
        if (elapsed >= moveDuration) {
            stopCurrentMovement();
        }
    }
    
    // ========================================================================
    // DISTANCE-BASED MOVEMENT MONITORING
    // Check for completion of sensor-feedback distance movements
    // ========================================================================
    if (isMoving && isDistanceMovement && useClosedLoopDistance) {
        float traveledDistance = abs(sensorDistance - startDistance);
        remainingDistance = targetDistance - traveledDistance;
        
        // Debug output every second for distance tracking
        static unsigned long lastDebugOutput = 0;
        if (millis() - lastDebugOutput > 1000) {
            Serial.println("Distance Control - Start: " + String(startDistance, 2) + 
                          "cm, Current: " + String(sensorDistance, 2) + 
                          "cm, Traveled: " + String(traveledDistance, 2) + 
                          "cm, Target: " + String(targetDistance, 2) + 
                          "cm, Remaining: " + String(remainingDistance, 2) + "cm");
            lastDebugOutput = millis();
        }
        
        // Send progress update to laptop every 1 seconds
        static unsigned long lastProgressUpdate = 0;
        if (millis() - lastProgressUpdate > 1000) {
            if (laptopConnected) {
                String progress = "DISTANCE_PROGRESS:TRAVELED:" + String(traveledDistance, 2) + 
                                 ",REMAINING:" + String(remainingDistance, 2) + 
                                 ",TARGET:" + String(targetDistance, 2) + 
                                 ",PROGRESS:" + String((traveledDistance / targetDistance) * 100, 1) + 
                                 ",START:" + String(startDistance, 2) + 
                                 ",CURRENT:" + String(sensorDistance, 2);
                const char* progressCStr = progress.c_str();
                pCharacteristic->setValue((uint8_t*)progressCStr, progress.length());
                pCharacteristic->notify();
            }
            lastProgressUpdate = millis();
        }
        
        // Check if target distance reached (with 0.5cm tolerance for precision)
        if (traveledDistance >= (targetDistance - 0.5)) {
            Serial.println("Target distance reached: " + String(traveledDistance, 2) + "/" + String(targetDistance, 2) + "cm");
            stopCurrentMovement();
        }
        
        // Safety timeout: stop if movement takes more than 10 minutes
        if (millis() - moveStartTime > 600000) {
            Serial.println("Distance movement timeout - stopping");
            stopCurrentMovement();
        }
    }
    
    // ========================================================================
    // PERIODIC STATUS REPORTING
    // Print comprehensive status to serial every 10 seconds
    // ========================================================================
    static unsigned long lastStatus = 0;
    if (millis() - lastStatus > 10000) {
        Serial.println("--- Status ---");
        Serial.println("Hub: " + String(legoHub.isConnected() ? "Connected" : "Disconnected"));
        Serial.println("Laptop: " + String(laptopConnected ? "Connected" : "Disconnected"));
        Serial.println("Emergency: " + String(emergencyStopActive ? "ACTIVE" : "Clear"));
        Serial.println("LED Control: " + String(manualLedActive ? "Manual" : "Automatic"));
        Serial.println("Distance Mode: " + String(useClosedLoopDistance ? "Sensor" : "Time"));
        Serial.println("Sensor: Distance=" + String(sensorDistance, 1) + "cm, Speed=" + String(sensorSpeed, 1) + "cm/s, RPM=" + String(sensorRPM, 1));
        
        // Show current movement status
        if (isMoving) {
            String moveType = isDistanceMovement ? "Distance" : "Timed";
            Serial.println("Moving (" + moveType + "): Speed=" + String(motorSpeed) + ", Elapsed=" + String((millis() - moveStartTime) / 1000) + "s");
            if (isDistanceMovement) {
                float traveled = abs(sensorDistance - startDistance);
                Serial.println("  Target: " + String(targetDistance, 1) + "cm, Traveled: " + String(traveled, 1) + "cm, Remaining: " + String(targetDistance - traveled, 1) + "cm");
            }
        } else {
            Serial.println("Status: Stopped");
        }
        
        // Show movement statistics
        Serial.println("Stats: Estimated=" + String(totalDistance) + "cm, Actual=" + String(sensorDistance, 1) + "cm, Time=" + String(totalMoveTime / 1000) + "s");
        lastStatus = millis();
    }
    
    // Small delay to prevent overwhelming the system
    delay(50);
}