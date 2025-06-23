#include "Lpf2Hub.h"
#include <NimBLEDevice.h>

// Arduino Nano ESP32 built-in LEDs
#define LED_RED 14      // RGB Red pin
#define LED_GREEN 16    // RGB Green pin  
#define LED_BLUE 15     // RGB Blue pin

// Rotation Sensor Configuration
#define ROTATION_SENSOR_PIN 2  // Interrupt pin for rotation sensor
const float WHEEL_RADIUS = 0.00825; // Radius of wheel in meters (8.25mm)
const int NO_OF_HOLES = 2;          // Number of holes in the wheel
const int SENSOR_CHANGES_PER_ROTATION = 2 * NO_OF_HOLES; // 4 changes per rotation

// Rotation Sensor Variables
volatile int rotationInterruptCount = 0;  // Raw interrupt count
volatile int rotationCount = 0;           // Count for RPM calculation
volatile unsigned long lastInterruptTime = 0;
float sensorDistance = 0.0;              // Distance from sensor (cm)
float sensorSpeed = 0.0;                 // Speed from sensor (cm/s)
float sensorRPM = 0.0;                   // RPM from sensor
unsigned long previousRPMTime = 0;       // Previous time for RPM calculation
unsigned long lastSensorUpdate = 0;     // Last time sensor data was updated
bool sensorEnabled = true;               // Enable/disable sensor

// BLE Server for laptop communication
NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pCharacteristic = nullptr;
bool laptopConnected = false;

// Service and Characteristic UUIDs for laptop communication
#define LAPTOP_SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define LAPTOP_CHARACTERISTIC_UUID "87654321-4321-4321-4321-cba987654321"

// LEGO Hub
Lpf2Hub legoHub;
byte motorPort = (byte)PoweredUpHubPort::A;

// Status variables
bool isInitialized = false;
int motorSpeed = 0;
int hubBatteryLevel = -1;
int hubRssi = 0;
String hubName = "";
int currentLedColor = 0;

// Advanced movement tracking
bool isMoving = false;
unsigned long moveStartTime = 0;
unsigned long moveDuration = 0;
int targetSpeed = 0;
String currentCommand = "";

// Distance-based movement control
bool isDistanceMovement = false;
float targetDistance = 0.0;
float startDistance = 0.0;
float remainingDistance = 0.0;
bool useClosedLoopDistance = true; // Use sensor feedback for distance control

// Movement statistics
float totalDistance = 0.0;
unsigned long totalMoveTime = 0;
int maxSpeedUsed = 0;
int totalMovements = 0;

// Emergency stop flag
bool emergencyStopActive = false;

// Manual LED control
bool manualLedActive = false;
unsigned long manualLedStartTime = 0;
const unsigned long MANUAL_LED_TIMEOUT = 10000; // 10 seconds timeout

// Rotation Sensor Interrupt Service Routine
void IRAM_ATTR rotationSensorISR() {
    unsigned long currentTime = millis();
    
    // Debounce: ignore interrupts too close together (< 5ms)
    if (currentTime - lastInterruptTime > 5) {
        rotationInterruptCount++;
        rotationCount++;
        lastInterruptTime = currentTime;
        
        // Calculate RPM when we have a full rotation
        if (rotationCount >= SENSOR_CHANGES_PER_ROTATION) {
            unsigned long timeTaken = currentTime - previousRPMTime;
            if (timeTaken > 0) {
                sensorRPM = (60000.0 / timeTaken); // RPM calculation
                previousRPMTime = currentTime;
            }
            rotationCount = 0;
        }
    }
}

// Update sensor calculations
void updateSensorData() {
    unsigned long currentTime = millis();
    
    // Reset RPM and speed if no movement detected for 1000ms
    if (currentTime - lastInterruptTime > 1000) {
        sensorRPM = 0.0;
        sensorSpeed = 0.0;
    } else {
        // Calculate speed from RPM
        sensorSpeed = WHEEL_RADIUS * sensorRPM * 0.1047; // cm/s
    }
    
    // Calculate total distance
    sensorDistance = (2 * PI * WHEEL_RADIUS * 100) * (rotationInterruptCount / (float)SENSOR_CHANGES_PER_ROTATION); // in cm
    
    lastSensorUpdate = currentTime;
}

// Hub property callback function
void hubPropertyChangeCallback(void *hub, HubPropertyReference hubProperty, uint8_t *pData) {
    Lpf2Hub *myHub = (Lpf2Hub *)hub;
    
    if (hubProperty == HubPropertyReference::BATTERY_VOLTAGE) {
        hubBatteryLevel = myHub->parseBatteryLevel(pData);
        Serial.println("Battery Level Updated: " + String(hubBatteryLevel) + "%");
        
        if (laptopConnected) {
            String response = "BATTERY_UPDATE:" + String(hubBatteryLevel);
            const char* responseCStr = response.c_str();
            pCharacteristic->setValue((uint8_t*)responseCStr, response.length());
            pCharacteristic->notify();
        }
    }
    
    if (hubProperty == HubPropertyReference::RSSI) {
        hubRssi = myHub->parseRssi(pData);
        Serial.println("RSSI Updated: " + String(hubRssi) + " dB");
        
        if (laptopConnected) {
            String response = "RSSI_UPDATE:" + String(hubRssi);
            const char* responseCStr = response.c_str();
            pCharacteristic->setValue((uint8_t*)responseCStr, response.length());
            pCharacteristic->notify();
        }
    }
}

// Function to set RGB LED color
void setRGBLed(int color) {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    
    switch(color) {
        case 0: break; // OFF
        case 1: digitalWrite(LED_RED, LOW); break; // RED
        case 2: digitalWrite(LED_GREEN, LOW); break; // GREEN
        case 3: digitalWrite(LED_BLUE, LOW); break; // BLUE
        case 4: digitalWrite(LED_RED, LOW); digitalWrite(LED_GREEN, LOW); break; // YELLOW
        case 5: digitalWrite(LED_RED, LOW); digitalWrite(LED_BLUE, LOW); break; // PURPLE
        case 6: digitalWrite(LED_GREEN, LOW); digitalWrite(LED_BLUE, LOW); break; // CYAN
        case 7: digitalWrite(LED_RED, LOW); digitalWrite(LED_GREEN, LOW); digitalWrite(LED_BLUE, LOW); break; // WHITE
    }
    currentLedColor = color;
}

void updateLEDStatus() {
    static unsigned long lastLedBlink = 0;
    static bool ledState = false;
    unsigned long currentTime = millis();
    
    // Check if manual LED control has timed out
    if (manualLedActive && (currentTime - manualLedStartTime > MANUAL_LED_TIMEOUT)) {
        manualLedActive = false;
        Serial.println("Manual LED control timeout - returning to automatic status");
    }
    
    // Skip automatic LED updates if manual control is active
    if (manualLedActive) {
        return;
    }
    
    if (emergencyStopActive) {
        // Fast red blink for emergency stop
        if (currentTime - lastLedBlink > 150) {
            ledState = !ledState;
            setRGBLed(ledState ? 1 : 0); // Fast red blink
            lastLedBlink = currentTime;
        }
    } else if (legoHub.isConnected() && laptopConnected) {
        if (isMoving) {
            setRGBLed(4); // Yellow when moving
        } else {
            setRGBLed(2); // Green when both connected and stopped
        }
    } else if (legoHub.isConnected() || laptopConnected) {
        // Partially connected - blinking blue
        if (currentTime - lastLedBlink > 500) {
            ledState = !ledState;
            setRGBLed(ledState ? 3 : 0); // Blue blink
            lastLedBlink = currentTime;
        }
    } else {
        // Not connected - slow red blink
        if (currentTime - lastLedBlink > 1000) {
            ledState = !ledState;
            setRGBLed(ledState ? 1 : 0); // Red blink
            lastLedBlink = currentTime;
        }
    }
}

// BLE Server Callbacks
class LaptopServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
        laptopConnected = true;
        Serial.println(">>> LAPTOP CONNECTED VIA BLE <<<");
        sendStatusToLaptop();
    }

    void onDisconnect(NimBLEServer* pServer) {
        laptopConnected = false;
        Serial.println(">>> LAPTOP DISCONNECTED <<<");
        stopCurrentMovement();
        pServer->startAdvertising();
    }
};

class LaptopCharacteristicCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pCharacteristic) {
        String command = pCharacteristic->getValue().c_str();
        Serial.println(">>> COMMAND: " + command + " <<<");
        processCommand(command);
    }
};

void sendSensorDataToLaptop() {
    if (!laptopConnected) return;
    
    String sensorData = "SENSOR_LIVE:";
    sensorData += "DISTANCE:" + String(sensorDistance, 2) + ",";
    sensorData += "SPEED:" + String(sensorSpeed, 2) + ",";
    sensorData += "RPM:" + String(sensorRPM, 1);
    
    const char* sensorCStr = sensorData.c_str();
    pCharacteristic->setValue((uint8_t*)sensorCStr, sensorData.length());
    pCharacteristic->notify();
}

void sendStatusToLaptop() {
    if (!laptopConnected) return;
    
    updateSensorData(); // Update sensor data before sending
    
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
    
    // Add distance movement progress if active
    if (isDistanceMovement) {
        float traveledDistance = abs(sensorDistance - startDistance);
        status += ",DISTANCE_PROGRESS:" + String(traveledDistance, 2) + "/" + String(targetDistance, 2);
    }
    
    Serial.println("Sending status to laptop: " + status);
    
    const char* statusCStr = status.c_str();
    pCharacteristic->setValue((uint8_t*)statusCStr, status.length());
    pCharacteristic->notify();
}

void stopCurrentMovement() {
    if (isMoving) {
        totalMoveTime += (millis() - moveStartTime);
        
        // Calculate actual distance traveled for both movement types
        float actualDistance = abs(sensorDistance - startDistance);
        unsigned long actualTime = millis() - moveStartTime;
        
        if (isDistanceMovement) {
            Serial.println("Distance movement completed: Target=" + String(targetDistance, 2) + 
                          "cm, Actual=" + String(actualDistance, 2) + "cm, Time=" + String(actualTime) + "ms");
            
            if (laptopConnected) {
                String response = "DISTANCE_COMPLETED:TARGET:" + String(targetDistance, 2) + 
                                 ",ACTUAL:" + String(actualDistance, 2) + 
                                 ",ERROR:" + String(abs(targetDistance - actualDistance), 2) + 
                                 ",TIME:" + String(actualTime);
                const char* responseCStr = response.c_str();
                pCharacteristic->setValue((uint8_t*)responseCStr, response.length());
                pCharacteristic->notify();
            }
        } else if (moveDuration > 0) {
            // This was a timed movement
            Serial.println("Timed movement completed: Duration=" + String(moveDuration) + 
                          "ms, Actual=" + String(actualTime) + "ms, Distance=" + String(actualDistance, 2) + "cm");
            
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
        
        isMoving = false;
        isDistanceMovement = false;
        moveStartTime = 0;
        moveDuration = 0;
        targetDistance = 0.0;
        startDistance = 0.0;
        remainingDistance = 0.0;
        currentCommand = "";
        
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

void startDistanceMovement(int speed, float distance) {
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
    
    stopCurrentMovement();
    
    motorSpeed = speed;
    targetSpeed = speed;
    moveStartTime = millis();
    isMoving = true;
    isDistanceMovement = true;
    targetDistance = distance;
    startDistance = sensorDistance; // Use current sensor distance as starting point
    remainingDistance = distance;
    totalMovements++;
    
    if (abs(speed) > maxSpeedUsed) {
        maxSpeedUsed = abs(speed);
    }
    
    legoHub.setBasicMotorSpeed(motorPort, speed);
    
    Serial.println("Started distance movement: Speed=" + String(speed) + ", Target=" + String(distance) + "cm, Start=" + String(startDistance, 2) + "cm");
    
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

void startTimedMovement(int speed, unsigned long duration) {
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
    
    stopCurrentMovement();
    
    motorSpeed = speed;
    targetSpeed = speed;
    moveStartTime = millis();
    moveDuration = duration;
    isMoving = true;
    isDistanceMovement = false; // This is timed movement, not distance
    startDistance = sensorDistance; // Track starting distance for timed movements too
    totalMovements++;
    
    if (abs(speed) > maxSpeedUsed) {
        maxSpeedUsed = abs(speed);
    }
    
    legoHub.setBasicMotorSpeed(motorPort, speed);
    
    Serial.println("Started timed movement: Speed=" + String(speed) + ", Duration=" + String(duration) + "ms");
    
    if (laptopConnected) {
        String response = "TIMED_MOVEMENT_STARTED:SPEED:" + String(speed) + 
                         ",DURATION:" + String(duration) + 
                         ",START_DISTANCE:" + String(sensorDistance, 2);
        const char* responseCStr = response.c_str();
        pCharacteristic->setValue((uint8_t*)responseCStr, response.length());
        pCharacteristic->notify();
    }
}

void processCommand(String command) {
    String response = "";
    
    if (command == "GET_STATUS") {
        sendStatusToLaptop();
        return;
    }
    else if (command == "GET_SENSOR_DATA") {
        updateSensorData();
        response = "SENSOR_DATA:DISTANCE:" + String(sensorDistance, 2) + 
                  ",SPEED:" + String(sensorSpeed, 2) + 
                  ",RPM:" + String(sensorRPM, 1) + 
                  ",ROTATIONS:" + String(rotationInterruptCount / SENSOR_CHANGES_PER_ROTATION, 2);
    }
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
    else if (command == "SENSOR_ENABLE") {
        sensorEnabled = true;
        response = "SENSOR_ENABLED";
        Serial.println(">>> ROTATION SENSOR ENABLED <<<");
    }
    else if (command == "SENSOR_DISABLE") {
        sensorEnabled = false;
        response = "SENSOR_DISABLED";
        Serial.println(">>> ROTATION SENSOR DISABLED <<<");
    }
    else if (command == "PING") {
        response = "PONG";
        Serial.println(">>> PING received, sending PONG <<<");
    }
    else if (command == "EMERGENCY_STOP") {
        emergencyStopActive = true;
        stopCurrentMovement();
        response = "EMERGENCY_STOP_EXECUTED";
        Serial.println(">>> EMERGENCY STOP ACTIVATED <<<");
    }
    else if (command == "CLEAR_EMERGENCY") {
        emergencyStopActive = false;
        response = "EMERGENCY_CLEARED";
        Serial.println(">>> EMERGENCY STOP CLEARED <<<");
    }
    else if (command == "INIT_SYSTEM") {
        emergencyStopActive = false;
        stopCurrentMovement();
        response = "SYSTEM_INITIALIZED";
        Serial.println(">>> SYSTEM INITIALIZED <<<");
    }
    else if (command == "TOGGLE_DISTANCE_MODE") {
        useClosedLoopDistance = !useClosedLoopDistance;
        response = "DISTANCE_MODE:" + String(useClosedLoopDistance ? "SENSOR" : "TIME");
        Serial.println(">>> DISTANCE MODE: " + String(useClosedLoopDistance ? "SENSOR_FEEDBACK" : "TIME_ESTIMATION") + " <<<");
    }
    else if (command == "GET_DISTANCE_MODE") {
        response = "DISTANCE_MODE:" + String(useClosedLoopDistance ? "SENSOR" : "TIME");
    }
    else if (command.startsWith("MOTOR_SPEED_")) {
        int speed = command.substring(12).toInt();
        if (speed >= -100 && speed <= 100) {
            if (emergencyStopActive) {
                response = "ERROR:EMERGENCY_STOP_ACTIVE";
            } else if (legoHub.isConnected()) {
                motorSpeed = speed;
                if (speed == 0) {
                    stopCurrentMovement();
                } else {
                    if (abs(speed) > maxSpeedUsed) maxSpeedUsed = abs(speed);
                    legoHub.setBasicMotorSpeed(motorPort, speed);
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
    else if (command.startsWith("MOVE_TIME_")) {
        int timeIndex = command.indexOf("_SPEED_");
        if (timeIndex > 0) {
            unsigned long moveTime = command.substring(10, timeIndex).toInt();
            int speed = command.substring(timeIndex + 7).toInt();
            
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
    else if (command.startsWith("MOVE_DISTANCE_")) {
        int speedIndex = command.indexOf("_SPEED_");
        if (speedIndex > 0) {
            float distance = command.substring(14, speedIndex).toFloat();
            int speed = command.substring(speedIndex + 7).toInt();
            
            if (distance > 0 && distance <= 1000 && speed >= -100 && speed <= 100) {
                if (useClosedLoopDistance && sensorEnabled) {
                    // Use sensor-based distance control
                    startDistanceMovement(speed, distance);
                    response = "SENSOR_DISTANCE_STARTED:DISTANCE:" + String(distance) + ",SPEED:" + String(speed);
                } else {
                    // Fall back to time-based estimation
                    float speedFactor = 0.3; // Improved calibration factor (cm per second per speed unit)
                    unsigned long estimatedTime = (distance / (abs(speed) * speedFactor)) * 1000;
                    estimatedTime = constrain(estimatedTime, 100, 300000); // 0.1s to 5 minutes
                    
                    startTimedMovement(speed, estimatedTime);
                    totalDistance += distance;
                    
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
    else if (command.startsWith("HUB_LED_")) {
        String color = command.substring(8);
        if (legoHub.isConnected()) {
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
    else if (command.startsWith("ESP32_LED_")) {
        String action = command.substring(10);
        
        if (action == "AUTO") {
            manualLedActive = false;
            Serial.println("LED: Returning to automatic status mode");
            response = "ESP32_LED_AUTO_DONE";
        }
        else if (action == "OFF") {
            setRGBLed(0);
            manualLedActive = false;
            Serial.println("LED: Manual OFF - returning to automatic status mode");
            response = "ESP32_LED_OFF_DONE";
        }
        else {
            manualLedActive = true;
            manualLedStartTime = millis();
            
            if (action == "RED") setRGBLed(1);
            else if (action == "GREEN") setRGBLed(2);
            else if (action == "BLUE") setRGBLed(3);
            else if (action == "YELLOW") setRGBLed(4);
            else if (action == "PURPLE") setRGBLed(5);
            else if (action == "CYAN") setRGBLed(6);
            else if (action == "WHITE") setRGBLed(7);
            else if (action == "BLINK") {
                for (int i = 0; i < 3; i++) {
                    setRGBLed(1); delay(200);
                    setRGBLed(2); delay(200);
                    setRGBLed(3); delay(200);
                    setRGBLed(0); delay(200);
                }
            }
            
            response = "ESP32_LED_" + action + "_DONE";
            Serial.println("LED: Manual color override (" + action + ") - will return to auto in 10s");
        }
    }
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
    else if (command == "GET_BATTERY") {
        if (legoHub.isConnected()) {
            legoHub.activateHubPropertyUpdate(HubPropertyReference::BATTERY_VOLTAGE, hubPropertyChangeCallback);
            response = "BATTERY_REQUEST_SENT";
        } else {
            response = "ERROR:HUB_NOT_CONNECTED";
        }
    }
    else if (command == "DISCONNECT_HUB") {
        if (legoHub.isConnected()) {
            stopCurrentMovement();
            legoHub.shutDownHub();
            response = "HUB_DISCONNECTED";
            isInitialized = false;
        } else {
            response = "ERROR:HUB_ALREADY_DISCONNECTED";
        }
    }
    else if (command == "RECONNECT_HUB") {
        isInitialized = false;
        legoHub.init();
        response = "HUB_RECONNECTION_STARTED";
    }
    else if (command == "FORCE_HUB_CHECK") {
        response = "HUB_CHECK:HUB_STATUS:" + String(legoHub.isConnected() ? "CONNECTED" : "DISCONNECTED");
        if (legoHub.isConnected()) {
            response += ",HUB_NAME:" + hubName;
        }
        Serial.println("Force hub check - Hub connected: " + String(legoHub.isConnected()));
    }
    else if (command == "GET_DETAILED_STATUS") {
        updateSensorData();
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
        
        if (isDistanceMovement) {
            float traveledDistance = abs(sensorDistance - startDistance);
            detailedStatus += ",TARGET_DISTANCE:" + String(targetDistance, 2) + 
                            ",TRAVELED_DISTANCE:" + String(traveledDistance, 2) + 
                            ",REMAINING_DISTANCE:" + String(targetDistance - traveledDistance, 2);
        }
        
        const char* detailedCStr = detailedStatus.c_str();
        pCharacteristic->setValue((uint8_t*)detailedCStr, detailedStatus.length());
        pCharacteristic->notify();
        return;
    }
    else {
        response = "UNKNOWN_COMMAND:" + command;
    }
    
    if (response.length() > 0 && laptopConnected) {
        const char* responseCStr = response.c_str();
        pCharacteristic->setValue((uint8_t*)responseCStr, response.length());
        pCharacteristic->notify();
        Serial.println("Response: " + response);
    }
}

void initializeBLE() {
    Serial.println(">>> INITIALIZING BLE SERVER <<<");
    
    NimBLEDevice::init("ESP32-LEGO-Train");
    
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new LaptopServerCallbacks());

    NimBLEService *pService = pServer->createService(LAPTOP_SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
                        LAPTOP_CHARACTERISTIC_UUID,
                        NIMBLE_PROPERTY::READ |
                        NIMBLE_PROPERTY::WRITE |
                        NIMBLE_PROPERTY::NOTIFY
                      );

    pCharacteristic->setCallbacks(new LaptopCharacteristicCallbacks());
    pService->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(LAPTOP_SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);
    pServer->getAdvertising()->start();
    
    Serial.println(">>> BLE ADVERTISING STARTED <<<");
}

void initializeRotationSensor() {
    pinMode(ROTATION_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ROTATION_SENSOR_PIN), rotationSensorISR, CHANGE);
    
    // Initialize sensor variables
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

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n>>> ESP32 LEGO TRAIN BRIDGE WITH ROTATION SENSOR <<<");
    Serial.println("Advanced Train Controller v3.0");
    Serial.println("================================================");
    
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    
    // Initialize emergency stop as false
    emergencyStopActive = false;
    manualLedActive = false;
    
    // Initialize distance movement variables
    isDistanceMovement = false;
    targetDistance = 0.0;
    startDistance = 0.0;
    remainingDistance = 0.0;
    
    setRGBLed(1); // Start with red
    
    // Initialize rotation sensor
    initializeRotationSensor();
    
    initializeBLE();
    legoHub.init();
    
    Serial.println(">>> SETUP COMPLETE <<<");
    Serial.println("Emergency Stop: CLEAR");
    Serial.println("LED Control: Automatic");
    Serial.println("Rotation Sensor: Enabled");
    Serial.println("Distance Control: " + String(useClosedLoopDistance ? "Sensor Feedback" : "Time Estimation"));
    Serial.println("Waiting for connections...");
}

void loop() {
    updateLEDStatus();
    
    // Update sensor data periodically and send to laptop if connected
    static unsigned long lastSensorDataUpdate = 0;
    static unsigned long lastSensorDataSent = 0;
    
    if (millis() - lastSensorDataUpdate > 50) { // Update every 50ms for smoother data
        updateSensorData();
        lastSensorDataUpdate = millis();
    }
    
    // Send sensor data to laptop every 200ms for real-time updates
    if (laptopConnected && (millis() - lastSensorDataSent > 200)) {
        sendSensorDataToLaptop();
        lastSensorDataSent = millis();
    }
    
    // Handle LEGO Hub connection
    if (legoHub.isConnecting() && !isInitialized) {
        legoHub.connectHub();
        if (legoHub.isConnected()) {
            Serial.println(">>> LEGO HUB CONNECTED <<<");
            hubName = String(legoHub.getHubName().c_str());
            
            legoHub.activateHubPropertyUpdate(HubPropertyReference::BATTERY_VOLTAGE, hubPropertyChangeCallback);
            legoHub.activateHubPropertyUpdate(HubPropertyReference::RSSI, hubPropertyChangeCallback);
            
            isInitialized = true;
            
            if (laptopConnected) {
                delay(100);
                sendStatusToLaptop();
                Serial.println("Hub connection status sent to laptop");
            }
        }
    }
    
    // Check for timed movement completion
    if (isMoving && moveDuration > 0 && !isDistanceMovement) {
        unsigned long elapsed = millis() - moveStartTime;
        
        // Send progress update every 2 seconds for timed movements
        static unsigned long lastTimedProgressUpdate = 0;
        if (millis() - lastTimedProgressUpdate > 2000) {
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
        
        // Check if time is up
        if (elapsed >= moveDuration) {
            stopCurrentMovement();
        }
    }
    
    // Check for distance-based movement completion
    if (isMoving && isDistanceMovement && useClosedLoopDistance) {
        float traveledDistance = abs(sensorDistance - startDistance);
        remainingDistance = targetDistance - traveledDistance;
        
        // Debug output every second
        static unsigned long lastDebugOutput = 0;
        if (millis() - lastDebugOutput > 1000) {
            Serial.println("Distance Control - Start: " + String(startDistance, 2) + 
                          "cm, Current: " + String(sensorDistance, 2) + 
                          "cm, Traveled: " + String(traveledDistance, 2) + 
                          "cm, Target: " + String(targetDistance, 2) + 
                          "cm, Remaining: " + String(remainingDistance, 2) + "cm");
            lastDebugOutput = millis();
        }
        
        // Send progress update every 2 seconds
        static unsigned long lastProgressUpdate = 0;
        if (millis() - lastProgressUpdate > 2000) {
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
        
        // Check if target distance reached (with small tolerance)
        if (traveledDistance >= (targetDistance - 0.5)) { // 0.5cm tolerance
            Serial.println("Target distance reached: " + String(traveledDistance, 2) + "/" + String(targetDistance, 2) + "cm");
            stopCurrentMovement();
        }
        
        // Safety timeout: stop if movement takes too long (max 10 minutes)
        if (millis() - moveStartTime > 600000) {
            Serial.println("Distance movement timeout - stopping");
            stopCurrentMovement();
        }
    }
    
    // Periodic status update
    static unsigned long lastStatus = 0;
    if (millis() - lastStatus > 10000) {
        Serial.println("--- Status ---");
        Serial.println("Hub: " + String(legoHub.isConnected() ? "Connected" : "Disconnected"));
        Serial.println("Laptop: " + String(laptopConnected ? "Connected" : "Disconnected"));
        Serial.println("Emergency: " + String(emergencyStopActive ? "ACTIVE" : "Clear"));
        Serial.println("LED Control: " + String(manualLedActive ? "Manual" : "Automatic"));
        Serial.println("Distance Mode: " + String(useClosedLoopDistance ? "Sensor" : "Time"));
        Serial.println("Sensor: Distance=" + String(sensorDistance, 1) + "cm, Speed=" + String(sensorSpeed, 1) + "cm/s, RPM=" + String(sensorRPM, 1));
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
        Serial.println("Stats: Estimated=" + String(totalDistance) + "cm, Actual=" + String(sensorDistance, 1) + "cm, Time=" + String(totalMoveTime / 1000) + "s");
        lastStatus = millis();
    }
    
    delay(50);
}