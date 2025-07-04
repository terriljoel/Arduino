#include "Lpf2Hub.h"
#include <NimBLEDevice.h>

// h 
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

// Movement statistics
float totalDistance = 0.0;
unsigned long totalMoveTime = 0;
int maxSpeedUsed = 0;
int totalMovements = 0;

// Emergency stop flag (start as false)
bool emergencyStopActive = false;

// Manual LED control
bool manualLedActive = false;
unsigned long manualLedStartTime = 0;
const unsigned long MANUAL_LED_TIMEOUT = 10000; // 10 seconds timeout

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

void sendStatusToLaptop() {
    if (!laptopConnected) return;
    
    String status = "STATUS:";
    status += "HUB:" + String(legoHub.isConnected() ? "C" : "D") + ",";
    status += "BATTERY:" + String(hubBatteryLevel) + ",";
    status += "MOTOR_SPEED:" + String(motorSpeed) + ",";
    status += "HUB_NAME:" + hubName + ",";
    status += "MOVING:" + String(isMoving ? "Y" : "N") + ",";
    status += "TOTAL_DISTANCE:" + String(totalDistance, 2) + ",";
    status += "TOTAL_TIME:" + String(totalMoveTime / 1000) + ",";
    status += "MAX_SPEED:" + String(maxSpeedUsed) + ",";
    status += "TOTAL_MOVEMENTS:" + String(totalMovements) + ",";
    status += "EMERGENCY:" + String(emergencyStopActive ? "Y" : "N");
    
    // Debug: Print what we're sending
    Serial.println("Sending status to laptop: " + status);
    
    // Ensure we're sending as text
    const char* statusCStr = status.c_str();
    pCharacteristic->setValue((uint8_t*)statusCStr, status.length());
    pCharacteristic->notify();
}

void stopCurrentMovement() {
    if (isMoving) {
        totalMoveTime += (millis() - moveStartTime);
        isMoving = false;
        moveStartTime = 0;
        moveDuration = 0;
        currentCommand = "";
        
        if (legoHub.isConnected()) {
            legoHub.stopBasicMotor(motorPort);
        }
        motorSpeed = 0;
        
        Serial.println(">>> MOVEMENT STOPPED <<<");
        
        if (laptopConnected) {
            String response = "MOVEMENT_COMPLETED";
            const char* responseCStr = response.c_str();
            pCharacteristic->setValue((uint8_t*)responseCStr, response.length());
            pCharacteristic->notify();
        }
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
    totalMovements++;
    
    if (abs(speed) > maxSpeedUsed) {
        maxSpeedUsed = abs(speed);
    }
    
    legoHub.setBasicMotorSpeed(motorPort, speed);
    
    Serial.println("Started timed movement: Speed=" + String(speed) + ", Duration=" + String(duration) + "ms");
    
    if (laptopConnected) {
        String response = "MOVEMENT_STARTED:SPEED:" + String(speed) + ",DURATION:" + String(duration);
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
        // Initialize system to known good state
        emergencyStopActive = false;
        stopCurrentMovement();
        response = "SYSTEM_INITIALIZED";
        Serial.println(">>> SYSTEM INITIALIZED <<<");
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
        // Format: MOVE_TIME_5000_SPEED_50 (move for 5 seconds at speed 50)
        int timeIndex = command.indexOf("_SPEED_");
        if (timeIndex > 0) {
            unsigned long moveTime = command.substring(10, timeIndex).toInt();
            int speed = command.substring(timeIndex + 7).toInt();
            
            if (moveTime > 0 && moveTime <= 300000 && speed >= -100 && speed <= 100) { // Max 5 minutes
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
        // Format: MOVE_DISTANCE_100_SPEED_50 (estimated distance in cm)
        int speedIndex = command.indexOf("_SPEED_");
        if (speedIndex > 0) {
            float distance = command.substring(14, speedIndex).toFloat();
            int speed = command.substring(speedIndex + 7).toInt();
            
            if (distance > 0 && distance <= 1000 && speed >= -100 && speed <= 100) { // Max 10 meters
                // Improved calculation: assume calibrated speed-to-distance ratio
                // This can be calibrated based on actual train measurements
                float speedFactor = 0.2; // cm per second per speed unit (adjustable)
                unsigned long estimatedTime = (distance / (abs(speed) * speedFactor)) * 1000;
                estimatedTime = constrain(estimatedTime, 100, 300000); // 0.1s to 5 minutes
                
                startTimedMovement(speed, estimatedTime);
                totalDistance += distance;
                
                response = "DISTANCE_MOVEMENT_STARTED:EST_TIME:" + String(estimatedTime) + ",DISTANCE:" + String(distance);
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
            // Return to automatic LED status
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
            // All other colors: activate manual control with timeout
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
        response = "STATS_RESET";
        Serial.println(">>> STATISTICS RESET <<<");
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
        // Force check hub connection and send immediate status
        response = "HUB_CHECK:HUB_STATUS:" + String(legoHub.isConnected() ? "CONNECTED" : "DISCONNECTED");
        if (legoHub.isConnected()) {
            response += ",HUB_NAME:" + hubName;
        }
        Serial.println("Force hub check - Hub connected: " + String(legoHub.isConnected()));
    }
    else if (command == "GET_DETAILED_STATUS") {
        String detailedStatus = "DETAILED_STATUS:";
        detailedStatus += "HUB_CONNECTED:" + String(legoHub.isConnected() ? "true" : "false") + ",";
        detailedStatus += "HUB_NAME:" + hubName + ",";
        detailedStatus += "BATTERY_LEVEL:" + String(hubBatteryLevel) + ",";
        detailedStatus += "RSSI:" + String(hubRssi) + ",";
        detailedStatus += "MOTOR_SPEED:" + String(motorSpeed) + ",";
        detailedStatus += "IS_MOVING:" + String(isMoving ? "true" : "false") + ",";
        detailedStatus += "TOTAL_DISTANCE:" + String(totalDistance, 2) + ",";
        detailedStatus += "TOTAL_TIME:" + String(totalMoveTime / 1000) + ",";
        detailedStatus += "MAX_SPEED:" + String(maxSpeedUsed) + ",";
        detailedStatus += "TOTAL_MOVEMENTS:" + String(totalMovements) + ",";
        detailedStatus += "EMERGENCY_STOP:" + String(emergencyStopActive ? "true" : "false");
        
        const char* detailedCStr = detailedStatus.c_str();
        pCharacteristic->setValue((uint8_t*)detailedCStr, detailedStatus.length());
        pCharacteristic->notify();
        return;
    }
    else {
        response = "UNKNOWN_COMMAND:" + command;
    }
    
    if (response.length() > 0 && laptopConnected) {
        // Ensure proper text transmission
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

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n>>> ESP32 LEGO TRAIN BRIDGE <<<");
    Serial.println("Advanced Train Controller v2.0");
    Serial.println("===============================");
    
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    
    // Initialize emergency stop as false
    emergencyStopActive = false;
    
    // Start with automatic LED status (not manual)
    manualLedActive = false;
    
    setRGBLed(1); // Start with red
    
    initializeBLE();
    legoHub.init();
    
    Serial.println(">>> SETUP COMPLETE <<<");
    Serial.println("Emergency Stop: CLEAR");
    Serial.println("LED Control: Automatic");
    Serial.println("Waiting for connections...");
}

void loop() {
    updateLEDStatus();
    
    // Handle LEGO Hub connection
    if (legoHub.isConnecting() && !isInitialized) {
        legoHub.connectHub();
        if (legoHub.isConnected()) {
            Serial.println(">>> LEGO HUB CONNECTED <<<");
            hubName = String(legoHub.getHubName().c_str());
            
            legoHub.activateHubPropertyUpdate(HubPropertyReference::BATTERY_VOLTAGE, hubPropertyChangeCallback);
            legoHub.activateHubPropertyUpdate(HubPropertyReference::RSSI, hubPropertyChangeCallback);
            
            isInitialized = true;
            
            // Immediately send status update to laptop when hub connects
            if (laptopConnected) {
                delay(100); // Small delay to ensure hub is fully ready
                sendStatusToLaptop();
                Serial.println("Hub connection status sent to laptop");
            }
        }
    }
    
    // Check for timed movement completion
    if (isMoving && moveDuration > 0) {
        if (millis() - moveStartTime >= moveDuration) {
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
        if (isMoving) {
            Serial.println("Moving: Speed=" + String(motorSpeed) + ", Elapsed=" + String((millis() - moveStartTime) / 1000) + "s");
        } else {
            Serial.println("Status: Stopped");
        }
        Serial.println("Stats: Distance=" + String(totalDistance) + "cm, Time=" + String(totalMoveTime / 1000) + "s, Movements=" + String(totalMovements));
        lastStatus = millis();
    }
    
    delay(50);
}