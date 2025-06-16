#include <NimBLEDevice.h>

// Arduino Nano ESP32 built-in LEDs
#define LED_RED 14      // RGB Red pin
#define LED_GREEN 16    // RGB Green pin  
#define LED_BLUE 15     // RGB Blue pin

// BLE Server for laptop communication
NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pCharacteristic = nullptr;
bool laptopConnected = false;

// Service and Characteristic UUIDs for laptop communication
#define LAPTOP_SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define LAPTOP_CHARACTERISTIC_UUID "87654321-4321-4321-4321-cba987654321"

// Status variables
int testValue = 0;
int currentLedColor = 0; // 0=off, 1=red, 2=green, 3=blue

// Function to set RGB LED color
void setRGBLed(int color) {
    // Turn off all colors first (HIGH = OFF for RGB LED)
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    
    // Set the desired color (LOW = ON for RGB LED)
    switch(color) {
        case 0: // OFF - all already high
            break;
        case 1: // RED
            digitalWrite(LED_RED, LOW);
            break;
        case 2: // GREEN
            digitalWrite(LED_GREEN, LOW);
            break;
        case 3: // BLUE
            digitalWrite(LED_BLUE, LOW);
            break;
        case 4: // YELLOW (Red + Green)
            digitalWrite(LED_RED, LOW);
            digitalWrite(LED_GREEN, LOW);
            break;
        case 5: // PURPLE (Red + Blue)
            digitalWrite(LED_RED, LOW);
            digitalWrite(LED_BLUE, LOW);
            break;
        case 6: // CYAN (Green + Blue)
            digitalWrite(LED_GREEN, LOW);
            digitalWrite(LED_BLUE, LOW);
            break;
        case 7: // WHITE (All colors)
            digitalWrite(LED_RED, LOW);
            digitalWrite(LED_GREEN, LOW);
            digitalWrite(LED_BLUE, LOW);
            break;
    }
    currentLedColor = color;
}

// BLE Server Callbacks for laptop connection
class LaptopServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
        laptopConnected = true;
        Serial.println(">>> LAPTOP CONNECTED VIA BLE <<<");
        setRGBLed(2); // Green = connected
        
        // Send initial test response
        String welcome = "CONNECTED:ESP32_BLE_TEST_READY";
        pCharacteristic->setValue(welcome.c_str());
        pCharacteristic->notify();
    }

    void onDisconnect(NimBLEServer* pServer) {
        laptopConnected = false;
        Serial.println(">>> LAPTOP DISCONNECTED <<<");
        setRGBLed(1); // Red = disconnected
        
        // Restart advertising
        pServer->startAdvertising();
        Serial.println("BLE advertising restarted");
    }
};

// BLE Characteristic Callbacks for handling laptop commands
class LaptopCharacteristicCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pCharacteristic) {
        String command = pCharacteristic->getValue().c_str();
        Serial.println(">>> RECEIVED COMMAND: " + command + " <<<");
        processTestCommand(command);
    }
};

// Process test commands from laptop
void processTestCommand(String command) {
    Serial.println("Processing: " + command);
    
    if (command == "TEST_PING") {
        String response = "TEST_PONG";
        pCharacteristic->setValue(response.c_str());
        pCharacteristic->notify();
        Serial.println("Sent: " + response);
    }
    else if (command == "GET_STATUS") {
        String status = "STATUS:";
        status += "CONNECTED:" + String(laptopConnected ? "YES" : "NO") + ",";
        status += "TEST_VALUE:" + String(testValue) + ",";
        status += "LED_COLOR:" + String(currentLedColor);
        
        pCharacteristic->setValue(status.c_str());
        pCharacteristic->notify();
        Serial.println("Status sent: " + status);
    }
    else if (command.startsWith("SET_VALUE_")) {
        int newValue = command.substring(10).toInt();
        testValue = newValue;
        
        String response = "VALUE_SET:" + String(testValue);
        pCharacteristic->setValue(response.c_str());
        pCharacteristic->notify();
        Serial.println("Test value set to: " + String(testValue));
    }
    else if (command.startsWith("LED_")) {
        String action = command.substring(4);
        
        if (action == "OFF") {
            setRGBLed(0);
        } else if (action == "RED") {
            setRGBLed(1);
        } else if (action == "GREEN") {
            setRGBLed(2);
        } else if (action == "BLUE") {
            setRGBLed(3);
        } else if (action == "YELLOW") {
            setRGBLed(4);
        } else if (action == "PURPLE") {
            setRGBLed(5);
        } else if (action == "CYAN") {
            setRGBLed(6);
        } else if (action == "WHITE") {
            setRGBLed(7);
        } else if (action == "BLINK") {
            // Blink sequence
            for (int i = 0; i < 3; i++) {
                setRGBLed(1); delay(200);
                setRGBLed(2); delay(200);
                setRGBLed(3); delay(200);
                setRGBLed(0); delay(200);
            }
            setRGBLed(2); // Back to green (connected)
        }
        
        String response = "LED_" + action + "_DONE";
        pCharacteristic->setValue(response.c_str());
        pCharacteristic->notify();
        Serial.println("LED command executed: " + action);
    }
    else {
        String response = "UNKNOWN_COMMAND:" + command;
        pCharacteristic->setValue(response.c_str());
        pCharacteristic->notify();
        Serial.println("Unknown command: " + command);
    }
}

// Initialize BLE server for laptop communication
void initializeBLE() {
    Serial.println(">>> INITIALIZING BLE SERVER <<<");
    
    // Initialize NimBLE with shorter name to avoid truncation
    NimBLEDevice::init("ESP32-LEGO");
    Serial.println("Device name set to: ESP32-LEGO");
    
    // Create BLE Server
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new LaptopServerCallbacks());
    Serial.println("BLE server created");

    // Create BLE Service
    NimBLEService *pService = pServer->createService(LAPTOP_SERVICE_UUID);
    Serial.println("Service created with UUID: " + String(LAPTOP_SERVICE_UUID));

    // Create BLE Characteristic
    pCharacteristic = pService->createCharacteristic(
                        LAPTOP_CHARACTERISTIC_UUID,
                        NIMBLE_PROPERTY::READ |
                        NIMBLE_PROPERTY::WRITE |
                        NIMBLE_PROPERTY::NOTIFY
                      );
    Serial.println("Characteristic created with UUID: " + String(LAPTOP_CHARACTERISTIC_UUID));

    pCharacteristic->setCallbacks(new LaptopCharacteristicCallbacks());

    // Start the service
    pService->start();
    Serial.println("Service started");

    // Start advertising
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(LAPTOP_SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);
    pServer->getAdvertising()->start();
    
    Serial.println(">>> BLE ADVERTISING STARTED <<<");
    Serial.println("Ready for laptop connection!");
}

void setup() {
    Serial.begin(115200);
    delay(1000); // Give serial time to initialize
    
    Serial.println("\n\n========================================");
    Serial.println(">>> ESP32 BLE TEST - NO LEGO HUB <<<");
    Serial.println("Hardware: Arduino Nano ESP32");
    Serial.println("========================================");
    
    // Initialize RGB LED pins
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    
    // Start with red LED (not connected)
    setRGBLed(1);
    Serial.println("RGB LED initialized - Red (waiting for connection)");
    
    // Initialize BLE server
    initializeBLE();
    
    Serial.println(">>> SETUP COMPLETE <<<");
    Serial.println("Waiting for laptop to connect...");
    Serial.println("Look for device: ESP32-LEGO");
    Serial.println("========================================\n");
}

void loop() {
    // Simple status indicator
    static unsigned long lastStatus = 0;
    if (millis() - lastStatus > 5000) {
        Serial.println("--- Status ---");
        Serial.println("Laptop: " + String(laptopConnected ? "CONNECTED" : "WAITING"));
        Serial.println("Test Value: " + String(testValue));
        Serial.println("LED Color: " + String(currentLedColor));
        Serial.println("Free Heap: " + String(ESP.getFreeHeap()));
        Serial.println("--------------");
        lastStatus = millis();
    }
    
    // Blink LED if not connected
    if (!laptopConnected) {
        static unsigned long lastBlink = 0;
        static bool blinkState = false;
        
        if (millis() - lastBlink > 1000) {
            blinkState = !blinkState;
            setRGBLed(blinkState ? 1 : 0); // Blink red
            lastBlink = millis();
        }
    }
    
    delay(100);
}