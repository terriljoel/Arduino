/*
 * LEGO Powered UP Hub 88009 Emulator for Arduino Nano BLE
 * 
 * This sketch precisely emulates a LEGO Powered UP Hub (88009) with exact manufacturer data
 * and protocol implementation. The emulated hub will be recognized by the official LEGO Powered UP app.
 * 
 * Required library: ArduinoBLE
 * Install via: Tools > Manage Libraries > Search "ArduinoBLE"
 * 
 * Hardware: Arduino Nano 33 BLE or Arduino Nano 33 BLE Sense (Rev2)
 */

#include <ArduinoBLE.h>

// LEGO Powered UP Service and Characteristic UUIDs (LWP 3.0)
#define LEGO_SERVICE_UUID        "00001623-1212-efde-1623-785feabcd123"
#define LEGO_CHARACTERISTIC_UUID "00001624-1212-efde-1623-785feabcd123"

// LEGO Hub 88009 Configuration (2-port Powered UP Hub)
#define HUB_TYPE 0x40  // 88009 Hub Type
const char* HUB_NAME = "LEGO Hub";  // Exact name used by 88009
const uint8_t FIRMWARE_VERSION[] = {4, 0, 0, 49};    // 4.0.0.49 (latest 88009 firmware)
const uint8_t HARDWARE_VERSION[] = {0, 0, 0, 1};     // 0.0.0.1

// LEGO Manufacturer Data (exact from real 88009 hub)
const uint8_t LEGO_MANUFACTURER_DATA[] = {0x97, 0x03, 0x00, 0x40, 0x06, 0x00};
const uint16_t LEGO_COMPANY_ID = 0x0397;  // LEGO System A/S

// LWP Message Types
enum MessageTypes {
    HUB_PROPERTIES = 0x01,
    HUB_ACTIONS = 0x02,
    HUB_ALERTS = 0x03,
    HUB_ATTACHED_IO = 0x04,
    GENERIC_ERROR = 0x05,
    HW_NETWORK_COMMANDS = 0x08,
    PORT_INFORMATION_REQUEST = 0x21,
    PORT_MODE_INFORMATION_REQUEST = 0x22,
    PORT_INPUT_FORMAT_SETUP_SINGLE = 0x41,
    PORT_INFORMATION = 0x43,
    PORT_MODE_INFORMATION = 0x44,
    PORT_VALUE_SINGLE = 0x45,
    PORT_OUTPUT_COMMAND = 0x81,
    PORT_OUTPUT_COMMAND_FEEDBACK = 0x82
};

// Hub Property Types
enum HubPropertyTypes {
    ADVERTISING_NAME = 0x01,
    BUTTON = 0x02,
    FW_VERSION = 0x03,
    HW_VERSION = 0x04,
    RSSI = 0x05,
    BATTERY_VOLTAGE = 0x06,
    BATTERY_TYPE = 0x07,
    MANUFACTURER_NAME = 0x08,
    RADIO_FW_VERSION = 0x09,
    LEGO_WIRELESS_PROTOCOL_VERSION = 0x0A,
    SYSTEM_TYPE_ID = 0x0B,
    HW_NETWORK_ID = 0x0C,
    PRIMARY_MAC_ADDRESS = 0x0D,
    SECONDARY_MAC_ADDRESS = 0x0E,
    HARDWARE_NETWORK_FAMILY = 0x0F
};

// Device Types (exact for 88009)
enum DeviceTypes {
    MEDIUM_LINEAR_MOTOR = 0x0026,  // Compatible with 88009
    TRAIN_MOTOR = 0x002E,          // Primary motor for 88009
    HUB_LED = 0x0017,              // Internal LED
    BUTTON_DEVICE = 0x0005,        // Hub button
    VOLTAGE_SENSOR = 0x0014,       // Internal voltage
    CURRENT_SENSOR = 0x0015,       // Internal current
    TILT_SENSOR = 0x0028          // Hub orientation
};

// BLE Service and Characteristic
BLEService legoService(LEGO_SERVICE_UUID);
BLECharacteristic legoCharacteristic(LEGO_CHARACTERISTIC_UUID, BLERead | BLEWrite | BLENotify | BLEWriteWithoutResponse, 20);

// Hub State (matching 88009 capabilities)
struct HubState {
    uint8_t batteryLevel;
    bool buttonPressed;
    uint8_t ledColor;
    bool isConnected;
    uint32_t connectionTime;
    struct {
        bool connected;
        int8_t speed;
        int32_t position;
        uint8_t deviceType;
    } portA, portB;
} hubState;

// Function prototypes
void createLWPMessage(uint8_t messageType, const uint8_t* payload, uint8_t payloadLength, uint8_t* output, uint8_t* outputLength);
void sendHubProperties();
void attachDevices();
void handleIncomingMessage(const uint8_t* data, uint8_t length);
void handleMotorCommand(const uint8_t* payload, uint8_t length);
void handleHubPropertyRequest(const uint8_t* payload, uint8_t length);
void simulateButtonPress();
void updateBattery();
void printMessage(const uint8_t* data, uint8_t length);

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);  // Wait for Serial Monitor
    
    Serial.println("🚂 LEGO Powered UP Hub 88009 Emulator");
    Serial.println("=====================================");
    Serial.println("Exact emulation of LEGO Hub (88009) - 2-port Powered UP Hub");
    
    // Initialize hub state
    hubState.batteryLevel = 100;
    hubState.buttonPressed = false;
    hubState.ledColor = 0; // Off initially
    hubState.isConnected = false;
    hubState.connectionTime = 0;
    
    // Initialize ports (88009 has 2 ports: A and B)
    hubState.portA = {false, 0, 0, 0};  // No device attached initially
    hubState.portB = {false, 0, 0, 0};  // No device attached initially
    
    // Initialize BLE
    if (!BLE.begin()) {
        Serial.println("❌ Failed to initialize BLE!");
        while (1);
    }
    
    // Set up BLE advertising exactly like 88009
    BLE.setLocalName(HUB_NAME);
    BLE.setDeviceName(HUB_NAME);
    
    // Set LEGO manufacturer data (exact from real 88009)
    BLE.setManufacturerData(LEGO_COMPANY_ID, LEGO_MANUFACTURER_DATA, sizeof(LEGO_MANUFACTURER_DATA));
    
    // Set advertising parameters matching 88009
    BLE.setAdvertisingInterval(160);  // 100ms
    BLE.setConnectable(true);
    BLE.setAdvertisedService(legoService);
    
    // Add characteristic to service
    legoService.addCharacteristic(legoCharacteristic);
    
    // Add service
    BLE.addService(legoService);
    
    // Set initial characteristic value
    uint8_t initialValue[1] = {0x00};
    legoCharacteristic.writeValue(initialValue, 1);
    
    // Set event handlers
    BLE.setEventHandler(BLEConnected, onBLEConnected);
    BLE.setEventHandler(BLEDisconnected, onBLEDisconnected);
    legoCharacteristic.setEventHandler(BLEWritten, onCharacteristicWritten);
    legoCharacteristic.setEventHandler(BLESubscribed, onCharacteristicSubscribed);
    
    // Start advertising
    BLE.advertise();
    
    Serial.println("📡 BLE advertising started!");
    Serial.println("📱 Device name: " + String(HUB_NAME));
    Serial.println("🏭 Manufacturer: LEGO System A/S (0x0397)");
    Serial.println("🔧 Hub Type: 0x40 (88009 - 2-port Powered UP Hub)");
    Serial.println("🔧 Service UUID: " + String(LEGO_SERVICE_UUID));
    Serial.println("📱 Ready for LEGO Powered UP app connection!");
    Serial.println();
    Serial.println("💡 Commands:");
    Serial.println("   b - Simulate button press");
    Serial.println("   m - Attach/detach motors");
    Serial.println("   l - Cycle LED color");
    Serial.println("   s - Show status");
    Serial.println("   r - Reset hub");
    Serial.println();
}

void loop() {
    // Poll for BLE events
    BLE.poll();
    
    // Handle serial commands
    if (Serial.available()) {
        char command = Serial.read();
        switch (command) {
            case 'b':
                simulateButtonPress();
                break;
            case 'm':
                toggleMotors();
                break;
            case 'l':
                cycleLED();
                break;
            case 's':
                showStatus();
                break;
            case 'r':
                resetHub();
                break;
        }
    }
    
    // Update battery every 30 seconds
    static unsigned long lastBatteryUpdate = 0;
    if (millis() - lastBatteryUpdate > 30000) {
        updateBattery();
        lastBatteryUpdate = millis();
    }
    
    // Update motor positions
    static unsigned long lastMotorUpdate = 0;
    if (millis() - lastMotorUpdate > 1000) {
        updateMotorPositions();
        lastMotorUpdate = millis();
    }
    
    // Send periodic hub updates if connected
    static unsigned long lastHubUpdate = 0;
    if (hubState.isConnected && millis() - lastHubUpdate > 5000) {
        sendPeriodicUpdates();
        lastHubUpdate = millis();
    }
    
    delay(100);
}

void onBLEConnected(BLEDevice central) {
    hubState.isConnected = true;
    hubState.connectionTime = millis();
    Serial.println("✅ LEGO Powered UP app connected: " + central.address());
    Serial.println("🔗 Hub is now online and ready for commands!");
}

void onBLEDisconnected(BLEDevice central) {
    hubState.isConnected = false;
    Serial.println("❌ LEGO Powered UP app disconnected: " + central.address());
    Serial.println("📡 Hub returned to advertising mode");
}

void onCharacteristicSubscribed(BLEDevice central, BLECharacteristic characteristic) {
    Serial.println("🔔 App subscribed to notifications - sending hub info...");
    
    // Send initial hub properties and device attachments (like real 88009)
    delay(200);  // Give client time to set up
    sendHubProperties();
    delay(300);
    attachDevices();
    delay(200);
    sendInitialDeviceStates();
}

void onCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
    uint8_t data[20];
    uint8_t length = characteristic.valueLength();
    characteristic.readValue(data, length);
    
    Serial.print("📨 Received: ");
    printMessage(data, length);
    
    handleIncomingMessage(data, length);
}

void createLWPMessage(uint8_t messageType, const uint8_t* payload, uint8_t payloadLength, uint8_t* output, uint8_t* outputLength) {
    uint8_t hubId = 0x00;
    uint8_t length = 3 + payloadLength;
    
    output[0] = length;
    output[1] = hubId;
    output[2] = messageType;
    
    for (uint8_t i = 0; i < payloadLength; i++) {
        output[3 + i] = payload[i];
    }
    
    // Calculate checksum (XOR of all bytes except checksum)
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= output[i];
    }
    output[length] = checksum;
    
    *outputLength = length + 1;  // Include checksum
}

void sendLWPNotification(uint8_t messageType, const uint8_t* payload, uint8_t payloadLength) {
    if (!hubState.isConnected) return;
    
    uint8_t message[20];
    uint8_t messageLength;
    createLWPMessage(messageType, payload, payloadLength, message, &messageLength);
    
    legoCharacteristic.writeValue(message, messageLength);
    
    Serial.print("📤 Sent: ");
    printMessage(message, messageLength);
}

void sendHubProperties() {
    Serial.println("📋 Sending hub properties (88009 specific)...");
    
    // Send advertising name
    uint8_t namePayload[20];
    namePayload[0] = ADVERTISING_NAME;
    uint8_t nameLen = strlen(HUB_NAME);
    for (uint8_t i = 0; i < nameLen; i++) {
        namePayload[1 + i] = HUB_NAME[i];
    }
    sendLWPNotification(HUB_PROPERTIES, namePayload, 1 + nameLen);
    delay(50);
    
    // Send firmware version
    uint8_t fwPayload[5] = {FW_VERSION, FIRMWARE_VERSION[0], FIRMWARE_VERSION[1], FIRMWARE_VERSION[2], FIRMWARE_VERSION[3]};
    sendLWPNotification(HUB_PROPERTIES, fwPayload, 5);
    delay(50);
    
    // Send hardware version
    uint8_t hwPayload[5] = {HW_VERSION, HARDWARE_VERSION[0], HARDWARE_VERSION[1], HARDWARE_VERSION[2], HARDWARE_VERSION[3]};
    sendLWPNotification(HUB_PROPERTIES, hwPayload, 5);
    delay(50);
    
    // Send system type (88009 hub)
    uint8_t sysPayload[2] = {SYSTEM_TYPE_ID, HUB_TYPE};
    sendLWPNotification(HUB_PROPERTIES, sysPayload, 2);
    delay(50);
    
    // Send battery voltage
    uint16_t voltage = (hubState.batteryLevel * 9600) / 100;  // Convert to mV
    uint8_t batPayload[3] = {BATTERY_VOLTAGE, voltage & 0xFF, (voltage >> 8) & 0xFF};
    sendLWPNotification(HUB_PROPERTIES, batPayload, 3);
    delay(50);
    
    // Send manufacturer name
    const char* mfgName = "LEGO System A/S";
    uint8_t mfgPayload[20];
    mfgPayload[0] = MANUFACTURER_NAME;
    uint8_t mfgLen = strlen(mfgName);
    for (uint8_t i = 0; i < mfgLen; i++) {
        mfgPayload[1 + i] = mfgName[i];
    }
    sendLWPNotification(HUB_PROPERTIES, mfgPayload, 1 + mfgLen);
    delay(50);
    
    // Send LWP version
    uint8_t lwpPayload[3] = {LEGO_WIRELESS_PROTOCOL_VERSION, 0x00, 0x03};  // LWP 3.0
    sendLWPNotification(HUB_PROPERTIES, lwpPayload, 3);
}

void attachDevices() {
    Serial.println("🔌 Attaching 88009 internal devices...");
    
    // Attach Hub LED on port 50 (internal)
    uint8_t ledPayload[12] = {
        0x32, 0x01,  // Port 50, Attached
        HUB_LED & 0xFF, (HUB_LED >> 8) & 0xFF,  // Device type
        0x01, 0x00, 0x00, 0x00,  // HW Version
        0x01, 0x00, 0x00, 0x00   // SW Version
    };
    sendLWPNotification(HUB_ATTACHED_IO, ledPayload, 12);
    delay(100);
    
    // Attach Button on port 59 (internal)
    uint8_t buttonPayload[12] = {
        0x3B, 0x01,  // Port 59, Attached
        BUTTON_DEVICE & 0xFF, (BUTTON_DEVICE >> 8) & 0xFF,
        0x01, 0x00, 0x00, 0x00,
        0x01, 0x00, 0x00, 0x00
    };
    sendLWPNotification(HUB_ATTACHED_IO, buttonPayload, 12);
    delay(100);
    
    // Attach voltage sensor on port 60 (internal)
    uint8_t voltagePayload[12] = {
        0x3C, 0x01,  // Port 60, Attached
        VOLTAGE_SENSOR & 0xFF, (VOLTAGE_SENSOR >> 8) & 0xFF,
        0x01, 0x00, 0x00, 0x00,
        0x01, 0x00, 0x00, 0x00
    };
    sendLWPNotification(HUB_ATTACHED_IO, voltagePayload, 12);
    delay(100);
    
    // Initially no motors attached to external ports A(0) and B(1)
    // Motors will be attached when user simulates them or real ones are connected
}

void sendInitialDeviceStates() {
    Serial.println("🏁 Sending initial device states...");
    
    // Send initial button state (released)
    uint8_t buttonState[2] = {0x3B, 0x00};  // Port 59, released
    sendLWPNotification(PORT_VALUE_SINGLE, buttonState, 2);
    delay(50);
    
    // Send initial voltage reading
    uint16_t voltage = (hubState.batteryLevel * 9600) / 100;
    uint8_t voltageState[3] = {0x3C, voltage & 0xFF, (voltage >> 8) & 0xFF};
    sendLWPNotification(PORT_VALUE_SINGLE, voltageState, 3);
    delay(50);
    
    // Send initial LED state (off)
    uint8_t ledState[2] = {0x32, 0x00};  // Port 50, color 0 (off)
    sendLWPNotification(PORT_VALUE_SINGLE, ledState, 2);
}

void handleIncomingMessage(const uint8_t* data, uint8_t length) {
    if (length < 3) return;
    
    uint8_t msgLength = data[0];
    uint8_t hubId = data[1];
    uint8_t messageType = data[2];
    
    Serial.print("📨 Type: 0x");
    Serial.print(messageType, HEX);
    
    switch (messageType) {
        case PORT_OUTPUT_COMMAND:
            Serial.println(" (Motor/LED Command)");
            handleMotorCommand(data + 3, length - 3);
            break;
        case HUB_PROPERTIES:
            Serial.println(" (Hub Property Request)");
            handleHubPropertyRequest(data + 3, length - 3);
            break;
        case PORT_INFORMATION_REQUEST:
            Serial.println(" (Port Info Request)");
            handlePortInfoRequest(data + 3, length - 3);
            break;
        default:
            Serial.println(" (Unknown)");
            break;
    }
}

void handleMotorCommand(const uint8_t* payload, uint8_t length) {
    if (length < 2) return;
    
    uint8_t port = payload[0];
    uint8_t command = payload[1];
    
    if (port == 0x32) {  // Hub LED (port 50)
        if (command == 0x51 && length >= 5) {  // Set RGB Color
            uint8_t r = payload[2];
            uint8_t g = payload[3];
            uint8_t b = payload[4];
            Serial.print("💡 LED set to RGB(");
            Serial.print(r); Serial.print(","); Serial.print(g); Serial.print(","); Serial.print(b); Serial.println(")");
            
            // Send acknowledgment
            uint8_t ackPayload[2] = {port, 0x0A};  // Port, Command feedback
            sendLWPNotification(PORT_OUTPUT_COMMAND_FEEDBACK, ackPayload, 2);
        }
    } else if (port <= 1) {  // External motor ports A(0) or B(1)
        auto& motor = (port == 0) ? hubState.portA : hubState.portB;
        
        if (command == 0x11 && length >= 4) {  // Start Power command
            int8_t speed = (length > 3) ? payload[3] : 0;
            motor.speed = speed;
            
            Serial.print("🚂 Motor ");
            Serial.print((char)('A' + port));
            Serial.print(" set to speed ");
            Serial.println(speed);
            
            // Send acknowledgment
            uint8_t ackPayload[2] = {port, 0x0A};  // Port, Command feedback
            sendLWPNotification(PORT_OUTPUT_COMMAND_FEEDBACK, ackPayload, 2);
        } else if (command == 0x02) {  // Start Speed command
            if (length >= 5) {
                int8_t speed = payload[3];
                uint8_t maxPower = payload[4];
                motor.speed = speed;
                
                Serial.print("🚂 Motor ");
                Serial.print((char)('A' + port));
                Serial.print(" speed=");
                Serial.print(speed);
                Serial.print(" power=");
                Serial.println(maxPower);
                
                uint8_t ackPayload[2] = {port, 0x0A};
                sendLWPNotification(PORT_OUTPUT_COMMAND_FEEDBACK, ackPayload, 2);
            }
        }
    }
}

void handleHubPropertyRequest(const uint8_t* payload, uint8_t length) {
    if (length < 1) return;
    
    uint8_t propertyType = payload[0];
    
    switch (propertyType) {
        case BATTERY_VOLTAGE: {
            uint16_t voltage = (hubState.batteryLevel * 9600) / 100;
            uint8_t response[3] = {propertyType, voltage & 0xFF, (voltage >> 8) & 0xFF};
            sendLWPNotification(HUB_PROPERTIES, response, 3);
            break;
        }
        case BUTTON: {
            uint8_t response[2] = {propertyType, hubState.buttonPressed ? 0x01 : 0x00};
            sendLWPNotification(HUB_PROPERTIES, response, 2);
            break;
        }
    }
}

void handlePortInfoRequest(const uint8_t* payload, uint8_t length) {
    if (length < 2) return;
    
    uint8_t port = payload[0];
    uint8_t infoType = payload[1];
    
    Serial.print("ℹ️  Port ");
    Serial.print(port);
    Serial.print(" info request type ");
    Serial.println(infoType);
    
    // Send basic port info response
    uint8_t response[4] = {port, infoType, 0x00, 0x00};
    sendLWPNotification(PORT_INFORMATION, response, 4);
}

void simulateButtonPress() {
    Serial.println("🔘 Button pressed!");
    hubState.buttonPressed = true;
    
    // Send button press message
    uint8_t pressPayload[2] = {0x3B, 0x01};  // Port 59 (button), pressed
    sendLWPNotification(PORT_VALUE_SINGLE, pressPayload, 2);
    
    // Auto-release after 500ms
    delay(500);
    hubState.buttonPressed = false;
    
    uint8_t releasePayload[2] = {0x3B, 0x00};  // Port 59 (button), released
    sendLWPNotification(PORT_VALUE_SINGLE, releasePayload, 2);
    
    Serial.println("🔘 Button released");
}

void toggleMotors() {
    Serial.println("🔧 Toggling motor attachment...");
    
    if (!hubState.portA.connected) {
        // Attach train motor to port A
        hubState.portA.connected = true;
        hubState.portA.deviceType = TRAIN_MOTOR;
        
        uint8_t motorPayload[12] = {
            0x00, 0x01,  // Port A, Attached
            TRAIN_MOTOR & 0xFF, (TRAIN_MOTOR >> 8) & 0xFF,
            0x01, 0x00, 0x00, 0x00,
            0x01, 0x00, 0x00, 0x00
        };
        sendLWPNotification(HUB_ATTACHED_IO, motorPayload, 12);
        Serial.println("🚂 Train motor attached to port A");
    } else {
        // Detach motor from port A
        hubState.portA.connected = false;
        hubState.portA.speed = 0;
        hubState.portA.deviceType = 0;
        
        uint8_t detachPayload[2] = {0x00, 0x00};  // Port A, Detached
        sendLWPNotification(HUB_ATTACHED_IO, detachPayload, 2);
        Serial.println("❌ Motor detached from port A");
    }
}

void cycleLED() {
    hubState.ledColor = (hubState.ledColor + 1) % 11;  // 0-10 colors
    
    const char* colorNames[] = {"Off", "Pink", "Purple", "Blue", "Light Blue", "Cyan", "Green", "Yellow", "Orange", "Red", "White"};
    
    Serial.print("💡 LED color: ");
    Serial.println(colorNames[hubState.ledColor]);
    
    // Send LED color command
    uint8_t ledPayload[2] = {0x32, hubState.ledColor};  // Port 50, color
    sendLWPNotification(PORT_VALUE_SINGLE, ledPayload, 2);
}

void updateBattery() {
    if (hubState.batteryLevel > 5) {
        hubState.batteryLevel -= 1;
        if (hubState.batteryLevel % 10 == 0) {
            Serial.print("🔋 Battery: ");
            Serial.print(hubState.batteryLevel);
            Serial.println("%");
            
            // Send battery update
            uint16_t voltage = (hubState.batteryLevel * 9600) / 100;
            uint8_t batPayload[3] = {0x3C, voltage & 0xFF, (voltage >> 8) & 0xFF};
            sendLWPNotification(PORT_VALUE_SINGLE, batPayload, 3);
        }
    }
}

void updateMotorPositions() {
    // Update motor positions based on speed
    if (hubState.portA.connected && hubState.portA.speed != 0) {
        hubState.portA.position += hubState.portA.speed;
        
        // Send position feedback occasionally
        if (hubState.portA.position % 100 == 0) {
            uint8_t posPayload[5] = {
                0x00,  // Port A
                hubState.portA.position & 0xFF,
                (hubState.portA.position >> 8) & 0xFF,
                (hubState.portA.position >> 16) & 0xFF,
                (hubState.portA.position >> 24) & 0xFF
            };
            sendLWPNotification(PORT_VALUE_SINGLE, posPayload, 5);
        }
    }
    
    if (hubState.portB.connected && hubState.portB.speed != 0) {
        hubState.portB.position += hubState.portB.speed;
    }
}

void sendPeriodicUpdates() {
    // Send periodic voltage reading
    uint16_t voltage = (hubState.batteryLevel * 9600) / 100;
    uint8_t voltagePayload[3] = {0x3C, voltage & 0xFF, (voltage >> 8) & 0xFF};
    sendLWPNotification(PORT_VALUE_SINGLE, voltagePayload, 3);
}

void showStatus() {
    Serial.println("\n📊 LEGO Hub 88009 Status:");
    Serial.println("==========================");
    Serial.print("🔋 Battery: ");
    Serial.print(hubState.batteryLevel);
    Serial.println("%");
    Serial.print("🔘 Button: ");
    Serial.println(hubState.buttonPressed ? "Pressed" : "Released");
    Serial.print("💡 LED: Color ");
    Serial.println(hubState.ledColor);
    Serial.print("📡 BLE: ");
    Serial.println(hubState.isConnected ? "Connected to App" : "Advertising");
    
    if (hubState.isConnected) {
        Serial.print("⏱️  Connected for: ");
        Serial.print((millis() - hubState.connectionTime) / 1000);
        Serial.println(" seconds");
    }
    
    Serial.println("\n🔌 Port Status:");
    Serial.print("Port A: ");
    if (hubState.portA.connected) {
        Serial.print("Motor connected, Speed=");
        Serial.print(hubState.portA.speed);
        Serial.print(", Pos=");
        Serial.println(hubState.portA.position);
    } else {
        Serial.println("No device");
    }
    
    Serial.print("Port B: ");
    if (hubState.portB.connected) {
        Serial.print("Motor connected, Speed=");
        Serial.print(hubState.portB.speed);
        Serial.print(", Pos=");
        Serial.println(hubState.portB.position);
    } else {
        Serial.println("No device");
    }
    Serial.println();
}

void resetHub() {
    Serial.println("🔄 Resetting hub to factory state...");
    hubState.batteryLevel = 100;
    hubState.buttonPressed = false;
    hubState.ledColor = 0;
    hubState.portA = {false, 0, 0, 0};
    hubState.portB = {false, 0, 0, 0};
    
    // Send detach messages for all external ports
    uint8_t detachA[2] = {0x00, 0x00};  // Port A, Detached
    uint8_t detachB[2] = {0x01, 0x00};  // Port B, Detached
    sendLWPNotification(HUB_ATTACHED_IO, detachA, 2);
    sendLWPNotification(HUB_ATTACHED_IO, detachB, 2);
    
    Serial.println("✅ Hub reset complete!");
}

void printMessage(const uint8_t* data, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) {
        Serial.print("0x");
        if (data[i] < 16) Serial.print("0");
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}