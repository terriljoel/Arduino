/*
  Battery Monitor

  This example creates a Bluetooth® Low Energy peripheral with the standard battery service and
   characteristic. The A0 pin is used to calculate the battery .

  The circuit:
  - Arduino MKR WiFi 1010, Arduino Uno WiFi Rev2 board, Arduino Nano 33 IoT,
    Arduino Nano 33 BLE, or Arduino Nano 33 BLE Sense board.

  You can use a generic Bluetooth® Low Energy central app, like LightBlue (iOS and Android) or
  nRF Connect (Android), to interact with the services and characteristics
  created in this sketch.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>

#include <Arduino_LSM9DS1.h>

#define MINIMUM_TILT 5  // Threshold for tilt detection in degrees
#define WAIT_TIME 500   // How often to run the code (in milliseconds)


const char *deviceServiceUuid = "00001623-1212-efde-1623-785feabcd123";
const char *deviceServiceCharacteristicUuid = "00001624-1212-efde-1623-785feabcd123";

// const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
// const char* deviceServiceCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";
float x, y, z;
int angleX = 0;
int angleY = 0;
unsigned long previousMillis = 0;
int tilt = -1;
int oldTiltValue = -1;
// Bluetooth® Low Energy Battery Service
BLEService wheelRotationService(deviceServiceUuid);

// Bluetooth® Low Energy wheelRotation  Characteristic
BLECharacteristic wheelRotationChar(deviceServiceCharacteristicUuid,  // standard 16-bit characteristic UUID
                                    BLERead | BLENotify, 4);          // remote clients will be able to get notifications if this characteristic changes
BLECharacteristic angleYChar("00001625-1212-efde-1623-785feabcd123", BLERead | BLENotify, 4);
// int oldwheelRotation = 0;  // last wheelRotation  reading from analog input
// long previousMillis = 0;  // last time the wheelRotation  was checked, in ms
int32_t val32 = 0;
void setup() {
  Serial.begin(9600);  // initialize serial communication
  while (!Serial)
    ;

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }
  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1)
      ;
  }

  /* Set a local name for the Bluetooth® Low Energy device
     This name will appear in advertising packets
     and can be used by remote devices to identify this Bluetooth® Low Energy device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName("RotationSensor");
  BLE.setAdvertisedService(wheelRotationService);  // add the service UUID
  wheelRotationService.addCharacteristic(wheelRotationChar);
  wheelRotationService.addCharacteristic(angleYChar);
  // add the wheelRotation  characteristic
  BLE.addService(wheelRotationService);                               // Add the wheelRotation service
  wheelRotationChar.writeValue((const void *)&val32, sizeof(val32));  // set initial value for this characteristic

  /* Start advertising Bluetooth® Low Energy.  It will start continuously transmitting Bluetooth® Low Energy
     advertising packets and will be visible to remote Bluetooth® Low Energy central devices
     until it receives a new connection */

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");
}

void loop() {
  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    // check the wheelRotation  every 200ms
    // while the central is connected:
    while (central.connected()) {
      // if 200ms have passed, check the wheelRotation :
      int val = updatewheelRotation();
      // wheelRotationChar.writeValue((byte)val);
      Serial.print("writing value ");
      Serial.println(val);
      val32 = (int32_t)val;
      wheelRotationChar.writeValue((const void *)&val32, sizeof(val32));
      delay(500); 
      Serial.println(" writing val 2");
      angleYChar.writeValue((const void *)&val32, sizeof(val32));
      delay(500);
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

// void updatewheelRotation() {
//   /* Read the current voltage  on the A0 analog input pin.
//      This is used here to simulate the charge  of a wheelRotation.
//   */
//   int wheelRotation = 0;

//   if (wheelRotation != oldwheelRotation) {      // if the wheelRotation  has changed
//     Serial.print("wheelRotation  % is now: "); // print it
//     Serial.println(wheelRotation);
//     wheelRotationChar.writeValue(wheelRotation);  // and update the wheelRotation  characteristic
//     oldwheelRotation = wheelRotation;           // save the  for next comparison
//   }
// }


int updatewheelRotation() {
  unsigned long currentMillis = millis();

  if (IMU.accelerationAvailable() && (currentMillis - previousMillis >= WAIT_TIME)) {
    previousMillis = currentMillis;

    IMU.readAcceleration(x, y, z);

    // Calculate tilt angles in degrees
    angleX = atan2(x, sqrt(y * y + z * z)) * 180 / PI;
    angleY = atan2(y, sqrt(x * x + z * z)) * 180 / PI;
    return angleY;
    // Determine the tilting direction based on angleX and angleY
    // if (angleX > MINIMUM_TILT) {  // Tilting up
    //   Serial.print("Tilting up ");
    //   Serial.print(angleX);
    //   Serial.println(" degrees");
    //   return 0;
    // } else if (angleX < -MINIMUM_TILT) {  // Tilting down
    //   Serial.print("Tilting down ");
    //   Serial.print(-angleX);
    //   Serial.println(" degrees");
    //   return 1;
    // }

    // if (angleY > MINIMUM_TILT) {  // Tilting left
    //   Serial.print("Tilting left ");
    //   Serial.print(angleY);
    //   Serial.println(" degrees");
    //   return 2;
    // } else if (angleY < -MINIMUM_TILT) {  // Tilting right
    //   Serial.print("Tilting right ");
    //   Serial.print(-angleY);
    //   Serial.println(" degrees");
    //   return 3;
    // }
  }
}
