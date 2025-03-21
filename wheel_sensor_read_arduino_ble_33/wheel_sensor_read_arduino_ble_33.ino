
#include <ArduinoBLE.h>
int right_intr = 0, angle = 0;
float radius_of_wheel = 0.00825;  //Measure the radius of your wheel and enter it here in cm
volatile byte rotation;           // variale for interrupt fun must be volatile
float timetaken, rpm, dtime, v, distance;
unsigned long pevtime;
int flag = 1;

const char *deviceServiceUuid = "00001623-1212-efde-1623-785feabcd123";
const char *deviceServiceCharacteristicUuid1 = "00001624-1212-efde-1623-785feabcd123";
const char *deviceServiceCharacteristicUuid2 = "00001625-1212-efde-1623-785feabcd123";
const char *deviceServiceCharacteristicUuid3 = "00001626-1212-efde-1623-785feabcd123";

BLEService wheelRotationService(deviceServiceUuid);

// Bluetooth® Low Energy wheelRotation  Characteristic
BLECharacteristic distanceChar(deviceServiceCharacteristicUuid1,  // standard 16-bit characteristic UUID
                               BLERead | BLENotify, 4);           // remote clients will be able to get notifications if this characteristic changes
BLECharacteristic speedChar(deviceServiceCharacteristicUuid2, BLERead | BLENotify, 4);
BLECharacteristic rpmChar(deviceServiceCharacteristicUuid3, BLERead | BLENotify, 4);


void setup() {

  Serial.begin(9600);  // initialize serial communication
  // while (!Serial)
  //   ;
  rotation = rpm = pevtime = 0;  //Initialize all variable to zero

  Serial.println("Speed Sensor");  //Intro Message line 1
  delay(2000);
  attachInterrupt(digitalPinToInterrupt(2), Right_ISR, CHANGE);  //Left_ISR is called when left wheel sensor is triggered

  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1)
      ;
  }

  BLE.setLocalName("RotationSensor");
  BLE.setAdvertisedService(wheelRotationService);  // add the service UUID
  wheelRotationService.addCharacteristic(distanceChar);
  wheelRotationService.addCharacteristic(speedChar);
  wheelRotationService.addCharacteristic(rpmChar);
  BLE.addService(wheelRotationService);  // Add the wheelRotation service
  // wheelRotationChar.writeValue((const void *)&val32, sizeof(val32));  // set initial value for this characteristic

  /* Start advertising Bluetooth® Low Energy.  It will start continuously transmitting Bluetooth® Low Energy
     advertising packets and will be visible to remote Bluetooth® Low Energy central devices
     until it receives a new connection */

  // start advertising
  BLE.advertise();
}

void loop() {


  BLEDevice central = BLE.central();


  if (central) {

    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    // check the wheelRotation  every 200ms
    // while the central is connected:
    while (central.connected()) {
      if (Serial.available() > 0) {
        char input = Serial.read();
        if (input == 'r') {
          right_intr = 0;
          flag = 1;
          distance = 0;
          Serial.println("count reset");
        } else if (input == 's') {
          // float wheel_circumference = 2 * 3.14159 * radius_of_wheel;  // Wheel circumference in meters
          // float distance = wheel_circumference * (wheel_count / 40.0);
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
      if (flag) {
        if (millis() - dtime > 500)  //no inetrrupt found for 500ms
        {
          rpm = v = 0;  // make rpm and velocity as zero
          dtime = millis();
        }
        v = radius_of_wheel * rpm * 0.1047;  //0.033 is the radius of the wheel in meter
        distance = (2 * 3.141 * radius_of_wheel) * (right_intr / 4.0);

        Serial.print("RPM- ");
        Serial.print(rpm);
        Serial.print("  Velocity- ");
        Serial.print(v);
        Serial.print("  Rotation- ");
        Serial.print(right_intr / 4);
        Serial.print("  Distance- ");
        Serial.print(distance);
        Serial.print("  Grid Count- ");
        Serial.println(right_intr);
        delay(10);

        Serial.print("* Writing Distnace: ");
        Serial.print(distance);
        distanceChar.writeValue((const void *)&distance, sizeof(distance));
        Serial.print("   Speed: ");
        Serial.print(v);
        speedChar.writeValue((const void *)&v, sizeof(v));
        Serial.print("   Rpm: ");
        Serial.println(rpm);
        rpmChar.writeValue((const void *)&rpm, sizeof(rpm));
        delay(500);
      }
      // when the central disconnects, turn off the LED:
      
    }
    digitalWrite(LED_BUILTIN, LOW);
      Serial.print("Disconnected from central: ");
      Serial.println(central.address());
  }
}
  void Right_ISR() {

    right_intr++;
    // delay(10);
    rotation++;
    dtime = millis();

    if (rotation >= 4) {
      timetaken = millis() - pevtime;  //timetaken in millisec
      rpm = (1000 / timetaken) * 60;   //formulae to calculate rpm
      pevtime = millis();
      rotation = 0;
    }
  }
