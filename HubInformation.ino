#include "Lpf2Hub.h"
Lpf2Hub myTrainHub;
byte port = (byte)PoweredUpHubPort::A;
bool isHwVersionAvailable = false;
bool isFwVersionAvailable = false;
bool isBatteryTypeAvailable = false;

bool isInitialized = false;
void hubPropertyChangeCallback(void *hub, HubPropertyReference hubProperty, uint8_t *pData)
{
  Lpf2Hub *myHub = (Lpf2Hub *)hub;

  Serial.print("HubProperty: ");
  Serial.println((byte)hubProperty, HEX);

  if (hubProperty == HubPropertyReference::RSSI)
  {
    Serial.print("RSSI: ");
    Serial.println(myHub->parseRssi(pData), DEC);
    return;
  }

  if (hubProperty == HubPropertyReference::ADVERTISING_NAME)
  {
    Serial.print("Advertising Name: ");
    Serial.println(myHub->parseHubAdvertisingName(pData).c_str());
    return;
  }

  if (hubProperty == HubPropertyReference::BATTERY_VOLTAGE)
  {
    Serial.print("BatteryLevel: ");
    Serial.println(myHub->parseBatteryLevel(pData), DEC);
    return;
  }

  if (hubProperty == HubPropertyReference::BUTTON)
  {
    Serial.print("Button: ");
    Serial.println((byte)myHub->parseHubButton(pData), HEX);
    return;
  }

  if (hubProperty == HubPropertyReference::BATTERY_TYPE)
  {
    Serial.print("BatteryType: ");
    Serial.println(myHub->parseBatteryType(pData), HEX);
    isBatteryTypeAvailable=true;
    return;
  }

  if (hubProperty == HubPropertyReference::FW_VERSION)
  {
    Version version = myHub->parseVersion(pData);
    Serial.print("FWVersion: ");
    Serial.print(version.Major);
    Serial.print("-");
    Serial.print(version.Minor);
    Serial.print("-");
    Serial.print(version.Bugfix);
    Serial.print(" Build: ");
    Serial.println(version.Build);
    isFwVersionAvailable = true;
    return;
  }

  if (hubProperty == HubPropertyReference::HW_VERSION)
  {
    Version version = myHub->parseVersion(pData);
    Serial.print("HWVersion: ");
    Serial.print(version.Major);
    Serial.print("-");
    Serial.print(version.Minor);
    Serial.print("-");
    Serial.print(version.Bugfix);
    Serial.print(" Build: ");
    Serial.println(version.Build);
    isHwVersionAvailable = true;
    return;
  }
}

void portValueChangeCallback(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData)
{
  Lpf2Hub *myHub = (Lpf2Hub *)hub;

  if (deviceType == DeviceType::VOLTAGE_SENSOR)
  {
    double voltage = myHub->parseVoltageSensor(pData);
    Serial.print("Voltage: ");
    Serial.println(voltage, 2);
    return;
  }

  if (deviceType == DeviceType::CURRENT_SENSOR)
  {
    double current = myHub->parseCurrentSensor(pData);
    Serial.print("Current: ");
    Serial.println(current, 2);
    return;
  }

  if (deviceType == DeviceType::MOVE_HUB_TILT_SENSOR)
  {
    int x = myHub->parseBoostTiltSensorX(pData);
    int y = myHub->parseBoostTiltSensorY(pData);
    Serial.print("Tilt X: ");
    Serial.print(x, DEC);
    Serial.print(" Y: ");
    Serial.println(y, DEC);
  }
}


void setup() {
  Serial.begin(115200);
}

void loop() {

  if (!myTrainHub.isConnected() && !myTrainHub.isConnecting()) {
    myTrainHub.init();  // initalize the PoweredUpHub instance"9c:9a:c0:06:d1:80"
    //myTrainHub.init("90:84:2b:03:19:7f"); //example of initializing an hub with a specific address
  }

  if (myTrainHub.isConnecting()) {
    myTrainHub.connectHub();
    if (myTrainHub.isConnected() && !isInitialized) {
      Serial.println("Connected to HUB");
      Serial.print("Hub address: ");
      Serial.println(myTrainHub.getHubAddress().toString().c_str());
      Serial.print("Hub name: ");
      Serial.println(myTrainHub.getHubName().c_str());
      myTrainHub.activateHubPropertyUpdate(HubPropertyReference::ADVERTISING_NAME, hubPropertyChangeCallback);
      delay(50);
      myTrainHub.activateHubPropertyUpdate(HubPropertyReference::BATTERY_VOLTAGE, hubPropertyChangeCallback);
      delay(50);
      myTrainHub.activateHubPropertyUpdate(HubPropertyReference::BUTTON, hubPropertyChangeCallback);
      delay(50);
      myTrainHub.activateHubPropertyUpdate(HubPropertyReference::RSSI, hubPropertyChangeCallback);
      myTrainHub.activatePortDevice((byte)MoveHubPort::TILT, portValueChangeCallback);
      delay(50);
      myTrainHub.activatePortDevice((byte)MoveHubPort::CURRENT, portValueChangeCallback);
      delay(50);
      myTrainHub.activatePortDevice((byte)MoveHubPort::VOLTAGE, portValueChangeCallback);
      isInitialized = true;
    } else {
      Serial.println("Failed to connect to HUB");
    }
  }
  if (myTrainHub.isConnected() && !isFwVersionAvailable)
  {
    myTrainHub.requestHubPropertyUpdate(HubPropertyReference::FW_VERSION, hubPropertyChangeCallback);
    delay(100);
  }
  if (myTrainHub.isConnected() && !isHwVersionAvailable)
  {
    myTrainHub.requestHubPropertyUpdate(HubPropertyReference::HW_VERSION, hubPropertyChangeCallback);
    delay(100);
  }
  if (myTrainHub.isConnected() && !isBatteryTypeAvailable)
  {
    myTrainHub.requestHubPropertyUpdate(HubPropertyReference::BATTERY_TYPE, hubPropertyChangeCallback);
    delay(100);
  }

}
