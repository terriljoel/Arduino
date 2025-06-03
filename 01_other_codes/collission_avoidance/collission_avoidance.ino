int count=0;
int flag=1;
int v=1;
void setup() {
Serial.begin (9600);
pinMode (9, INPUT); //Sensor output
}
void loop() {

if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == 's') {
      // readSensor = false;  // Stop reading sensor data
      Serial.println("Sensor reading stopped.");
      Serial.println("Sleeper Count");
      flag=0;
      Serial.println(count);

    } else if (input == 'r') {
      // readSensor = true;   // Resume reading sensor data
      Serial.println("Sensor reading resumed.");
      count = 0;
      flag=1;
      
    }
  }

  if(flag==1){
Serial.print ("Sensor: ");
v=digitalRead(9);
Serial.println (v); //print the sensor output}
  // if(v==1){
  //   flag=0;
  // }
  if(v==1){
    count++;
  }
  }
// delay (500); //wait half a second
}
// ******C