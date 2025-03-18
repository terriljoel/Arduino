int left_intr = 0,right_intr = 0, angle=0;
float radius_of_wheel = 0.00825;  //Measure the radius of your wheel and enter it here in cm
volatile byte rotation;  // variale for interrupt fun must be volatile
float timetaken, rpm, dtime, v, distance;
unsigned long pevtime;
int flag = 1, bts = 0, btr = 0;

void setup()
{
  rotation = rpm = pevtime = 0;  //Initialize all variable to zero
  Serial.begin(9600);

  Serial.println("Vechile Monitor");  //Intro Message line 1
  delay(2000);
  attachInterrupt(digitalPinToInterrupt(2), Right_ISR, CHANGE);  //Left_ISR is called when left wheel sensor is triggered
}

void loop() {
  if (Serial.available() > 0 ) {
    char input = Serial.read();
    if (input == 'r') {
      right_intr = 0;
      flag = 1;
      distance=0;
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
  }
}

void Right_ISR()
{

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
