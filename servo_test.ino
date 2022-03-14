#include <Servo.h>

Servo myservo,ESC2;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(3);  // attaches the servo on pin 9 to the servo object
  ESC2.attach(7,1000,2000);
}

void loop() {

  ESC2.write(110);
}
