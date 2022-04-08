#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
int i = 60;

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(40);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  if (i == 140){
    i = 60;
  }
  myservo.write(i);
  i++;
  delay(200); 
}
