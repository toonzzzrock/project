#include <Servo.h>
Servo servo;

void setup() {
  // initialize serial:
  Serial.begin(9600); // set serial monitor baud rate to match
                  // set serial monitor line ending to Newline
  servo.write(90);
  servo.attach(3);
  prntIt();
}

void loop() {
  // if there's any serial available, read it:
  while (Serial.available() > 0) {

    // look for the next valid integer in the incoming serial stream:
    int pos = Serial.parseInt();
    // look for the newline. That's the end of your sentence:
    servo.write(pos);
    prntIt();
  }
}
void prntIt()
{
  Serial.print("  degrees = "); 
  Serial.print(servo.read());
  Serial.print("\t");
  Serial.print("microseconds =  ");
  Serial.println(servo.readMicroseconds());
}
