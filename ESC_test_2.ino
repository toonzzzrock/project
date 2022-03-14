#include <Servo.h>

Servo ESC1, ESC2, servo;     // create servo object to control the ESC
int potValue = 50;
char wait[] = "wait";
char start[] = "start";
void setup() {
  // Attach the ESC on pin 9v
  ESC1.attach(8,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
  ESC2.attach(7,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  servo.attach(3);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 1) {
     potValue = Serial.parseInt();    
  }
  Serial.println(potValue);
  
               // tell servo to go to position in variable 'pos'
  if (potValue >= 100) { ESC2.write(potValue); ESC1.write(potValue); servo.write(90); }
  else { ESC2.write(90); ESC1.write(90); servo.write(potValue);  }
  delay (500);
}
