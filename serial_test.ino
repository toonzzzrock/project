#include <Servo.h>

Servo ESC1, ESC2, servo;     // create servo object to control the ESC

int spd = 90, ang = 90;
String inp;
char wait[] = "wait";
char start[] = "start";

void setup() {
  // Attach the ESC on pin 9v
  ESC1.attach(7,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
  ESC2.attach(8,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  servo.attach(5);
  Serial.begin(115200);
  Serial.setTimeout(1);
  
}

void loop() {
  if (Serial.available() > 1){
    inp = Serial.readStringUntil(',');
    spd = inp.substring(0, 3).toInt();
    ang = inp.substring(3).toInt();
    
  }/*
  Serial.print(spd);
  Serial.print('\t');
  Serial.print(ang);
  */
  servo.write(ang);
  ESC1.write(spd);
  ESC2.write(spd);
  delay (100);
}
