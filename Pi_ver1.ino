#include <Servo.h>

Servo myservo; 
Servo servo1;  
 // create servo object to control a servo
// twelve servo objects can be created on most boards
const int DC = 8; 
const int ServoPin = 9;  

int pos = 0;     // variable to store the servo position

void setup() { 
 
  myservo.attach(ServoPin); 
  pinMode(ServoPin, OUTPUT);
  servo1.attach(DC); 
  pinMode(DC, OUTPUT); 

   Serial.begin(9600); 
   Serial.flush(); 
   Serial.setTimeout(500); 
} 

void loop() {
   if (Serial.available() > 0) {
    servo_angle = Serial.parseInt();

    servo_angle = servo_angle + sum;
    if (servo_angle != 0) {
      myservo.write(servo_angle);
      SpeedValue = 1618;
      Serial.print(servo_angle);
      // servo1.write(SpeedValue);
    }
    else {
      myservo.write(90);
      servo1.write(1600);
}  
