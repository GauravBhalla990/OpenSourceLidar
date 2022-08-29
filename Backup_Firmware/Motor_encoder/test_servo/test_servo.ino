#include <Servo.h>

Servo myservo;  // create servo object to control a servo


void setup() {
  Serial.begin(9600);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  delay(1);
  myservo.write(10);                  // sets the servo position according to the scaled value
  delay(5000);

}
int val  = 0;
void loop() {
myservo.write(50);

//  while(Serial.available() > 0){
//   val  = Serial.parseInt();
//    Serial.println(val);
//    myservo.write(val);
//  }
  
}
