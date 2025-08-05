#define VRX_PIN A0
#define VRY_PIN A1

#include <Servo.h>
Servo myservo2;
Servo myservo;
int xValue;
int yValue;

float angle;

void setup() {
  Serial.begin(9600);
  myservo.attach(3);
  myservo2.attach(5);
}

void loop() {
  // put your main code here, to run repeatedly:
  xValue = analogRead(VRX_PIN)-330;
  yValue = analogRead(VRY_PIN)-341;

  angle =atan2(xValue , yValue)/(2*M_PI)*360;
  
  Serial.print("x = ");
  Serial.print(xValue);
  Serial.print(", y = ");
  Serial.println(yValue);
  Serial.println(angle);
if (abs(xValue) > 100 ||abs(yValue) > 100 ){

  myservo.write(angle);
  myservo2.write(angle);
}
}
