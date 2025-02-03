#include <Servo.h>

Servo myservo;
int angle = 90;

void setup() {
  Serial.begin(9600);
  myservo.attach(9);
  myservo.write(angle);
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    int newAngle = input.toInt();

    if (newAngle >= 0 && newAngle <= 180 && newAngle % 5 == 0) {
      angle = newAngle;
      myservo.write(angle);
    }
  }
  delay(50);
}