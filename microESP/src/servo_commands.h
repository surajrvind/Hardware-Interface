#include <Arduino.h>
#include <ESP32Servo.h>

Servo myservo;
int servo = 25;

void setupServo(){
    ESP32PWM::allocateTimer(0);

    myservo.setPeriodHertz(50);
    myservo.attach(servo, 600, 2300);
}

void setSteering(float steering){
    int angle = 86 + steering*40;
    angle = constrain(angle, 40, 130);

    myservo.write(angle);
}
