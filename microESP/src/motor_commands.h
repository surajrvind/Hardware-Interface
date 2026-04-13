#include <Arduino.h>

#define LPWM 12
#define RPWM 13
#define L_EN 14
#define R_EN 27


void setupMotor() {
    pinMode(LPWM, OUTPUT);
    pinMode(RPWM, OUTPUT);
    pinMode(L_EN, OUTPUT);
    pinMode(R_EN, OUTPUT);

    digitalWrite(L_EN, HIGH);
    digitalWrite(R_EN, HIGH);
}

void setMotor(float throttle){
    int pwm = abs(throttle)*100;

    if (throttle > 0.05){        // Forward
        analogWrite(LPWM, pwm);
        analogWrite(RPWM, 0);
    }
    else if (throttle < -0.05){  // Reverse
        analogWrite(RPWM, pwm);
        analogWrite(LPWM, 0);
    }
    else {                       // Stop
        analogWrite(LPWM, 0);
        analogWrite(RPWM, 0);
    }
}