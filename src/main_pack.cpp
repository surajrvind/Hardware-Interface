#include <Arduino.h>
#include "steering.h"
#include "motor_commands.h"

void setup(){
    setupMotor();
    setupServo();
    Serial.begin(115200);
    Serial.setTimeout(10);
    setSteering(0.0);
    setThrottle(0);
    updateMotor();
}

void loop(){

    if (Serial.available() > 0){
    String line = Serial.readStringUntil('\n');
    line.trim();
    Serial.println(line);

    if(!line.startsWith("@")){
        return;
    }
    line = line.substring(1); // Remove '@'
    int last_comma = line.lastIndexOf(',');
    if (last_comma < 0){
        return;
    }

    String payload = line.substring(0, last_comma);
    String checksum_str = line.substring(last_comma + 1);

    int received_checksum = checksum_str.toInt();

    uint8_t computed_checksum = 0;
    for (size_t i = 0; i < payload.length(); i++){
        computed_checksum += payload[i];
    }
    if (computed_checksum != received_checksum){
        Serial.println("Checksum mismatch");
        return;
    
    if (payload.startsWith("C")){
        Serial.println("Received control command");
        int comma1 = payload.indexOf(',');
        int comma2 = payload.indexOf(',', comma1 + 1);
        if (comma1 < 0 || comma2 < 0) return;

        int throttle = payload.substring(comma1 + 1, comma2).toInt();
        float steering = payload.substring(comma2 + 1).toFloat();

        setThrottle(throttle);
        setSteering(steering);

        Serial.print("OK: ");
        Serial.print(throttle);
        Serial.print(",");
        Serial.println(steering);
    }
    }      
    }
updateMotor();
}
