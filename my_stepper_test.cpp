#include <Arduino.h>
#include <MsTimer2.h>
#include "my_stepper.h"

void serialStepXP() {
    Serial.write('1');
}

void serialStepXN() {
    Serial.write('2');
}

void serialStepYP() {
    Serial.write('3');
}

void serialStepYN() {
    Serial.write('4');
}

MyStepper stepperX(serialStepXP, serialStepXN);
MyStepper stepperY(serialStepYP, serialStepYN);
PositionController positionControllerX;
PositionController positionControllerY;

void run() {
    positionControllerX.run(&stepperX);
    positionControllerY.run(&stepperY);
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    MsTimer2::set(1, run); // 500ms period
    MsTimer2::start();
}

float v_theta_s = 50 * PI/180.0;
float v_theta_ms = v_theta_s / 1000;
float r = 100;
float rv = v_theta_ms * r * 0x10000;
float theta = 0;
int ms = 100;
void loop() {
    while (true) {
        int16_t vx = (int16_t)(rv * cos(theta));
        int16_t vy = (int16_t)(-rv * sin(theta));
        stepperX.v = vx;
        stepperY.v = vy;
        delay(ms);
        theta += v_theta_ms * ms;
    }
}


