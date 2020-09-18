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
SpeedController speedControllerX(&stepperX, 0);
SpeedController speedControllerY(&stepperY, 0);
PositionController positionControllerX(&stepperX);
PositionController positionControllerY(&stepperY);

void run() {
    speedControllerX.run();
    stepperX.run();
    speedControllerY.run();
    stepperY.run();
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    MsTimer2::set(1, run); // 500ms period
    MsTimer2::start();
}
int8_t count = -1;
int8_t buffer[2];
int16_t x_target;
int16_t y_target;
int16_t v_max = 0x7FF0;

void loop() {
    //以最快v_max速度运行到指定点
    int16_t sx, sy;
    noInterrupts();
    sx = x_target;
    sy = y_target;
    interrupts();
    int32_t x_diff = sx - stepperX.step;
    int32_t y_diff = sy - stepperY.step;
    int32_t x_diff_abs = x_diff<0?-x_diff:x_diff;
    int32_t y_diff_abs = y_diff<0?-y_diff:y_diff;
    if (y_diff_abs > x_diff_abs) {
        if (y_diff > 0) {
            speedControllerY.v_target = v_max;
        } else if (y_diff < 0) {
            speedControllerY.v_target = -v_max;
        } else {
            speedControllerY.v_target = 0;
        }
        speedControllerX.v_target = x_diff * v_max / y_diff_abs;
    } else {
        if (x_diff > 0) {
            speedControllerX.v_target = v_max;
        } else if (x_diff < 0) {
            speedControllerX.v_target = -v_max;
        } else {
            speedControllerX.v_target = 0;
        }
        speedControllerY.v_target = y_diff * v_max / x_diff_abs;
    }

}


float v_theta_s = 50 * PI/180.0;
float v_theta_ms = v_theta_s / 1000;
float r = 100;
float rv = v_theta_ms * r * 0x10000;
float theta = 0;
int ms = 100;
void circle() {
    while (true) {
        int16_t vx = (int16_t)(rv * cos(theta));
        int16_t vy = (int16_t)(-rv * sin(theta));
        stepperX.v = vx;
        stepperY.v = vy;
        delay(ms);
        theta += v_theta_ms * ms;
    }
}



void serialEvent() {
    while (Serial.available()) {
        // get the new byte:
        char inChar = (char)Serial.read();
        if (count == -1 && inChar == 0x00) {
            count = 0;
        } else if (count == 0) {
            buffer[0] = inChar;
            count = 1;
        } else if (count == 1) {
            buffer[1] = inChar;
            x_target = buffer[0];
            y_target = buffer[1];
            count = -1;
        }
    }
}


