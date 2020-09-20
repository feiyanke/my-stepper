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
PositionController positionControllerX(&speedControllerX);
PositionController positionControllerY(&speedControllerY);

void run() {
    positionControllerX.run();
    speedControllerX.run();
    stepperX.run();
    positionControllerY.run();
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
uint8_t buffer[4];
bool update= true;
int16_t x_target = 0;
int16_t y_target = 0;
int16_t v_max = 0x7FF0;
char p[32];

void loop1() {
    if (!update) {
        return;
    }
    update = false;
    int16_t sx, sy;
    noInterrupts();
    sx = x_target;
    sy = y_target;
    interrupts();
    sprintf(p, "target: %d, %d\n", sx, sy);
    Serial.print(p);
}

void loop() {
    if (!update) {
        return;
    }
    update = false;
    //以最快v_max速度运行到指定点
    int16_t sx, sy;
    noInterrupts();
    sx = x_target;
    sy = y_target;
    interrupts();
    int32_t x_diff = sx - stepperX.step;
    int32_t y_diff = sy - stepperY.step;
    int32_t x_diff_abs = x_diff<0?(-x_diff):x_diff;
    int32_t y_diff_abs = y_diff<0?(-y_diff):y_diff;
    if (y_diff_abs > x_diff_abs) {
//        if (y_diff > 0) {
//            speedControllerY.v_target = v_max;
//        } else if (y_diff < 0) {
//            speedControllerY.v_target = -v_max;
//        } else {
//            speedControllerY.v_target = 0;
//        }
        positionControllerY.setTarget(sy, v_max);
        int16_t vx = x_diff * v_max / y_diff_abs;
        positionControllerX.setTarget(sx, x_diff * v_max / y_diff_abs, false);
//        speedControllerX.v_target = x_diff * v_max / y_diff_abs;
    } else {
//        if (x_diff > 0) {
//            speedControllerX.v_target = v_max;
//        } else if (x_diff < 0) {
//            speedControllerX.v_target = -v_max;
//        } else {
//            speedControllerX.v_target = 0;
//        }
//        speedControllerY.v_target = y_diff * v_max / x_diff_abs;
        positionControllerX.setTarget(sx, v_max);
        positionControllerY.setTarget(sy, y_diff * v_max / x_diff_abs, false);
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
        uint8_t inChar = (uint8_t)Serial.read();
        if (count == -1 && inChar == 0x00) {
            count = 0;
        } else if (count < 3) {
            buffer[count] = inChar;
            count++;
        } else if (count == 3) {
            buffer[3] = inChar;
            x_target = buffer[0] + (buffer[1]<<8);
            y_target = buffer[2] + (buffer[3]<<8);
            update = true;
            count = -1;
        } else {
            count = -1;
        }
    }
}


