#include <Arduino.h>
#include <MsTimer2.h>
#include "my_stepper.h"

class TestStepper : public MyStepper {
public:
    void setDirN() override;
    void setDirP() override;
    void doStep() override;
};

void TestStepper::setDirN() {
//    Serial.println("setDirN");
}

void TestStepper::setDirP() {
//    Serial.println("setDirP");
}

void TestStepper::doStep() {
    Serial.println("doStep");
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    TestStepper stepper;
    stepper.a = 100;
    stepper.v = 200;
    stepper.s = 300;
    stepper.step = 400;
    uint32_t s = micros();
    stepper.run();
    uint32_t e = micros();
    uint32_t d = e - s;
    Serial.print("\nTime: ");
    Serial.print(d);
    Serial.println(" us");
    while (1);
}


