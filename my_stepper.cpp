#include "my_stepper.h"
#include <math.h>

MyStepper::MyStepper(StepMethod sp, StepMethod sn) {
    this->stepP = sp;
    this->stepN = sn;
}

//每个dt调用,更新a,v,s, 如果需要输出step
void MyStepper::run()
{
    int16_t s = this->s;
    int16_t v = this->v;
    int16_t a = this->a;
    int16_t v1 = v + a;
    int16_t s1 = s + v;
    if(v>0)
    {
        if(s1<s)
        {
            this->stepP();
            this->step++;
        }
    }
    else
    {
        if(s1>s)
        {
            this->stepN();
            this->step--;
        }
    }
    this->s = s1;
    this->v = v1;
}

SpeedController::SpeedController(MyStepper* stepper, int16_t a) {
    this->a = a<0?-a:a;
    this->stepper = stepper;
}

void SpeedController::run() {
    if (this->a == 0) {
        this->stepper->a = 0;
        this->stepper->v = this->v_target;
        return;
    }
    int16_t v_diff = this->v_target - this->stepper->v;
    if(v_diff >= this->a) {
        this->stepper->a = this->a;
    } else if (v_diff <= (-this->a)) {
        this->stepper->a = -(this->a);
    } else {
        this->stepper->a = 0;
        this->stepper->v = this->v_target;
    }
}

PositionController::PositionController(MyStepper* stepper) {
    this->v_max = 0x7FF0;
    this->stepper = stepper;
}

PositionController::PositionController(MyStepper* stepper, int16_t v_max) {
    this->v_max = v_max;
    this->stepper = stepper;
}

void PositionController::run() {
    if (this->s_target = this->stepper->step) {
        this->stepper->v = 0;
    }
    this->stepper->run();
}

void PositionController::setTarget(int32_t target, int32_t duration) {
    int32_t diff = (target - this->stepper->step)<<16;
    int16_t v = diff/duration;
    if (v>v_max) {
        v = v_max;
    } else if (v<-v_max) {
        v = -v_max;
    }
    this->stepper->v = v;
    this->s_target = target;
}