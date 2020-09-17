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

SpeedController::SpeedController(int16_t a) {
    this->a = a<0?-a:a;
}

void SpeedController::run(MyStepper* stepper) {
    int16_t v_diff = this->v_target - stepper->v;
    if(v_diff >= this->a) {
        stepper->a = this->a;
    } else if (v_diff <= (-this->a)) {
        stepper->a = -(this->a);
    } else {
        stepper->a = 0;
        stepper->v = this->v_target;
    }
}

PositionController::PositionController() {
    this->v_max = 0x7FF0;
}

PositionController::PositionController(int16_t v_max) {
    this->v_max = v_max;
}

void PositionController::run(MyStepper* stepper) {
    if (this->s_target = stepper->step) {
        stepper->v = 0;
    }
    stepper->run();
}