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

PositionController::PositionController(SpeedController* speedController) {
    this->speedController = speedController;
    this->stepper = speedController->stepper;
}

void PositionController::run() {
    if (this->s_target == this->stepper->step) {
        this->speedController->v_target = 0;
    }
}

void PositionController::setTarget(int32_t target, int16_t velocity, bool flag) {
    //以velocity速度运行到指定点
    int32_t diff = target - this->stepper->step;
    if (flag) {
        if (diff > 0) {
            this->speedController->v_target = velocity;
        } else if (diff < 0) {
            this->speedController->v_target = -velocity;
        } else {
            this->speedController->v_target = 0;
        }
    } else {
        this->speedController->v_target = velocity;
    }
    this->s_target = target;
}