#ifndef _MY_STEPPER_H_
#define _MY_STEPPER_H_

#include <inttypes.h>

typedef void (*StepMethod)();

//以dt为周期调用run(), a 单位为 step/dt/dt, v 单位为 step/dt, s 单位为 step
//a,v,s均为定点数，范围为 -0.5 ~ 0.5
//每次运行run(), s = s + v * dt = s + v, 由于需要输出step信号，s 为 -0.5 ~ 0.5的范围

class MyStepper {
private:
    StepMethod stepP;
    StepMethod stepN;
public:
    int16_t a = 0;
    int16_t v = 0;
    int16_t s = 0;
    int32_t step = 0;
    MyStepper(StepMethod sp, StepMethod sn);
    void run();
};

class SpeedController {
private:
    int16_t a;          //加减速度绝对值,0为无加减速过程
public:
    MyStepper* stepper;
    int16_t v_target;   //目标速度
    SpeedController(MyStepper* stepper, int16_t a);
    void run();
};

//以给定速度运动到给定位置，保证最终位置到达
class PositionController {
public:
    MyStepper* stepper;
    SpeedController* speedController;
    int32_t s_target;   //目标位置
    PositionController(SpeedController* speedController);
    void run();
    void setTarget(int32_t target, int16_t velocity, bool flag = true);
};

#endif //_MY_STEPPER_H_
