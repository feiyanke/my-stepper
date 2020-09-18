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
    int16_t a;          //加减速度绝对值
public:
    int16_t v_target;   //目标速度
    SpeedController(int16_t a);
    void run(MyStepper* s);
};

//以恒定速度，在给定时间内运动到给定位置，保证最终位置到达，最大程度保证时间
class PositionController {
public:
    int32_t s_target;   //目标位置
    int32_t duration;   //给定时间,单位dt
    int16_t v_max;      //最大速度(绝对值)
    PositionController();
    PositionController(int16_t v_max);
    void run(MyStepper* stepper);
};

#endif //_MY_STEPPER_H_
