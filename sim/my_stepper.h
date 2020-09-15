#ifndef _MY_STEPPER_H_
#define _MY_STEPPER_H_

#include <inttypes.h>

//以dt为周期调用run(), a 单位为 step/dt/dt, v 单位为 step/dt, s 单位为 step
//a,v,s均为定点数，范围为 -0.5 ~ 0.5
//每次运行run(), s = s + v * dt = s + v, 由于需要输出step信号，s 为 -0.5 ~ 0.5的范围

class MyStepper {
public:
    int16_t a = 0;
    int16_t v = 0;
    int16_t s = 0;
    int32_t step = 0;
    void run();
    void (* stepN)();
    void (* stepP)();
};

#endif //_MY_STEPPER_H_
