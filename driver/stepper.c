#include "stepper.h"
#include "driver.h"

//s的低32位为SPWM的单周期角度theta：-4~4
static inline fixedpt get_theta(int64_t s) {
    return (fixedpt)s;
}

//s的高32位代表SPWM的周期数
static inline int32_t get_cycle(int64_t s) {
    return s >> 32;
}

//步数是SPWM周期数的4倍
static inline int32_t get_step(int64_t s) {
    return get_cycle(s) << 2;
}

void run(Stepper* stepper) {
    stepper->v += stepper->a;
    if (stepper->v > MAX_V) {
        stepper->v = MAX_V;
    } else if (stepper->v < -MAX_V) {
        stepper->v = -MAX_V;
    }
    stepper->s += stepper->v;
    fixedpt theta = get_theta(stepper->s); 
    PWM a, b;
    get_output(theta, &a, &b);
    (stepper->out_a)(a.direction, a.duty);
    (stepper->out_b)(b.direction, b.duty);
}

