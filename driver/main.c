#include <stdio.h>
#include "driver.h"

// void get_output_test(fixedpt theta) {
//     PWM pwm_a;
//     PWM pwm_b;
//     get_output(theta, &pwm_a, &pwm_b);
//     printf("theta: %f \t A: %d, %d \t B: %d, %d\n", fixedpt_tof(theta), pwm_a.direction, pwm_a.duty, pwm_b.direction, pwm_b.duty);
// }

// void get_out_test(fixedpt theta) {
//     fixedpt a = get_out(theta);
//     printf("theta: %f,  out: %f \n", fixedpt_tof(theta), fixedpt_tof(a));
// }
typedef union {
    int64_t s;
    struct {
        fixedpt theta;
        int32_t c;
    };
} Theta;



int main() {

    fixedpt theta = FIXEDPT(-3);
    Theta t;
    t.s = 0;
    int64_t a = 0;
    for (size_t i = 0; i < 16; i++)
    {
        a += theta;
        t.s += theta;
        fixedpt w = a;
        printf("a: %lld ,  0x%llx,  w: %f \n", a>>FIXEDPT_FBITS, a, fixedpt_tof(w));
        printf("a: %lld ,  0x%llx,  w: %f \n", a>>FIXEDPT_FBITS, t.c, fixedpt_tof(t.theta));
    }
}

