#include <stdio.h>
#include "driver.h"

void get_output_test(fixedpt theta) {
    PWM pwm_a;
    PWM pwm_b;
    get_output(theta, &pwm_a, &pwm_b);
    printf("theta: %f \t A: %d, %d \t B: %d, %d\n", fixedpt_tof(theta), pwm_a.direction, pwm_a.duty, pwm_b.direction, pwm_b.duty);
}

void get_out_test(fixedpt theta) {
    fixedpt a = get_out(theta);
    printf("theta: %f,  out: %f \n", fixedpt_tof(theta), fixedpt_tof(a));
}

int main() {

    fixedpt theta = FIXEDPT(0);
    for (int i=0; i < 16; i++) {
        get_output_test(theta);
        theta+=fixedpt_formf(0.5);
    }

}

