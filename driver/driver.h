#ifndef DRIVER_H_
#define DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "fixedpt.h"

#define DUTY_BIT 10     //占空比位数
#define DUTY_BIT_DIFF (FIXEDPT_FBITS - DUTY_BIT)

typedef struct {
    int8_t direction;
    int32_t duty;
} PWM;

void get_output(fixedpt theta, PWM* pwm_a, PWM* pwm_b);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_H_ */
