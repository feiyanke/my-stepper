#ifndef STEPPER_H_
#define STEPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "fixedpt.h"

#define MAX_V FIXEDPT(2)    //最大速度，实际上就是是 1 step/dt
#define MAX_A fixedpt_formf(0.2)

typedef struct {
    fixedpt a; //加速度
    fixedpt v; //速度
    int64_t s; //位置，绝对位置，实际上参与计算的是低32位
    void (*out_a)(int8_t dir, int32_t duty);
    void (*out_b)(int8_t dir, int32_t duty);
} Stepper;

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_H_ */
