#include <stdio.h>
#include "fixedpt.h"

int main() {
    fixedpt a = FIXEDPT(3) + FIXEDPT(4);
    double b = fixedpt_tof(a);
    
    printf("%x, %f", a, b);
}

/**
 * @brief 根据角度w，以下函数值。w周期为-4~4
 * [-4,-3]     4+a
 * [-3,-1]    1
 * [-1,1]     -a
 * [1,3]      -1
 * [3,4]      -4+a，由于定点数的特性，实际上也等于 4+a
 * @param w 
 * @return fixedpt 
 */
fixedpt get_out(fixedpt w) {
    if (w <= FIXEDPT(1) && w >= FIXEDPT(-1)) {
        return -w;
    } else if (w <= FIXEDPT(3) && w >= FIXEDPT(1)) {
        return FIXEDPT(-1);
    } else if (w <= FIXEDPT(-1) && w >= -FIXEDPT(-3)) {
        return FIXEDPT(1);
    } else {
        return w + FIXEDPT(4);
    }
}

#define W_LIMIT FIXEDPT(2)

typedef struct {
    fixedpt w; //模拟的角速度，实际取值范围为 -W_LIMIT ~ W_LIMIT
    fixedpt a; //角度
} Stepper;

//每dt计算一次
void run(Stepper* stepper) {
    stepper->a += stepper->w;
    fixedpt ab_diff = stepper->w >= 0 ? FIXEDPT(2) : FIXEDPT(-2);
    fixedpt output_a = get_out(stepper->a); //A相输出
    fixedpt output_b = get_out(stepper->a + ab_diff); //A相输出
    output(stepper, output_a, output_b);
}

#define DUTY_BIT 10
#define DUTY_BIT_DIFF (FIXEDPT_FBITS - DUTY_BIT)

int8_t get_direct(fixedpt a) {
    return a >= 0;
}

int32_t get_duty(fixedpt a) {
    return  (a >= 0 ? a : -a) >> DUTY_BIT_DIFF;
}

void output(Stepper* stepper, fixedpt a, fixedpt b) {
    int8_t direction_a = get_direct(a);
    int32_t duty_a = get_duty(a);
    int8_t direction_b = get_direct(b);
    int32_t duty_b = get_duty(b);
    //输出两相的方向信号和PWM信号
}