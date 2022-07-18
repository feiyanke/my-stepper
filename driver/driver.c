#include <stdio.h>
#include "fixedpt.h"
#include "driver.h"

/**
 * @brief 根据角度w，以下函数值。w周期为-4~4
 * [-4,-3]     -4-a
 * [-3,-1]    1
 * [-1,1]     a
 * [1,3]      -1
 * [3,4]      4-a，由于定点数的特性，实际上也等于 4+a
 * @param w 
 * @return fixedpt 
 */
static fixedpt get_out(fixedpt w) {
    if (w <= FIXEDPT(1) && w >= FIXEDPT(-1)) {
        return w;
    } else if (w <= FIXEDPT(3) && w >= FIXEDPT(1)) {
        return FIXEDPT(1);
    } else if (w <= FIXEDPT(-1) && w >= FIXEDPT(-3)) {
        return FIXEDPT(-1);
    } else {
        return FIXEDPT(4) - w;
    }
}

static inline int8_t get_direct(fixedpt a) {
    return a >= 0;
}

static inline int32_t get_duty(fixedpt a) {
    return  (a >= 0 ? a : -a) >> DUTY_BIT_DIFF;
}

/**
 * @brief 根据输出幅值计算PWM参数（占空比和方向）
 * 
 * @param output 输出幅值
 * @param pwm PWM参数
 */
static void get_pwm(fixedpt output, PWM* pwm) {
    pwm->direction = get_direct(output);
    pwm->duty = get_duty(output);
}

/**
 * @brief 根据SPWM的角度，计算两相线圈的PWM参数
 * 
 * @param theta SPWM的角度
 * @param pwm_a A相PWM参数
 * @param pwm_b B相PWM参数
 */
void get_output(fixedpt theta, PWM* pwm_a, PWM* pwm_b) {
    fixedpt output_a = get_out(theta); //A相输出
    fixedpt output_b = get_out(theta + FIXEDPT(2)); //B相输出
    get_pwm(output_a, pwm_a);
    get_pwm(output_b, pwm_b);
}

