/*
 * fixedpt.h
 *
 *  Created on: 2011-3-31
 *      Author: feiyanke
 */
#ifndef FIXEDPT_H_
#define FIXEDPT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

typedef int32_t fixedpt;

//#define fixedpt int32_t;

/* Actually, you can redefine the FIXEDPT_WBITS constant to support other
 * divisions of the 32-bit integer, but who wants to work with 16-bit integers
 * these days? :) */

#define FIXEDPT_BITS	32
#ifndef FIXEDPT_WBITS
#define FIXEDPT_WBITS	0		//整数部分位数，可任意配置，关系的到定点数的取值范围和小数部分精度
#endif
#define FIXEDPT_FBITS	(FIXEDPT_BITS - FIXEDPT_WBITS)		//小数部分位数
#define FIXEDPT_FMASK	((1 << FIXEDPT_FBITS) - 1)

#define fixedpt_rconst(R) (int32_t)(R * (1LL << FIXEDPT_FBITS) + (R >= 0 ? 0.5 : -0.5))		//常数的定点化
#define fixedpt_fromint(I) ((int64_t)I << FIXEDPT_FBITS)	//整数的定点化
#define fixedpt_toint(F) (F >> FIXEDPT_FBITS)	//定点数整数部分转为INT类型
#define fixedpt_add(A,B) (A + B)	//定点数加法
#define fixedpt_sub(A,B) (A - B)	//定点数减法
#define fixedpt_xmul(A,B) (int32_t)(((int64_t)A * (int64_t)B) >> FIXEDPT_FBITS)		//定点数乘法
#define fixedpt_xdiv(A,B) (int32_t)(((int64_t)A << FIXEDPT_FBITS) / (int64_t)B)		//定点数除法
#define fixedpt_fracpart(A) (A & FIXEDPT_FMASK)		//定点数的小数部分

#define FIXEDPT_ONE	(int32_t)(1 << FIXEDPT_FBITS)	//定点数的1
#define FIXEDPT_ONE_HALF (FIXEDPT_ONE >> 1)		//定点数的0.5
#define FIXEDPT_TWO	(FIXEDPT_ONE + FIXEDPT_ONE)		//定点数的2
#define FIXEDPT_PI	fixedpt_rconst(3.14159265)		//定点数的PI
#define FIXEDPT_TWO_PI	fixedpt_rconst(2*3.14159265)	//定点数的2*PI
#define FIXEDPT_HALF_PI fixedpt_rconst(3.14159265/2)	//定点数的0.5*PI
#define FIXEDPT_E	fixedpt_rconst(2.71828183)		//定点数的E

#define fixedpt_abs(A) (A < 0 ? -A : A)		//定点数的绝对值

static inline fixedpt fixedpt_formf(float a)
{
    return (fixedpt)(a * (1LL << FIXEDPT_FBITS) + (a >= 0 ? 0.5 : -0.5));
}

static inline int32_t fixedpt_mul(fixedpt A, fixedpt B)
{
	return (((int64_t)A * (int64_t)B) >> FIXEDPT_FBITS);
}

static inline int32_t fixedpt_div(fixedpt A, fixedpt B)
{
	return (((int64_t)A << FIXEDPT_FBITS) / (int64_t)B);
}

#ifdef __cplusplus
}
#endif

#endif /* FIXEDPT_H_ */
