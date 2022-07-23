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
#define FIXEDPT_WBITS	0		//��������λ�������������ã���ϵ�ĵ���������ȡֵ��Χ��С�����־���
#endif
#define FIXEDPT_FBITS	(FIXEDPT_BITS - FIXEDPT_WBITS)		//С������λ��
#define FIXEDPT_FMASK	((1 << FIXEDPT_FBITS) - 1)

#define fixedpt_rconst(R) (int32_t)(R * (1LL << FIXEDPT_FBITS) + (R >= 0 ? 0.5 : -0.5))		//�����Ķ��㻯
#define fixedpt_fromint(I) ((int64_t)I << FIXEDPT_FBITS)	//�����Ķ��㻯
#define fixedpt_toint(F) (F >> FIXEDPT_FBITS)	//��������������תΪINT����
#define fixedpt_add(A,B) (A + B)	//�������ӷ�
#define fixedpt_sub(A,B) (A - B)	//����������
#define fixedpt_xmul(A,B) (int32_t)(((int64_t)A * (int64_t)B) >> FIXEDPT_FBITS)		//�������˷�
#define fixedpt_xdiv(A,B) (int32_t)(((int64_t)A << FIXEDPT_FBITS) / (int64_t)B)		//����������
#define fixedpt_fracpart(A) (A & FIXEDPT_FMASK)		//��������С������

#define FIXEDPT_ONE	(int32_t)(1 << FIXEDPT_FBITS)	//��������1
#define FIXEDPT_ONE_HALF (FIXEDPT_ONE >> 1)		//��������0.5
#define FIXEDPT_TWO	(FIXEDPT_ONE + FIXEDPT_ONE)		//��������2
#define FIXEDPT_PI	fixedpt_rconst(3.14159265)		//��������PI
#define FIXEDPT_TWO_PI	fixedpt_rconst(2*3.14159265)	//��������2*PI
#define FIXEDPT_HALF_PI fixedpt_rconst(3.14159265/2)	//��������0.5*PI
#define FIXEDPT_E	fixedpt_rconst(2.71828183)		//��������E

#define fixedpt_abs(A) (A < 0 ? -A : A)		//�������ľ���ֵ

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
