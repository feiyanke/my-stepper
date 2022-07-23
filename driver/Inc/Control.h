#ifndef __CONTROL_H
#define __CONTROL_H



#ifdef __cplusplus
extern "C" {
#endif
    
#include "stm32f1xx_hal.h"


typedef struct
{
    float acc;
    float target_v;
} SpeedController;

/*typedef struct
{
    uint8_t tween;
    uint16_t total_frame;
    uint16_t current_frame;
    float target_s;
    float param1;    //用于存放参数
    float param2;
} FrameController;*/

typedef struct
{
    uint8_t tween;
    uint16_t total_ms;
    uint16_t current_ms;
    float target_s;
    float param1;    //用于存放参数
    float param2;
} TimeController;

typedef struct
{
    int32_t target; //step
    uint32_t dt;
    //float time;  //s
    //uint32_t current_time;
    float acc;
} PositionController;

#define TWEEN_SINE_IN_OUT   0
#define TWEEN_QUAD_IN_OUT   1
#define TWEEN_CUBIC_IN_OUT  2

#define RELATIVE       1
#define ABSOLUTE       0

/*__INLINE void MsTimerStart(void)
{
    TIM6->CNT = 0;
    TIM6->CR1|=(TIM_CR1_CEN); //开始
}*/

/*__INLINE void MsTimerEnd(void)
{
    TIM6->CR1&=(~TIM_CR1_CEN); //结束
}*/
    
#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_H */
