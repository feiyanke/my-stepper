#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif
    
#include "stm32f1xx_hal.h"
#include "fixedpt.h"
#include "config.h"
    
//config
/*
MOTOR0_CLK        PA15      TIM2_CH1
MOTOR0_SLEEP      PB3
MOTOR0_DIR        PB4
MOTOR0_EN         PB7
MOTOR0_M0         PB5
MOTOR0_M1         PB6
MOTOR0_ORG        PB9
*/
    
#define MOTOR0_STEP_TIM     TIM2
#define MOTOR0_STEP_PORT    GPIOA
#define MOTOR0_STEP_PIN     GPIO_PIN_15
#define MOTOR0_DIR_PORT     GPIOB
#define MOTOR0_DIR_PIN      GPIO_PIN_4
#define MOTOR0_SLEEP_PORT   GPIOB
#define MOTOR0_SLEEP_PIN    GPIO_PIN_3
#define MOTOR0_EN_PORT      GPIOB
#define MOTOR0_EN_PIN       GPIO_PIN_7
#define MOTOR0_M0_PORT      GPIOB
#define MOTOR0_M0_PIN       GPIO_PIN_5
#define MOTOR0_M1_PORT      GPIOB
#define MOTOR0_M1_PIN       GPIO_PIN_6
#define MOTOR0_ORG_PORT     GPIOB
#define MOTOR0_ORG_PIN      GPIO_PIN_9

#define MOTOR0_ORG_LEFT_POSITION    -136
#define MOTOR0_ORG_RIGHT_POSITION    136
//config
    
#define MOTOR_CALIBRATION_MEASURE_NO

#define WAKEUP         3
#define POSITION       2
#define ABSOLUTE       1
#define RELATIVE       0

#define HARD_RESET       0  //所有状态清零包括forward, round, step_position
#define SOFT_RESET       1  //仅复位step_position
    
typedef struct Motor_ Motor;
typedef void (*ControlMethod)(Motor* motor);
    
//由a,v来模拟计算s，并输出step，计算步长为dt, dt由所需的最大速度限制
//由于ds=dt*v， 需要ds<1(s单位为step),因此dt_max<1/v_max
//另外, 基于计算的优化, s的单位为step, t的单位为dt, a的单位为step/dt/dt, v的单位为step/dt
typedef struct Motor_
{
    uint8_t id;
    
//单位换算参数
    uint16_t    dt; //用于计算的时间间隔(StepRun()的调用间隔)单位us，不能小于输出STEP脉冲的周期（目前设置的是6us），并且能运行的最大速度取决于这个值，如果以dt为单位，则最大速度应该是1step/dt。而dt也是实际计算时所用的单位
    float       step2degree;
    float       degree2step;
    float       dt2s;
    float       s2dt;
    float       dps2spdt;       //速度单位转换，°/s -> step/dt
    float       spdt2dps;       //速度单位转换, step/dt -> °/s
    float       dpss2spdtdt;    //加速度单位转换, °/s/s -> step/dt/dt
    float       spdtdt2dpss;    //加速度单位转换, step/dt/dt -> °/s/s
    
//硬件配置参数
    TIM_TypeDef* tim; //用于输出脉冲的TIMER
    GPIO_TypeDef* dir_port; //用于输出方向的GPIO port
    GPIO_TypeDef* org_port; //用于复位和位置校准的GPIO Port
    GPIO_TypeDef*    sleep_port;    //用于输出Sleep的GPIO pin
    uint16_t    dir_pin;    //用于输出Sleep的GPIO pin
    uint16_t    org_pin;    //用于复位和位置校准的GPIO pin
    uint16_t    sleep_pin;    //用于复位和位置校准的GPIO pin
    GPIO_PinState dir_p;
    GPIO_PinState dir_n;
    
//运动参数
    uint16_t check_number;
    uint8_t org_state;
    uint8_t need_calibrate; //在运动过程中是否需要校准位置
    int16_t calibration_left;
    int16_t calibration_right;
    int16_t step_position_max;
    
//位置变量
    int16_t step_position;
    int16_t round;  //当前的圈数
    
//运行变量
    fixedpt a;    //当前的加速度
    fixedpt v;    //当前的速度
    fixedpt s;    //当前的位置 
    int32_t step;   //当前的步数,用于记录相对运动的距离,在相对运动开始时清零
    //int8_t dir;     //当前的方向,-1:负向 1:正向 0:未知
    //uint8_t enable;   //是否在使能运动，如果不使能，MotorRun不运作，以提高性能
    
    //uint16_t control_time_count;
    //uint16_t control_time;
    
//控制参数
    //fixedpt v_max;
    fixedpt target_v_a;
    fixedpt target_v;
    int32_t target_s;
    uint32_t target_s_v;

    //uint32_t stop_time;
    uint8_t hard_reset;
    //uint8_t is_control_end;
    uint8_t org_fault;  //在没有执行运动的时候触发复位信号
    uint8_t position_org_fault; //在运动时，触发复位信号，但是与当前位置差距超过阈值
    uint8_t position_fault; //在运动时，运动经过复位位置（一定的范围）,但是没有触发复位信号

    ControlMethod control_method;
} Motor;

__STATIC_INLINE void MotorSetTargetVA(Motor* motor, float acc)
{
    motor->target_v_a = fixedpt_formf(acc * motor->dpss2spdtdt);
}

__STATIC_INLINE void MotorSetTargetV(Motor* motor, float v)
{
    motor->target_v = fixedpt_formf(v * motor->dps2spdt);
}

__STATIC_INLINE void MotorSetTargetS(Motor* motor, float s)
{
    motor->target_s = s * motor->degree2step;
}

__STATIC_INLINE void MotorSetTargetSV(Motor* motor, float v)
{
    motor->target_s_v = fixedpt_formf(v * motor->dps2spdt);
}

/*__STATIC_INLINE void MotorSetLimit(Motor* motor, float v_max, float a_max, float v_min)
{
    motor->v_max = fixedpt_formf(v_max * motor->dps2spdt);
    motor->a_max = fixedpt_formf(a_max * motor->dpss2spdtdt);
    motor->v_min = fixedpt_formf(v_min * motor->dps2spdt);
}*/

__STATIC_INLINE void MotorDriverStart(TIM_TypeDef* tim, IRQn_Type irq)
{
    HAL_NVIC_SetPriority(irq, 0, 0); //最高优先级
    HAL_NVIC_EnableIRQ(irq); 
    tim->CR1|=(TIM_CR1_CEN); //开始
}

void MotorInit(Motor* motor, uint8_t id, TIM_TypeDef* tim, GPIO_TypeDef* dir_port, uint16_t    dir_pin,
                GPIO_TypeDef* org_port, uint16_t org_pin, GPIO_TypeDef* sleep_port, uint16_t sleep_pin, 
                GPIO_PinState dir_p, GPIO_PinState dir_n, 
                uint16_t dt, uint8_t big_gear, uint8_t small_gear, 
                float step_angle, float micro_step, int16_t org_left, int16_t org_right);

void MotorRun(Motor* motor);
//void MotorPositionCalibrate(Motor* motor);
__STATIC_INLINE void MotorSleep(Motor* motor)
{
    HAL_GPIO_WritePin(motor->sleep_port, motor->sleep_pin, GPIO_PIN_RESET);
}

__STATIC_INLINE void MotorWakeup(Motor* motor)
{
    HAL_GPIO_WritePin(motor->sleep_port, motor->sleep_pin, GPIO_PIN_SET);
}


__STATIC_INLINE float MotorGetPosition(Motor* motor)
{
    return motor->step_position * motor->step2degree;
}

__STATIC_INLINE uint8_t IsMotorStop(Motor* motor)
{
    return (motor->v == 0);
}

__STATIC_INLINE uint8_t MotorOrgState(Motor* motor)
{
    return HAL_GPIO_ReadPin(motor->org_port, motor->org_pin);
}


__STATIC_INLINE void MotorInitOrgState(Motor* motor)
{
    motor->org_state = MotorOrgState(motor);
}

__STATIC_INLINE void MotorStep(Motor* motor)
{
    motor->tim->CR1|=(TIM_CR1_CEN);
}

__STATIC_INLINE void MotorSetDirP(Motor* motor)
{
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, motor->dir_p);
}

__STATIC_INLINE void MotorSetDirN(Motor* motor)
{
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, motor->dir_n);
}

void MotorDriverInit(void);
void MoveRelative(Motor* motor, float degree, float speed);
void MoveAbsolute(Motor* motor, float degree, float speed);
void MovePosition(Motor* motor, float degree, float speed);
void MoveAngle(Motor* motor, float degree, float speed, uint8_t type);
void MoveSpeed(Motor* motor, float speed, float acc);
void MoveSpeedDirect(Motor* motor, float speed);
void MoveHardReset(Motor* motor, float speed);
void MoveSoftReset(Motor* motor, float speed);
void MoveReset(Motor* motor, float speed, uint8_t mode);
void RobodySetForward(float angle, uint8_t type);
float RobodyGetForward(void);
void MoveWakeup(Motor* motor, float angle, float speed);

extern Motor motor0;

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
