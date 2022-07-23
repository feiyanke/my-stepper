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

#define HARD_RESET       0  //����״̬�������forward, round, step_position
#define SOFT_RESET       1  //����λstep_position
    
typedef struct Motor_ Motor;
typedef void (*ControlMethod)(Motor* motor);
    
//��a,v��ģ�����s�������step�����㲽��Ϊdt, dt�����������ٶ�����
//����ds=dt*v�� ��Ҫds<1(s��λΪstep),���dt_max<1/v_max
//����, ���ڼ�����Ż�, s�ĵ�λΪstep, t�ĵ�λΪdt, a�ĵ�λΪstep/dt/dt, v�ĵ�λΪstep/dt
typedef struct Motor_
{
    uint8_t id;
    
//��λ�������
    uint16_t    dt; //���ڼ����ʱ����(StepRun()�ĵ��ü��)��λus������С�����STEP��������ڣ�Ŀǰ���õ���6us�������������е�����ٶ�ȡ�������ֵ�������dtΪ��λ��������ٶ�Ӧ����1step/dt����dtҲ��ʵ�ʼ���ʱ���õĵ�λ
    float       step2degree;
    float       degree2step;
    float       dt2s;
    float       s2dt;
    float       dps2spdt;       //�ٶȵ�λת������/s -> step/dt
    float       spdt2dps;       //�ٶȵ�λת��, step/dt -> ��/s
    float       dpss2spdtdt;    //���ٶȵ�λת��, ��/s/s -> step/dt/dt
    float       spdtdt2dpss;    //���ٶȵ�λת��, step/dt/dt -> ��/s/s
    
//Ӳ�����ò���
    TIM_TypeDef* tim; //������������TIMER
    GPIO_TypeDef* dir_port; //������������GPIO port
    GPIO_TypeDef* org_port; //���ڸ�λ��λ��У׼��GPIO Port
    GPIO_TypeDef*    sleep_port;    //�������Sleep��GPIO pin
    uint16_t    dir_pin;    //�������Sleep��GPIO pin
    uint16_t    org_pin;    //���ڸ�λ��λ��У׼��GPIO pin
    uint16_t    sleep_pin;    //���ڸ�λ��λ��У׼��GPIO pin
    GPIO_PinState dir_p;
    GPIO_PinState dir_n;
    
//�˶�����
    uint16_t check_number;
    uint8_t org_state;
    uint8_t need_calibrate; //���˶��������Ƿ���ҪУ׼λ��
    int16_t calibration_left;
    int16_t calibration_right;
    int16_t step_position_max;
    
//λ�ñ���
    int16_t step_position;
    int16_t round;  //��ǰ��Ȧ��
    
//���б���
    fixedpt a;    //��ǰ�ļ��ٶ�
    fixedpt v;    //��ǰ���ٶ�
    fixedpt s;    //��ǰ��λ�� 
    int32_t step;   //��ǰ�Ĳ���,���ڼ�¼����˶��ľ���,������˶���ʼʱ����
    //int8_t dir;     //��ǰ�ķ���,-1:���� 1:���� 0:δ֪
    //uint8_t enable;   //�Ƿ���ʹ���˶��������ʹ�ܣ�MotorRun�����������������
    
    //uint16_t control_time_count;
    //uint16_t control_time;
    
//���Ʋ���
    //fixedpt v_max;
    fixedpt target_v_a;
    fixedpt target_v;
    int32_t target_s;
    uint32_t target_s_v;

    //uint32_t stop_time;
    uint8_t hard_reset;
    //uint8_t is_control_end;
    uint8_t org_fault;  //��û��ִ���˶���ʱ�򴥷���λ�ź�
    uint8_t position_org_fault; //���˶�ʱ��������λ�źţ������뵱ǰλ�ò�೬����ֵ
    uint8_t position_fault; //���˶�ʱ���˶�������λλ�ã�һ���ķ�Χ��,����û�д�����λ�ź�

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
    HAL_NVIC_SetPriority(irq, 0, 0); //������ȼ�
    HAL_NVIC_EnableIRQ(irq); 
    tim->CR1|=(TIM_CR1_CEN); //��ʼ
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
