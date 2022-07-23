#include "Motor.h"
#include "Communication.h"
#include "main.h"

Motor motor0;
void MotorPositionCalibratePoll(Motor* motor);

static float WrapAngle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

static void MotorBspInit()
{
    //引脚配置
    /*
    PA15 - STEP	OUT
    PB3 - nSLEEP OUT
    PB4 - DIR OUT
    PB5 - M0 OUT
    PB6 - M1 OUT
    PB7 - nENBL OUT
    PB9 - ORG IN
    */
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 |GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_TIM2_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    //时钟配置
    __TIM2_CLK_ENABLE();
    TIM2->PSC = 63; //计时时钟为1MHz
    TIM2->CCR1 = 20;   //CH1
    TIM2->ARR = 40;
    BITSET0(TIM2->CCMR1, TIM_CCMR1_CC1S_0);BITSET0(TIM2->CCMR1, TIM_CCMR1_CC1S_1);  //CH1配置为输出
    BITSET1(TIM2->CCMR1, TIM_CCMR1_OC1M_0);BITSET1(TIM2->CCMR1, TIM_CCMR1_OC1M_1);BITSET1(TIM2->CCMR1, TIM_CCMR1_OC1M_2); //CH1 OC模式，计数值小于CCR时输出低，否则输出高
    BITSET0(TIM2->CR1, TIM_CR1_DIR); //配置为递增计数
    BITSET1(TIM2->CR1, TIM_CR1_OPM); //配置为单脉冲模式
    BITSET1(TIM2->CCER, TIM_CCER_CC1E); //使能输出,CH1
    BITSET1(TIM2->EGR, TIM_EGR_UG); //更新寄存器
    
    //初始化作为STEP控制时基的TIMER，设置为TIM7，预分频后CLK为1MHZ
    //三个电机均在一个TIMER控制下完成，减少进出中断的时间
    __TIM4_CLK_ENABLE();
    TIM4->PSC = 63; //计时时钟为1MHz
    TIM4->ARR = 100; //时间间隔为100us
    
    BITSET1(TIM4->CR1, TIM_CR1_URS);
    BITSET1(TIM4->DIER, TIM_DIER_UIE); //使能中断
    BITSET1(TIM4->EGR, TIM_EGR_UG);
    
    //global enable
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);//DIR
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);//SLEEP
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);//M0
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);//M1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);//EN
    
}

void MotorDriverInit()
{
    MotorBspInit();
    MotorInit(&motor0, 0,
            MOTOR0_STEP_TIM, 
            MOTOR0_DIR_PORT, 
            MOTOR0_DIR_PIN, 
            MOTOR0_ORG_PORT, 
            MOTOR0_ORG_PIN,        
            MOTOR0_SLEEP_PORT,
            MOTOR0_SLEEP_PIN,
            GPIO_PIN_RESET,
            GPIO_PIN_SET, 
            100, 72, 24, 11.25/16, 1, MOTOR0_ORG_LEFT_POSITION, MOTOR0_ORG_RIGHT_POSITION);
    MotorDriverStart(TIM4, TIM4_IRQn);
}

void MotorInit(Motor* motor, uint8_t id, TIM_TypeDef* tim, GPIO_TypeDef* dir_port, uint16_t    dir_pin, 
                GPIO_TypeDef* org_port, uint16_t org_pin, GPIO_TypeDef* sleep_port, uint16_t sleep_pin,
                GPIO_PinState dir_p, GPIO_PinState dir_n, 
                uint16_t dt, uint8_t big_gear, uint8_t small_gear, 
                float step_angle, float micro_step, int16_t org_left, int16_t org_right)
{
    motor->step2degree = step_angle / micro_step * small_gear / big_gear;
    motor->degree2step = 1.0f / motor->step2degree;
    
    motor->s2dt = 1000000.0f / dt;
    motor->dt2s = 1.0f / motor->s2dt;
    
    motor->dps2spdt = motor->degree2step / motor->s2dt;
    motor->spdt2dps = 1.0f / motor->dps2spdt;
    
    motor->dpss2spdtdt = motor->dps2spdt / motor->s2dt;
    motor->spdtdt2dpss = 1.0f / motor->dpss2spdtdt;
    
    motor->calibration_left = org_left;
    motor->calibration_right = org_right;
    motor->step_position_max = 180 * motor->degree2step;
    
    motor->step_position = 0;
    motor->round = 0;
    
    motor->tim = tim;
    motor->dir_port = dir_port;
    motor->dir_pin = dir_pin;
    motor->org_port = org_port;
    motor->org_pin = org_pin;
    motor->sleep_port = sleep_port;
    motor->sleep_pin = sleep_pin;
    motor->dir_p = dir_p;
    motor->dir_n = dir_n;
    motor->dt = dt;

    motor->need_calibrate = 1;
    motor->hard_reset = 0;
    
    motor->check_number = 0;

    motor->a = 0.0f;
    motor->v = 0.0f;
    motor->s = 0.0f;
    motor->step = 0;
    //motor->dir = 0;
    
    MotorInitOrgState(motor);

}



void MotorPositionController(Motor* motor)
{
    fixedpt v = motor->target_s_v;
    fixedpt ds = motor->target_s - motor->step;
    if(ds > 5)
    {
        motor->target_v = v;
    }
    else if(ds < -5)
    {
        motor->target_v = -v;
    }
    else
    {
        motor->target_v = 0;
        motor->control_method = NULL;
        MotorSleep(motor);
        TransmitMotorStop();
    }
}

void MotorPositionDirectController(Motor* motor)
{
    fixedpt v = motor->target_s_v;
    fixedpt target = motor->target_s;
    fixedpt s = motor->step;
    if(target > s)
    {
        motor->v = v;
    }
    else if(target < s)
    {
        motor->v = -v;
    }
    else
    {
        motor->v = 0;
        motor->control_method = NULL;
        MotorSleep(motor);
    }
}


static void MoveRelativeBase(Motor* motor, int32_t step, fixedpt v)
{
    MotorWakeup(motor);
    motor->target_s = step;
    motor->target_s_v = v;
    motor->step = 0;
    motor->control_method = MotorPositionDirectController;
} 

void MoveRelative(Motor* motor, float degree, float speed)
{
    MoveRelativeBase(motor, degree*motor->degree2step, fixedpt_formf(speed*motor->dps2spdt));
} 

void MoveAbsolute(Motor* motor, float degree, float speed)
{
    int32_t current_position = motor->step_position + motor->round * (motor->step_position_max << 1);
    int32_t move_position = degree*motor->degree2step - current_position;
    MoveRelativeBase(motor, move_position, fixedpt_formf(speed*motor->s2dt));
}

void MovePosition(Motor* motor, float degree, float speed)
{
    float current = motor->step_position * motor->step2degree;
    float move_angle = WrapAngle(degree) - current;
    
    if(move_angle < -180.0f)
    {
        move_angle = 360.0f + move_angle;
    }
    else if(move_angle < 180)
    {
        move_angle = move_angle;
    }
    else
    {
        move_angle = move_angle - 360;
    }
    MoveRelative(motor, move_angle, speed);
}

void MoveAngle(Motor* motor, float degree, float speed, uint8_t type)
{
    if(type == RELATIVE)
    {
        MoveRelative(motor, degree, speed);
    }
    else if(type == ABSOLUTE)
    {
        MoveAbsolute(motor, degree, speed);
    }
    else if(type == POSITION)
    {
        MovePosition(motor, degree, speed);
    }
    else if(type == WAKEUP)
    {
        MoveWakeup(motor, degree, speed);
    }
}

void MotorSpeedController(Motor* motor)
{
    fixedpt acc = motor->target_v_a;
    fixedpt dv = motor->target_v - motor->v;
    if(dv > acc)
    {
        motor->a = acc;
    }
    else if(dv < -acc)
    {
        motor->a = -acc;
    }
    else
    {
        motor->a = 0;
        motor->v = motor->target_v;
        motor->control_method = NULL;
        if(motor->v == 0)
        {
            MotorSleep(motor);
        }
    }
}

void MoveSpeedBase(Motor* motor, fixedpt speed, fixedpt acc)
{
    MotorWakeup(motor);
    motor->target_v = speed;
    motor->target_v_a = acc;
    motor->step = 0;
    motor->control_method = MotorSpeedController;
}

void MoveSpeed(Motor* motor, float speed, float acc)
{
    MoveSpeedBase(motor, fixedpt_formf(speed*motor->dps2spdt), fixedpt_formf(acc*motor->dpss2spdtdt));
}

void MoveSpeedDirect(Motor* motor, float speed)
{
    motor->v = fixedpt_formf(speed*motor->dps2spdt);
    if(motor->v == 0)
    {
        MotorSleep(motor);
    } 
    else
    {
        MotorWakeup(motor);
    }
    motor->control_method = NULL;
}

//每个dt调用,更新a,v,s, 如果需要输出step
void MotorRun(Motor* motor)
{
    fixedpt s1, v1;
    //fixedpt v_max = motor->v_max;
    fixedpt s = motor->s;
    fixedpt v = motor->v;
    fixedpt a = motor->a;

    //v = v > v_max? v_max : v;
    v1 = v + a;
    s1 = s + v;
    if(v>0)
    {
        MotorSetDirP(motor);
        if(s1<s)
        {
            MotorStep(motor);
            motor->step++;
            if(motor->step_position >= motor->step_position_max)
            {
                motor->step_position = - motor->step_position_max;
                motor->round ++;
            }
            else
            {
                motor->step_position ++;
            }
        }
    }
    else
    {
        MotorSetDirN(motor);
        if(s1>s)
        {
            MotorStep(motor);
            motor->step--;
            if(motor->step_position <= (- motor->step_position_max))
            {
                motor->step_position = motor->step_position_max;
                motor->round --;
            }
            else
            {
                motor->step_position --;
            }
        }
    }
    
    motor->s = s1;
    motor->v = v1;
    
    //MotorSpeedController(motor);
    
    if(motor->control_method != NULL) 
    {
        motor->control_method(motor);
    }
    
    MotorPositionCalibratePoll(motor);
}




#ifdef MOTOR_CALIBRATION_MEASURE
void MotorCalibrationMeasure(Motor* motor)
{
    if(HAL_GPIO_ReadPin(motor->org_port, motor->org_pin) == GPIO_PIN_SET)
    {
        //上升沿
        if(motor->dir > 0)
        {
            //正向
            motor->calibration_left = motor->step_position;
        }
        else if(motor->dir < 0)
        {
            //负向
            motor->calibration_right = motor->step_position;
        }
    }
    else
    {
        //下降沿
        if(motor->dir > 0)
        {
            //正向
            motor->calibration_right  = motor->step_position;
        }
        else if(motor->dir < 0)
        {
            //负向
            motor->calibration_left = motor->step_position;
        }
    }
}
#endif

void MotorPositionCalibratePoll(Motor* motor)
{
    uint8_t state = MotorOrgState(motor);
    uint8_t change = (motor->org_state != state);
    if(change) 
    {
        motor->check_number++;
        if(motor->check_number>200)
        {
            motor->org_state = state;
            //沿触发
            if(IsMotorStop(motor))
            {
                //非运动状态不进行位置校正
                motor->org_fault = 1;
                return;
            }
            
            //在运动时才校正位置
            if(state > 0)
            {
                //上升沿
                if(motor->v > 0)
                {
                    //正向
                    #ifdef MOTOR_CALIBRATION_MEASURE
                    motor->calibration_left = motor->step_position;
                    #else
                    motor->step_position = motor->calibration_left;
                    #endif
                }
                else if(motor->v < 0)
                {
                    //负向
                    #ifdef MOTOR_CALIBRATION_MEASURE
                    motor->calibration_right = motor->step_position;
                    #else
                    motor->step_position = motor->calibration_right;
                    #endif
                }
            }
            else
            {
                //下降沿
                if(motor->v > 0)
                {
                    //正向
                    #ifdef MOTOR_CALIBRATION_MEASURE
                    motor->calibration_right = motor->step_position;
                    #else
                    motor->step_position = motor->calibration_right;
                    #endif
                    
                }
                else if(motor->v < 0)
                {
                    //负向
                    #ifdef MOTOR_CALIBRATION_MEASURE
                    motor->calibration_left = motor->step_position;
                    #else
                    motor->step_position = motor->calibration_left;
                    #endif
                }
            }
            
            if(motor->hard_reset)
            {
                motor->hard_reset = 0;
                motor->round = 0;
                MovePosition(motor, 0, 0);
            }
        }
    }
    else
    {
        motor->check_number = 0;
    }
}

//复位
void MoveHardReset(Motor* motor, float v)
{
    if(motor->hard_reset)
    {
        return;
    }
    
    if(motor->step_position < 0.0f)
    {
        motor->v = fixedpt_formf(v*motor->dps2spdt);
    }
    else
    {
        motor->v = -fixedpt_formf(v*motor->dps2spdt);
    }
    
    motor->hard_reset = 1;
}

void MoveSoftReset(Motor* motor, float v)
{
    MovePosition(motor, 0, v);
}

void MoveReset(Motor* motor, float speed, uint8_t type)
{
    if(type == HARD_RESET)
    {
        MoveHardReset(motor, speed);
    }
    else if(type == SOFT_RESET)
    {
        MoveSoftReset(motor, speed);
    }
}

float MotorPosition(Motor* motor)
{
    return motor->step_position * motor->step2degree;
}


float forward_angle = 0.0f;
extern Motor motor0;

static void RobodySetForwardAbsolute(float angle)
{
    //通过当前的正向位置，计算当前电机位置的绝对位置
    float current_angle;
    int32_t forward_angle_step;
    angle = WrapAngle(angle);
    current_angle = motor0.step_position * motor0.step2degree + forward_angle;
    forward_angle = angle;	//更新新的正向位置
    current_angle -= angle;	//当前位置
    current_angle = WrapAngle(current_angle);
    forward_angle_step = angle * motor0.degree2step;
    motor0.step_position = current_angle * motor0.degree2step; //更新当前位置
    motor0.calibration_left = MOTOR0_ORG_LEFT_POSITION - forward_angle_step;
    motor0.calibration_right = MOTOR0_ORG_RIGHT_POSITION - forward_angle_step;	
}

static void RobodySetForwardRelative(float angle)
{
    RobodySetForwardAbsolute(forward_angle + angle);
}
//设置正位角度, angle -180-180, type:0-相对;1-绝对
void RobodySetForward(float angle, uint8_t type)
{
    if(type==ABSOLUTE)
    {
        RobodySetForwardAbsolute(angle);
    }
    else
    {
        RobodySetForwardRelative(angle);
    }
}

float RobodyGetForward()
{
    return forward_angle;
}

void MoveWakeup(Motor* motor, float angle, float speed)
{
    RobodySetForwardRelative(MotorGetPosition(motor) + angle);
    MoveSoftReset(motor, speed);
}
