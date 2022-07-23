#include "Control.h"
#include <math.h>
#include "arm_math.h"
#include "Motor.h"
//#include "Utility.h"
//#include "Application.h"

//运动控制结束后回调
static void ControlEnd(Motor* motor)
{
    //motor->control_method_per_frame = NULL;
    ClearTimeController(motor);
    ClearClkController(motor);
    //motor->is_control_end = 1;
    motor->v_co = 0.0f;
}

//速度控制
//加减速控制：以给定的加速度加速或减速
/*void AccelerateControlMethod(Motor* motor)
{
    float a1, a_max;
    a1 = motor->a;
    a_max = motor->a_max;
    if(a1 > a_max)
    {
        a1 = a_max;
    }
    else if(a1 < -a_max)
    {
        a1 = -a_max;
    }
    motor->a = a1;
    
    motor->v += motor->a;
}*/
//速度控制模式1：以给定加速度加速或减速到给定速度
void SpeedControlMethod(Motor* motor)
{
    float dv = motor->speed_controller.target_v - motor->v;
    if(dv > 0.0001f)
    {
        motor->a = motor->speed_controller.acc;
        //MotorPowerControl();
    }
    else if(dv < - 0.0001f)
    {
        motor->a = -motor->speed_controller.acc;
        //MotorPowerControl();
    }
    else
    {
        motor->a = 0.0f;
        motor->v = motor->speed_controller.target_v;
        ControlEnd(motor);
    }
    //AccelerateControlMethod(motor);
}

void SpeedControlEndlessMethod(Motor* motor)
{
    float dv = motor->speed_controller.target_v - motor->v;
    if(dv > 0.0001f)
    {
        motor->a = motor->speed_controller.acc;
        //MotorPowerControl();
    }
    else if(dv < - 0.0001f)
    {
        motor->a = -motor->speed_controller.acc;
        //MotorPowerControl();
    }
    else
    {
        motor->a = 0.0f;
        motor->v = motor->speed_controller.target_v;
    }
    //AccelerateControlMethod(motor);
}

//acc:°/s/s, v:°/s
void MoveSpeedControl(Motor* motor, float v)
{
    /*if(motor->hard_reset)
    {
        return;
    }*/
    
    motor->v_co = 0.0f;
    motor->speed_controller.target_v = v * motor->dps2spdt;
    //motor->speed_controller.acc = acc * motor->dpss2spdtdt;
    
    //motor->control_method_per_frame = NULL;
    ClearTimeController(motor);
    SetClkController(motor, SpeedControlMethod);
    //motor->is_control_end = 0;
    
    //MotorSetDAC(MOTOR_VREF_RUN_DAC);
    //MotorPowerControl();
}

void MoveSpeedEndlessControl(Motor* motor, float v)
{
    motor->v_co = 0.0f;
    motor->speed_controller.target_v = v * motor->dps2spdt;
    //motor->speed_controller.acc = acc * motor->dpss2spdtdt;
    
    //motor->control_method_per_frame = NULL;
    ClearTimeController(motor);
    SetClkController(motor, SpeedControlEndlessMethod);
    //motor->is_control_end = 0;
    
    //MotorSetDAC(MOTOR_VREF_RUN_DAC);
    //MotorPowerControl();
}

void MoveSpeedDirectControl(Motor* motor, float v)
{
    motor->v_co = 0.0f;
    motor->v = v * motor->dps2spdt;
    ClearClkController(motor);
    ClearTimeController(motor);
}

//位置控制模式2：指定时间的位置运动，与帧运动方式一致，只是计算间隔变为1ms
void MoveByTimeControlMethod(Motor* motor)
{
    // 计算下一ms应该运行到的位置，并计算出需要的速度，然后更新v
    float t,v;
    uint16_t time_ms, total_time_ms;
    time_ms = motor->time_controller.current_ms;
    total_time_ms = motor->time_controller.total_ms;
    //t_all = total_frame / 30.0f;
    if(motor->time_controller.tween == TWEEN_SINE_IN_OUT)
    {
        if(time_ms >= total_time_ms) {
            motor->v=0.0f;
            //MsTimerEnd();
            ControlEnd(motor);
            return;
        }
        else
        {
            //t = (float)frame;
            t = (time_ms + 0.5f) / 1000.0f;
            v = motor->time_controller.param1 * arm_sin_f32(motor->time_controller.param2*t);
            motor->v = v/motor->s2dt;
        }
    }
    else if(motor->time_controller.tween == TWEEN_QUAD_IN_OUT)
    {
        if(time_ms >= total_time_ms) {
            motor->v=0.0f;
            //MsTimerEnd();
            ControlEnd(motor);
            return;
        }
        else
        {
            if(time_ms < (total_time_ms>>1))
            {
                t = (time_ms + 0.5f) / 1000.0f;
            }
            else
            {
                t = (total_time_ms - time_ms - 0.5f) / 1000.0f;
            }
            v = motor->time_controller.param1 * t;
            motor->v = v/motor->s2dt;
        }
    }
    else if(motor->time_controller.tween == TWEEN_CUBIC_IN_OUT)
    {
        if(time_ms >= total_time_ms) {
            motor->v=0.0f;
            //MsTimerEnd();
            ControlEnd(motor);
            return;
        }
        else
        {
            if(time_ms < (total_time_ms>>1))
            {
                t = (time_ms + 0.5f) / 1000.0f;
            }
            else
            {
                t = (total_time_ms - time_ms - 0.5f) / 1000.0f;
            }
            v = motor->time_controller.param1 * t * t;
            motor->v = v/motor->s2dt;
        }
    }
    motor->time_controller.current_ms++;

}

//target_s:step
static void MoveByTimeRelativeBase(Motor* motor, uint16_t time_ms, float target_s, uint8_t tween)
{
    float t = time_ms / 1000.0f;
    
    /*if(motor->hard_reset)
    {
        return;
    }*/
    
    motor->v_co = 0.0f;
    motor->time_controller.total_ms = time_ms;
    motor->time_controller.tween = tween;
    motor->time_controller.target_s = target_s;
    motor->time_controller.current_ms = 0;
    //motor->control_method_per_frame = NULL;
    
    //参数计算，以优化速度
    if(tween == TWEEN_SINE_IN_OUT)
    {
        motor->time_controller.param1 = PI*target_s/t/2.0f;
        motor->time_controller.param2 = PI/t;
    }
    else if(tween == TWEEN_QUAD_IN_OUT)
    {
        motor->time_controller.param1 = 4.0f*target_s/t/t;
    }
    else if(tween == TWEEN_CUBIC_IN_OUT)
    {
        motor->time_controller.param1 = 12.0f*target_s/t/t/t;
    }
    
    //MsTimerStart();
    //motor->ms_count = 0;
    SetTimeController(motor, MoveByTimeControlMethod, 400);
    ClearClkController(motor);
    //motor->is_control_end = 0;
    
    //MotorSetDAC(MOTOR_VREF_RUN_DAC);
    //MotorPowerControl();
    
    MoveByTimeControlMethod(motor);
}

//target_angle:°
static void MoveByTimeRelative(Motor* motor, uint16_t time_ms, float target_angle, uint8_t tween)
{
    MoveByTimeRelativeBase(motor, time_ms, target_angle * motor->degree2step, tween);
}

//target_angle:°
static void MoveByTimeAbsolute(Motor* motor, uint16_t time_ms, float target_angle, uint8_t tween)
{
    float target_s = target_angle * motor->degree2step;
    float current_position = motor->step_position + motor->round * motor->step_position_max * 2.0f;
    float move_position = target_s - current_position;
    MoveByTimeRelativeBase(motor, time_ms, move_position, tween);
}

//target_angle:°
void MoveByTime(Motor* motor, uint16_t time_ms, float target_angle, uint8_t tween, uint8_t type)
{
    if(type == ABSOLUTE)
    {
        MoveByTimeAbsolute(motor, time_ms, target_angle, tween);
    }
    else
    {
        MoveByTimeRelative(motor, time_ms, target_angle, tween);
    }
}

void PositionControlMethod1_(Motor* motor)
{
    float v,s,t,a,acc_max,acc_s,v_min;
    uint8_t flg;
    v = motor->v;
    s = motor->position_controller.target - motor->step;
    acc_max = motor->a_max;
    v_min = motor->v_min;
    t = motor->position_controller.dt;
    
    if(t<motor->control_time)
    {
        motor->position_controller.dt=0;
    }
    else
    {
        motor->position_controller.dt-=motor->control_time;
    }

    flg = (v<=v_min)&&(v>=-v_min);
    
    if((s<40)&&(s>-40)&&flg)
    {
        motor->a = 0.0f;
        motor->v = 0.0f;
        ControlEnd(motor);
        return;
    }
    
    if(v==0.0f)
    {
        if(s>0.0f)
        {
            motor->v = v_min;
        }
        else
        {
            motor->v = -v_min;
        }
        motor->a = 0.0f;
        return;
    }
    
    if(flg&&(s*v<0.0f))
    {
        if(v>0.0f)
        {
            motor->v = -v_min;
        }
        else
        {
            motor->v = v_min;
        }
        motor->a = 0.0f;
        return;
    }

    if(t==0)
    {
        if(s>0.0f&&v<=0.0f)
        {
            a = acc_max;
        }
        else if(s<0.0f&&v>=0.0f)
        {
            a = -acc_max;
        }
        else
        {
            //直接减速到0，运动的距离
            acc_s = v * v / acc_max / 2.0f;
            if(acc_s < fabsf(s))
            {
                //加速
                if(s > 0)
                {
                    a = acc_max;
                }
                else
                {
                    a = -acc_max;
                }
            }
            else
            {
                //减速
                if(s > 0)
                {
                    a = -acc_max;
                }
                else
                {
                    a = acc_max;
                }
            }
        }
    }
    else
    {
        a = (6.0f*s-4.0f*v*t)/(t*t);
        if(fabsf(a)>acc_max)
        {
            if(s>0&&v<=0)
            {
                a = acc_max;
            }
            else if(s<0&&v>=0)
            {
                a = -acc_max;
            }
            else
            {
                //直接减速到0，运动的距离
                acc_s = v * v / acc_max / 2.0f;
                if(fabsf(s)>acc_s)
                {
                    //加速
                    if(s > 0)
                    {
                        a = acc_max;
                    }
                    else
                    {
                        a = -acc_max;
                    }
                }
                else
                {
                    //减速
                    if(s > 0)
                    {
                        a = -acc_max;
                    }
                    else
                    {
                        a = acc_max;
                    }
                }
            }
        }
    }
    
    if(flg&&(v*a<0.0f))
    {
        if(v>0.0f)
        {
            motor->v = v_min;
        }
        else
        {
            motor->v = -v_min;
        }
        motor->a = 0.0f;
    }
    else
    {
        motor->a = a;
    }
}

void PositionControlMethod1(Motor* motor)
{
    float v,s,t,a,acc_max,acc_s,v_min,tt;
    v = motor->v;
    s = motor->position_controller.target - motor->step;
    acc_max = motor->a_max;
    v_min = motor->v_min;
    t = motor->position_controller.dt;
    
    if(t<motor->control_time)
    {
        motor->position_controller.dt=0;
    }
    else
    {
        motor->position_controller.dt-=motor->control_time;
    }
    
    if((v<=v_min)&&(v>=-v_min))
    {
        if((s<40)&&(s>-40))
        {
            motor->a = 0.0f;
            motor->v = 0.0f;
            ControlEnd(motor);
            return;
        }
        else
        {
            tt = fabsf(s)/v_min;
            if(s>0.0f)
            {
                v = v_min;
            }
            else
            {
                v = -v_min;
            }
            
            if(t>tt)
            {
                motor->v = v;
                motor->a = 0.0f;
                return;
            }
        }
    }

    if(t==0)
    {
        if(s>0.0f&&v<=0.0f)
        {
            a = acc_max;
        }
        else if(s<0.0f&&v>=0.0f)
        {
            a = -acc_max;
        }
        else
        {
            //直接减速到0，运动的距离
            acc_s = v * v / acc_max / 2.0f;
            if(acc_s < fabsf(s))
            {
                //加速
                if(s > 0)
                {
                    a = acc_max;
                }
                else
                {
                    a = -acc_max;
                }
            }
            else
            {
                //减速
                if(s > 0)
                {
                    a = -acc_max;
                }
                else
                {
                    a = acc_max;
                }
            }
        }
    }
    else
    {
        a = (6.0f*s-4.0f*v*t)/(t*t);
        if(fabsf(a)>acc_max)
        {
            if(s>0&&v<=0)
            {
                a = acc_max;
            }
            else if(s<0&&v>=0)
            {
                a = -acc_max;
            }
            else
            {
                //直接减速到0，运动的距离
                acc_s = v * v / acc_max / 2.0f;
                if(fabsf(s)>acc_s)
                {
                    //加速
                    if(s > 0)
                    {
                        a = acc_max;
                    }
                    else
                    {
                        a = -acc_max;
                    }
                }
                else
                {
                    //减速
                    if(s > 0)
                    {
                        a = -acc_max;
                    }
                    else
                    {
                        a = acc_max;
                    }
                }
            }
        }
    }
    
    motor->a = a;
}


//位置控制模式1：给定加速度，运行时间和目标位置，在任意初始速度下，以梯形速度曲线运动，到达指定目标?
void PositionControlMethod(Motor* motor)
{
    float time, v, acc, s, target_s, acc_time, t, v_min, tt;
    int32_t step;
    step = motor->position_controller.target - motor->step;
    v = motor->v;
    v_min = motor->v_min;
    acc = motor->position_controller.acc;//必须大于0
    s = (float)step;
    t = motor->position_controller.dt;
    
    if(t<motor->control_time)
    {
        motor->position_controller.dt=0;
    }
    else
    {
        motor->position_controller.dt-=motor->control_time;
    }
    
    if((v<=v_min)&&(v>=-v_min))
    {
        if((s<40.0f)&&(s>-40.0f))
        {
            motor->a = 0.0f;
            motor->v = 0.0f;
            ControlEnd(motor);
            return;
        }
        else
        {
            tt = fabsf(s)/v_min;
            if(s>0.0f)
            {
                v = v_min;
            }
            else
            {
                v = -v_min;
            }
            
            if(t>tt)
            {
                motor->v = v;
                motor->a = 0.0f;
                return;
            }
        }
    }
 
    if((step<0)&&(v>0.0f)) //如果当前速度方向与目标方向相反则加速
    {
        motor->a = -acc;
        //MotorPowerControl();
    }
    else if((step>0)&&(v<0.0f))
    {
        motor->a = acc;
        //MotorPowerControl();
    }
    else
    {
        //target_s = fabsf(step);
        if(step < 0)
        {
            target_s = -step;
        }
        else
        {
            target_s = step;
        }
        
        if(v < 0.0f)
        {
            v = -v;
        }
        //直接减速到0，运动的距离
        acc_time = v / acc / 2.0f;
        s = v * acc_time;
        if(target_s > s)
        {
            //计算以当前速度，运动到目标的最短时间
            t = target_s/v + acc_time;
            time = motor->position_controller.dt;
            if(t > time)
            {
                //加速
                if(step > 0)
                {
                    motor->a = acc;
                    //MotorPowerControl();
                }
                else
                {
                    motor->a = -acc;
                    //MotorPowerControl();
                }
            }
            else
            {
                //匀速
                motor->a = 0.0f;
            }
        }
        else
        {
            //减速
            if(step > 0)
            {
                motor->a = -acc;
                //MotorPowerControl();
            }
            else
            {
                motor->a = acc;
                //MotorPowerControl();
            }
        }
    }
}

//acc: °/s/s
/*void MovePositionAccSet(Motor* motor, float acc)
{
    motor->position_controller.acc = acc * motor->dpss2spdtdt;
}*/

ControlMethod MoveControlMethod = PositionControlMethod1;

void MoveModeSetting(uint8_t mode)
{
    if(mode==0)
    {
        MoveControlMethod = PositionControlMethod;
    }
    else
    {
        MoveControlMethod = PositionControlMethod1;
    }
}

uint8_t GetMoveModeSetting()
{
    if(MoveControlMethod == PositionControlMethod)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}


//基本的相对运动
//target:step
static void MoveRelativeBase(Motor* motor, int32_t step, uint32_t dt)
{
    /*if(motor->hard_reset)
    {
        return;
    }*/
    motor->v_co = 0.0f;
    motor->step = 0;
    motor->position_controller.target = step;
    motor->position_controller.dt = dt;
    SetTimeController(motor, MoveControlMethod, 40);
    ClearClkController(motor);
    //motor->is_control_end = 0;
} 

//target:°
void MoveRelative(Motor* motor, float target, float time)
{
    MoveRelativeBase(motor, target*motor->degree2step, time*motor->s2dt);
} 

//target:°
void MoveAbsolute(Motor* motor, float target, float time)
{
    int32_t current_position = motor->step_position + motor->round * motor->step_position_max * 2.0f;
    int32_t move_position = target*motor->degree2step - current_position;
    MoveRelativeBase(motor, move_position, time*motor->s2dt);
}

//不考虑圈数的最近距离绝对运动
//target：°
void MovePosition(Motor* motor, float target, float time)
{
    float current = motor->step_position * motor->step2degree;
    float move_angle = WrapAngle(target) - current;
    
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
    MoveRelative(motor, move_angle, time);
}

static uint16_t RunDAC = MOTOR_VREF_RUN_DAC;

void MotorRunDacSetting(uint16_t dac)
{
    RunDAC = dac;
}

uint16_t GetMotorRunDacSetting()
{
    return RunDAC;
}
