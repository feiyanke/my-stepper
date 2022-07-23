#include "Command.h"
#include "Motor.h"

void CmdMoveEnableExecute(CmdMoveEnable* cmd)
{
    Motor* motor;
    uint8_t motor_id = cmd->motor_id;
    uint8_t enable = cmd->enable;
    if(motor_id<3)
    {
        motor = &Motors[motor_id];
        MotorRunSetEnable(motor, enable);
    }
}

void CmdMoveRelativeExecute(CmdMoveRelative* cmd)
{
    Motor* motor;
    float target;
    float time;
    uint8_t motor_id = cmd->motor_id;
    if(motor_id<3)
    {
        motor = &Motors[motor_id];
        target = cmd->angle;
        time = cmd->time;
        MoveRelative(motor, target, time);
    }
}

void CmdMoveAbsoluteExecute(CmdMoveAbsolute* cmd)
{
    Motor* motor;
    float target;
    float time;
    uint8_t motor_id = cmd->motor_id;
    if(motor_id<3)
    {
        motor = &Motors[motor_id];
        target = cmd->angle;
        time = cmd->time;
        MoveAbsolute(motor, target, time);
    }
}

void CmdMovePositionExecute(CmdMovePosition* cmd)
{
    Motor* motor;
    float target;
    float time;
    uint8_t motor_id = cmd->motor_id;
    if(motor_id<3)
    {
        motor = &Motors[motor_id];
        target = cmd->angle;
        time = cmd->time;
        MovePosition(motor, target, time);
    }
}

/*void CmdMoveFrameExecute(CmdMoveFrame* cmd)
{
    uint16_t frame = cmd->frame;
    uint8_t type = cmd->type;
    uint8_t tween = cmd->tween;
    MoveByFrame(&Motors[0], frame, cmd->motor0, tween, type);
    MoveByFrame(&Motors[1], frame, cmd->motor1, tween, type);
    MoveByFrame(&Motors[2], frame, cmd->motor2, tween, type);
}*/

void CmdMoveByTimeExecute(CmdMoveByTime* cmd)
{
    uint16_t time_ms = cmd->time_ms;
    uint8_t type = cmd->type;
    uint8_t tween = cmd->tween;
    //float s0 = cmd->motor0 * Motors[0].degree2step;
    //float s1 = cmd->motor1 * Motors[1].degree2step;
    //float s2 = cmd->motor2 * Motors[2].degree2step;
    MoveByTime(&Motors[0], time_ms, cmd->motor0, tween, type);
    MoveByTime(&Motors[1], time_ms, cmd->motor1, tween, type);
    MoveByTime(&Motors[2], time_ms, cmd->motor2, tween, type);
}

void CmdMoveResetExecute(CmdMoveReset* cmd)
{
    //float v = cmd->velocity;
    //float v0 = v * Motors[0].degree2step;
    //float v1 = v * Motors[1].degree2step;
    //float v2 = v * Motors[2].degree2step;
    //MotorHardReset(&Motors[0], v);
    //MotorHardReset(&Motors[1], v);
    //MotorHardReset(&Motors[2], v);
    
    Motor* motor;
    uint8_t mode = cmd->mode;
    uint8_t motor_id = cmd->motor_id;
    if(motor_id<3)
    {
        motor = &Motors[motor_id];
        if(mode == MOVE_RESET_MODE_HARD)
        {
            MotorHardReset(motor);
        }
        else if(mode == MOVE_RESET_MODE_SOFT)
        {
            MotorSoftReset(motor);
        }
        else if(mode == MOVE_RESET_MODE_ROUND)
        {
            motor->round = 0;
        }
    }
}

/*void CmdMoveRoundResetExecute(CmdMoveRoundReset* cmd)
{
    Motor* motor;
    uint8_t motor_id = cmd->motor_id;
    if(motor_id<3)
    {
        motor = &Motors[motor_id];
        motor->round = 0;
    }
}*/

void CmdMoveSpeedExecute(CmdMoveSpeed* cmd)
{
    Motor* motor;
    float v;
    uint8_t motor_id = cmd->motor_id;
    if(motor_id<3)
    {
        motor = &Motors[motor_id];
        v = cmd->velocity;
        MoveSpeedControl(motor, v);
    }
}

void CmdMoveSpeedDirectExecute(CmdMoveSpeed* cmd)
{
    Motor* motor;
    float v;
    uint8_t motor_id = cmd->motor_id;
    if(motor_id<3)
    {
        motor = &Motors[motor_id];
        v = cmd->velocity;
        MoveSpeedDirectControl(motor, v);
    }
}

void CmdForwardSettingExecute(CmdForwardSetting* cmd)
{
    RobodySetForward(cmd->angle, cmd->type);
}

void CmdMoveAccelerateSettingExecute(CmdMoveAccelerateSetting* cmd)
{
    Motor* motor;
    uint8_t motor_id = cmd->motor_id;
    if(motor_id<3)
    {
        motor = &Motors[motor_id];
        MoveAccelerateSetting(motor, cmd->acc);
    }
}

void CmdMoveLimitSettingExecute(CmdMoveLimitSetting* cmd)
{
    uint8_t i;
    for(i=0;i<3;i++)
    {
        Motors[i].v_max = cmd->v_limit[i] * Motors[i].dps2spdt;
        Motors[i].a_max = cmd->a_limit[i] * Motors[i].dpss2spdtdt;
        
    }
}

void CmdMoveAttitudeExecute(CmdMoveAttitude* cmd)
{
    if(cmd->part == 0)
    {
        MovePitchSpeed(cmd->v);
    }
    else
    {
        MoveYawSpeed(cmd->v);
    }
}

void CmdMovePitchExecute(CmdMovePitch* cmd)
{
    MovePitchAbsolute(cmd->position, cmd->v);
}
